# pfn_bittrue_sim.py
# Bit-true functional simulation for your pfn_layer.v
# No timing simulation, only arithmetic behavior.

from __future__ import annotations
import math
from typing import List


# ============================================================
# 0. Parameters aligned with your RTL
# ============================================================
EXPAND_PT_DIM = 9
OUT_PT_DIM = 64
PT_WIDTH = 16
WEIGHT_WIDTH = 16
ACC_WIDTH = 32

# RTL:
# dequant_temp[j] = st3_mac_tree[j] * 779;
# relu_comb[j]    = dequant_temp[j][PT_WIDTH+20-1 : 20];
DEQUANT_MUL = 3482
DEQUANT_SHIFT = 28

# In your RTL debug print:
# dim2 and dim6 are Q4.12, others are Q1.15
Q412_DIMS = {2, 6}

# Change these to your real .mem paths.
WEIGHT_MEM_PATH = r"E:\mmdetection3d\my_output_parameters\pfn_layer_fused_int_kitti\mem\pfn_weight_2.mem"
BIAS_MEM_PATH = r"E:\mmdetection3d\my_output_parameters\pfn_layer_fused_int_kitti\mem\bias.mem"

# If your weight.mem is exported according to RTL slicing:
# weight_row_rom[oc][i*16 +: 16], then dim0 is at LSB.
WEIGHT_DIM0_AT_LSB = False


# ============================================================
# 1. Direct input: all points inside one voxel
#    Order: dim0, dim1, ..., dim8
# ============================================================
points_in_voxel = [
    # [-0.065430, -0.070313, 0.171875, 0.479980, 0.000000, 0.000000, 0.000000, -0.065430, -0.070313],
    [-0.0650, -0.0700, 0.1720, 0.4800, 0.0000, 0.0000, 0.0000, -0.0650, -0.0700],
    # Add more points here:
    # [dim0, dim1, dim2, dim3, dim4, dim5, dim6, dim7, dim8],
]


# ============================================================
# 2. Bit-level helpers
# ============================================================
def mask(width: int) -> int:
    return (1 << width) - 1


def u(val: int, width: int) -> int:
    """Unsigned truncation to width bits."""
    return val & mask(width)


def s(val: int, width: int) -> int:
    """Interpret val as signed two's complement of given width."""
    val &= mask(width)
    sign_bit = 1 << (width - 1)
    return val - (1 << width) if (val & sign_bit) else val


def add_s32(a: int, b: int) -> int:
    """Verilog-like signed 32-bit addition with wrap-around."""
    return s(u(a, 32) + u(b, 32), 32)


def mul_s32(a: int, b: int) -> int:
    """int16 * int16 assigned into signed [31:0]."""
    return s(a * b, 32)


def fmt_hex(val: int, width: int) -> str:
    return f"0x{u(val, width):0{width // 4}x}"


def round_half_away_from_zero(x: float) -> int:
    """
    Used to reconstruct fixed-point integers from decimal float values.
    If your decimal values are already printed from RTL CSV with 6 decimals,
    nearest rounding usually recovers the original int16.
    """
    if x >= 0:
        return int(math.floor(x + 0.5))
    else:
        return int(math.ceil(x - 0.5))


def quantize_pfe_float_to_int16(value: float, dim: int) -> int:
    """
    Convert input decimal value to the int16 value seen by pfn_layer.

    dim2, dim6 : Q4.12, scale = 4096
    others     : Q1.15, scale = 32768
    """
    scale = 4096.0 if dim in Q412_DIMS else 32768.0
    q = round_half_away_from_zero(value * scale)
    return s(q, 16)


def output_q8_8_to_float(q: int) -> float:
    """
    RTL output CSV decodes m_axis_pfn_data as signed / 256.0.
    ReLU output should be non-negative, but signed interpretation is kept here.
    """
    return s(q, 16) / 256.0


# ============================================================
# 3. readmemh parser
# ============================================================
def readmemh_words(path: str) -> List[int]:
    """
    Basic $readmemh parser.
    Supports:
      - one or multiple hex words per line
      - // comments
      - @addr address directives
    """
    words: List[int] = []
    addr = 0

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("//", 1)[0].strip()
            if not line:
                continue

            tokens = line.replace("\t", " ").split()
            for tok in tokens:
                if not tok:
                    continue

                if tok.startswith("@"):
                    addr = int(tok[1:], 16)
                    while len(words) < addr:
                        words.append(0)
                    continue

                tok = tok.replace("_", "")
                val = int(tok, 16)

                while len(words) <= addr:
                    words.append(0)
                words[addr] = val
                addr += 1

    return words


def load_weight_mem(path: str) -> List[List[int]]:
    """
    Load weight_row_rom[0:OUT_PT_DIM-1].

    RTL:
        op_weight[i_j][i_k] =
            $signed(weight_row_rom[sel_oc][i_k*WEIGHT_WIDTH +: WEIGHT_WIDTH]);

    Therefore, by default:
        dim0 is bits [15:0]
        dim1 is bits [31:16]
        ...
        dim8 is bits [143:128]
    """
    words = readmemh_words(path)

    if len(words) < OUT_PT_DIM:
        raise ValueError(f"weight.mem has only {len(words)} rows, expected {OUT_PT_DIM}")

    weights: List[List[int]] = []

    for oc in range(OUT_PT_DIM):
        row = words[oc] & mask(EXPAND_PT_DIM * WEIGHT_WIDTH)
        w_row = []

        for dim in range(EXPAND_PT_DIM):
            if WEIGHT_DIM0_AT_LSB:
                shift = dim * WEIGHT_WIDTH
            else:
                shift = (EXPAND_PT_DIM - 1 - dim) * WEIGHT_WIDTH

            w = s((row >> shift) & mask(WEIGHT_WIDTH), WEIGHT_WIDTH)
            w_row.append(w)

        weights.append(w_row)

    return weights


def load_bias_mem(path: str) -> List[int]:
    """
    Load bias_rom[0:OUT_PT_DIM-1] as signed 32-bit.
    Your RTL currently uses:
        op_bias[i_j] = $signed(bias_rom[sel_oc]);
    No <<< 8 shift here.
    """
    words = readmemh_words(path)

    if len(words) < OUT_PT_DIM:
        raise ValueError(f"bias.mem has only {len(words)} rows, expected {OUT_PT_DIM}")

    return [s(words[oc], ACC_WIDTH) for oc in range(OUT_PT_DIM)]


# ============================================================
# 4. PFN arithmetic simulation
# ============================================================
def mac_tree_like_rtl(bias: int, mults: List[int]) -> int:
    """
    Match RTL addition grouping:

    st3_mac_tree[j] <= (
        (
            (bias + mult0) +
            (mult1 + mult2)
        ) + (
            (mult3 + mult4) +
            (mult5 + mult6)
        )
    ) + (
        mult7 + mult8
    );

    Every '+' is modeled as signed 32-bit wrap-around.
    """
    if len(mults) != 9:
        raise ValueError("EXPAND_PT_DIM must be 9")

    a0 = add_s32(bias, mults[0])
    a1 = add_s32(mults[1], mults[2])
    a = add_s32(a0, a1)

    b0 = add_s32(mults[3], mults[4])
    b1 = add_s32(mults[5], mults[6])
    b = add_s32(b0, b1)

    c = add_s32(mults[7], mults[8])

    return add_s32(add_s32(a, b), c)


def relu_dequant_like_rtl(mac: int) -> int:
    """
    RTL:
        dequant_temp = st3_mac_tree * 779;
        if (st3_mac_tree < 0)
            relu = 0;
        else
            relu = dequant_temp[35:20];

    No saturation. Just bit slice.
    """
    if mac < 0:
        return 0

    prod64 = s(mac, 32) * DEQUANT_MUL
    prod64_u = u(prod64, 64)

    relu16 = (prod64_u >> DEQUANT_SHIFT) & mask(16)
    return relu16


def pfn_one_point(q_point: List[int], weights: List[List[int]], biases: List[int]) -> List[int]:
    """
    Simulate one PFE point through PFN linear layer.
    Output: 64 channels, each uint16 bit pattern.
    """
    if len(q_point) != EXPAND_PT_DIM:
        raise ValueError(f"Each point must have {EXPAND_PT_DIM} dims")

    out = []

    for oc in range(OUT_PT_DIM):
        mults = []
        for dim in range(EXPAND_PT_DIM):
            prod = mul_s32(q_point[dim], weights[oc][dim])
            mults.append(prod)

        mac = mac_tree_like_rtl(biases[oc], mults)
        relu = relu_dequant_like_rtl(mac)
        out.append(relu)

    return out


def pfn_voxel(points_float: List[List[float]], weights: List[List[int]], biases: List[int]) -> List[int]:
    """
    Simulate one voxel:
      - quantize each input point to int16
      - compute 64-dim PFN output per point
      - max-pool over all points
    """
    max_pool = [0 for _ in range(OUT_PT_DIM)]

    for pt_idx, pt in enumerate(points_float):
        if len(pt) != EXPAND_PT_DIM:
            raise ValueError(f"Point {pt_idx} has {len(pt)} dims, expected {EXPAND_PT_DIM}")

        q_point = [quantize_pfe_float_to_int16(pt[dim], dim) for dim in range(EXPAND_PT_DIM)]

        print(f"\nInput point {pt_idx}:")
        for dim, qv in enumerate(q_point):
            scale_name = "Q4.12" if dim in Q412_DIMS else "Q1.15"
            print(f"  dim{dim}: float={pt[dim]: .6f}, {scale_name}, int={qv:7d}, hex={fmt_hex(qv, 16)}")

        point_out = pfn_one_point(q_point, weights, biases)

        print(f"  Per-point PFN output hex[0:8] = {[fmt_hex(v, 16) for v in point_out[:8]]}")

        for oc in range(OUT_PT_DIM):
            # RTL compares relu_comb with max_pool_regs.
            # Values are non-negative bit patterns, so integer max is enough here.
            if point_out[oc] > max_pool[oc]:
                max_pool[oc] = point_out[oc]

    return max_pool


def pack_m_axis_pfn_data(out64: List[int]) -> int:
    """
    RTL output packing:
        m_axis_pfn_data[oc*PT_WIDTH +: PT_WIDTH] = channel oc
    Thus channel 0 is at LSB.
    """
    if len(out64) != OUT_PT_DIM:
        raise ValueError(f"Expected {OUT_PT_DIM} output channels")

    word = 0
    for oc, val in enumerate(out64):
        word |= u(val, 16) << (oc * 16)

    return word


# ============================================================
# 5. Main
# ============================================================
def main() -> None:
    weights = load_weight_mem(WEIGHT_MEM_PATH)
    biases = load_bias_mem(BIAS_MEM_PATH)

    out64 = pfn_voxel(points_in_voxel, weights, biases)

    packed = pack_m_axis_pfn_data(out64)

    print("\n" + "=" * 80)
    print("Final voxel PFN max-pooled output")
    print("=" * 80)

    print("\nOutput int16/hex/Q8.8:")
    for oc, val in enumerate(out64):
        print(
            f"dim{oc:02d}: "
            f"int={s(val, 16):7d}, "
            f"hex={fmt_hex(val, 16)}, "
            f"q8.8={output_q8_8_to_float(val): .6f}"
        )

    print("\nPacked m_axis_pfn_data:")
    print(f"0x{packed:0{OUT_PT_DIM * 4}x}")

    print("\nPython list, hex:")
    print([fmt_hex(v, 16) for v in out64])

    print("\nPython list, Q8.8 float:")
    print([output_q8_8_to_float(v) for v in out64])


if __name__ == "__main__":
    main()