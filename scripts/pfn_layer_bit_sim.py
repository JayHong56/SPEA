#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bit-level Python simulator for the provided Verilog pfn_layer.

Main characteristics matched to RTL:
  - EXPAND_PT_DIM = 9 by default.
  - OUT_PT_DIM    = 64 by default.
  - weight_row_rom[oc][k*16 +: 16] is used, so dim0/weight0 is in the LOW 16 bits.
  - bias_rom[oc] is signed 32-bit, then arithmetic-left-shifted by 8 in a 32-bit container.
  - MAC is evaluated with 32-bit signed wraparound after every Verilog-style add/multiply assignment.
  - ReLU + dequant is exactly: if mac < 0 => 0 else (mac * 779)[35:20].
  - Max-pool is performed over all points in a voxel; output happens at last=1.

Input CSV format supported:
  Required columns:
    voxel_x, voxel_y, and dim0..dim8. Column order does not matter.

  The RTL debug CSV pfn_layer_pfe_input_dec.csv is directly supported, e.g.:
    time,voxel_x,voxel_y,dim8,dim7,dim6,dim5,dim4,dim3,dim2,dim1,dim0

  Important mapping:
    dim8 is feature index 8, dim7 is feature index 7, ..., dim0 is feature index 0.
    Internally the simulator always rebuilds the feature vector as [dim0, dim1, ..., dim8],
    matching RTL bit slicing s_axis_pfe_data[k*PT_WIDTH +: PT_WIDTH].

  Optional:
    last        : 1 for the last point of a voxel. If absent, inferred when voxel_x/y changes.
    voxel_valid : if present and 0, the row is skipped.

By default, dim columns are treated as raw signed/fixed 16-bit integers. If --input-is-float is used,
values are quantized according to the debug convention in the RTL:
  dim5: Q1.15; dim2/dim6: Q4.12; all others: Q8.8.

Example:
  python pfn_layer_bit_sim.py \
      --weight pfn_weight.mem \
      --bias pfn_bias.mem \
      --input pfe_points.csv \
      --output pfn_layer_out_py.csv
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# ============================================================
# 1. Verilog bit helpers
# ============================================================


def mask(width: int) -> int:
    if width <= 0:
        raise ValueError("width must be positive")
    return (1 << width) - 1


def uN(x: int, width: int) -> int:
    """Unsigned truncation to width bits."""
    return int(x) & mask(width)


def sN(x: int, width: int) -> int:
    """Interpret low width bits as signed two's complement."""
    x = uN(x, width)
    sign = 1 << (width - 1)
    return x - (1 << width) if (x & sign) else x


def trunc_s(x: int, width: int) -> int:
    """Signed value after assignment to signed [width-1:0]."""
    return sN(x, width)


def add_s32(a: int, b: int) -> int:
    """Verilog-style 32-bit signed addition result."""
    return trunc_s(sN(a, 32) + sN(b, 32), 32)


def mul_s16_s16_to_s32(a: int, b: int) -> int:
    """16x16 signed multiply assigned into signed [31:0]."""
    return trunc_s(sN(a, 16) * sN(b, 16), 32)


def shl_s32(a: int, shift: int) -> int:
    """Verilog signed 32-bit left shift assigned back to signed [31:0]."""
    return trunc_s(sN(a, 32) << shift, 32)


def pack_u16_le(vals: Sequence[int]) -> int:
    """Pack vals so vals[i] occupies [i*16 +: 16], matching RTL."""
    out = 0
    for i, v in enumerate(vals):
        out |= uN(v, 16) << (16 * i)
    return out


def unpack_u16_le(word: int, n: int) -> List[int]:
    """Unpack [i*16 +: 16] fields, returned as signed int16 values."""
    return [sN(word >> (16 * i), 16) for i in range(n)]


def parse_memh_line(line: str) -> Optional[int]:
    """Parse one $readmemh line. Supports comments and underscores."""
    line = line.split("//", 1)[0].split("#", 1)[0].strip().replace("_", "")
    if not line:
        return None
    if line.lower().startswith("0x"):
        line = line[2:]
    return int(line, 16)


def readmemh(path: str, expected_depth: Optional[int] = None) -> List[int]:
    vals: List[int] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            v = parse_memh_line(line)
            if v is not None:
                vals.append(v)
    if expected_depth is not None and len(vals) < expected_depth:
        raise ValueError(
            f"{path}: expected at least {expected_depth} entries, got {len(vals)}"
        )
    return vals


# ============================================================
# 2. PFE fixed-point conversion helpers
# ============================================================


def pfe_dim_frac_bits(dim_idx: int) -> int:
    """
    Fraction bits used by the upstream PFE debug convention.
      dim5      : intensity Q1.15
      dim2,dim6 : z and z-mean Q4.12
      others    : Q8.8
    """
    if dim_idx == 5:
        return 15
    if dim_idx in (2, 6):
        return 12
    return 8


def float_to_s16_fixed(x: float, frac_bits: int, rounding: str = "nearest") -> int:
    scale = 1 << frac_bits
    if rounding == "nearest":
        q = int(round(float(x) * scale))
    elif rounding == "floor":
        import math

        q = int(math.floor(float(x) * scale))
    elif rounding == "trunc":
        q = int(float(x) * scale)
    else:
        raise ValueError(f"unknown rounding mode: {rounding}")
    return sN(q, 16)


def s16_fixed_to_float(x: int, frac_bits: int) -> float:
    return sN(x, 16) / float(1 << frac_bits)


def out_q88_to_float(x: int) -> float:
    """PFN output is printed by RTL debug as signed Q8.8."""
    return sN(x, 16) / 256.0


# ============================================================
# 3. Data classes
# ============================================================


@dataclass
class PFNInputPoint:
    voxel_x: int
    voxel_y: int
    dims: List[int]  # signed/raw 16-bit values, dim0..dim8
    last: bool = False
    voxel_valid: bool = True


@dataclass
class PFNOutputVoxel:
    voxel_x: int
    voxel_y: int
    pt_cnt: int
    dims_u16: List[int]  # raw 16-bit output bit patterns, dim0..dim63

    @property
    def dims_s16(self) -> List[int]:
        return [sN(v, 16) for v in self.dims_u16]

    @property
    def dims_q88(self) -> List[float]:
        return [out_q88_to_float(v) for v in self.dims_u16]

    @property
    def packed_word(self) -> int:
        return pack_u16_le(self.dims_u16)


# ============================================================
# 4. PFN bit simulator
# ============================================================


class PFNLayerBitSim:
    def __init__(
        self,
        weights: Sequence[int] | Sequence[Sequence[int]],
        biases: Sequence[int],
        expand_pt_dim: int = 9,
        out_pt_dim: int = 64,
        pt_width: int = 16,
        weight_width: int = 16,
        acc_width: int = 32,
        dequant_mul: int = 779,
        dequant_shift: int = 20,
        tdm_factor: int = 2,
    ) -> None:
        self.expand_pt_dim = int(expand_pt_dim)
        self.out_pt_dim = int(out_pt_dim)
        self.pt_width = int(pt_width)
        self.weight_width = int(weight_width)
        self.acc_width = int(acc_width)
        self.dequant_mul = int(dequant_mul)
        self.dequant_shift = int(dequant_shift)
        self.tdm_factor = int(tdm_factor)

        if self.pt_width != 16 or self.weight_width != 16 or self.acc_width != 32:
            raise NotImplementedError(
                "This bit simulator currently targets PT_WIDTH=16, WEIGHT_WIDTH=16, ACC_WIDTH=32."
            )
        if self.expand_pt_dim != 9:
            raise ValueError(
                "The uploaded RTL parameterizes EXPAND_PT_DIM, but its adder tree explicitly sums indices 0..8. "
                "For bit-exact matching to this RTL, use EXPAND_PT_DIM=9."
            )
        if self.out_pt_dim % self.tdm_factor != 0:
            raise ValueError("OUT_PT_DIM must be divisible by TDM_FACTOR.")
        if len(biases) < self.out_pt_dim:
            raise ValueError(f"Need {self.out_pt_dim} bias entries, got {len(biases)}")

        # weights may be raw packed rows or already-unpacked rows.
        self.weight_rows: List[List[int]] = []
        for oc in range(self.out_pt_dim):
            row = weights[oc]
            if isinstance(row, int):
                unpacked = unpack_u16_le(row, self.expand_pt_dim)
            else:
                unpacked = [sN(v, 16) for v in row]
            if len(unpacked) != self.expand_pt_dim:
                raise ValueError(
                    f"Weight row {oc}: need {self.expand_pt_dim} entries, got {len(unpacked)}"
                )
            self.weight_rows.append(unpacked)

        self.biases_s32 = [sN(b, 32) for b in biases[: self.out_pt_dim]]

    @classmethod
    def from_memh(
        cls,
        weight_file: str,
        bias_file: str,
        expand_pt_dim: int = 9,
        out_pt_dim: int = 64,
        **kwargs,
    ) -> "PFNLayerBitSim":
        weight_rows = readmemh(weight_file, expected_depth=out_pt_dim)
        biases = readmemh(bias_file, expected_depth=out_pt_dim)
        return cls(
            weights=weight_rows,
            biases=biases,
            expand_pt_dim=expand_pt_dim,
            out_pt_dim=out_pt_dim,
            **kwargs,
        )

    def _mac_one_channel(self, dims_s16: Sequence[int], oc: int) -> int:
        """Exact 32-bit adder-tree MAC for one output channel."""
        if len(dims_s16) != 9:
            raise ValueError("RTL MAC tree requires exactly 9 input dimensions")

        # op_bias[i] = $signed(bias_rom[sel_oc]) <<< 8;
        bias = shl_s32(self.biases_s32[oc], 8)

        # st2_mult[j][k] <= op_data[k] * op_weight[j][k];
        mult = [
            mul_s16_s16_to_s32(dims_s16[k], self.weight_rows[oc][k]) for k in range(9)
        ]

        # Match the parenthesized RTL adder tree. Every add is effectively 32-bit.
        t0 = add_s32(bias, mult[0])
        t1 = add_s32(mult[1], mult[2])
        u0 = add_s32(t0, t1)

        t2 = add_s32(mult[3], mult[4])
        t3 = add_s32(mult[5], mult[6])
        u1 = add_s32(t2, t3)

        v0 = add_s32(u0, u1)
        t4 = add_s32(mult[7], mult[8])
        mac = add_s32(v0, t4)
        return mac

    def _relu_dequant_u16(self, mac_s32: int) -> int:
        """RTL: if mac < 0 then 0 else (mac * 779)[35:20]."""
        mac_s32 = sN(mac_s32, 32)
        prod_s64 = trunc_s(mac_s32 * self.dequant_mul, 64)
        if mac_s32 < 0:
            return 0
        return uN(prod_s64 >> self.dequant_shift, 16)

    def point_forward_u16(self, dims: Sequence[int]) -> List[int]:
        """
        Compute PFN output for one input point before max-pool.
        Return raw 16-bit bit patterns for all OUT_PT_DIM channels.
        """
        if len(dims) != self.expand_pt_dim:
            raise ValueError(f"Need {self.expand_pt_dim} input dims, got {len(dims)}")
        dims_s16 = [sN(v, 16) for v in dims]
        out: List[int] = []
        for oc in range(self.out_pt_dim):
            mac = self._mac_one_channel(dims_s16, oc)
            out.append(self._relu_dequant_u16(mac))
        return out

    def run_stream(self, points: Iterable[PFNInputPoint]) -> List[PFNOutputVoxel]:
        """
        Simulate stream behavior without timing/backpressure.
        A voxel result is emitted when input point.last is True.
        """
        max_pool_u16 = [0 for _ in range(self.out_pt_dim)]
        outputs: List[PFNOutputVoxel] = []
        pt_cnt = 0
        cur_x: Optional[int] = None
        cur_y: Optional[int] = None

        for p in points:
            if not p.voxel_valid:
                continue
            if cur_x is None:
                cur_x, cur_y = int(p.voxel_x), int(p.voxel_y)
            elif (int(p.voxel_x), int(p.voxel_y)) != (cur_x, cur_y):
                # This catches missing last flags; better to fail loudly for bit comparison.
                raise ValueError(
                    f"Voxel changed from ({cur_x},{cur_y}) to ({p.voxel_x},{p.voxel_y}) before last=1. "
                    "Provide a correct last column or use infer_last_for_points()."
                )

            point_out = self.point_forward_u16(p.dims)
            pt_cnt += 1

            # RTL comparison is effectively unsigned because relu_comb is unsigned [15:0].
            for oc, val in enumerate(point_out):
                val_u = uN(val, 16)
                if val_u > max_pool_u16[oc]:
                    max_pool_u16[oc] = val_u

            if p.last:
                outputs.append(PFNOutputVoxel(cur_x, cur_y, pt_cnt, list(max_pool_u16)))
                max_pool_u16 = [0 for _ in range(self.out_pt_dim)]
                pt_cnt = 0
                cur_x, cur_y = None, None

        if pt_cnt != 0:
            raise ValueError("Input ended before last=1 for the final voxel")
        return outputs


def infer_last_for_points(points: List[PFNInputPoint]) -> List[PFNInputPoint]:
    """Infer last flag whenever the next row belongs to a different voxel or EOF."""
    out: List[PFNInputPoint] = []
    for i, p in enumerate(points):
        is_last = (
            (i == len(points) - 1)
            or (points[i + 1].voxel_x != p.voxel_x)
            or (points[i + 1].voxel_y != p.voxel_y)
        )
        out.append(
            PFNInputPoint(
                p.voxel_x, p.voxel_y, list(p.dims), bool(is_last), p.voxel_valid
            )
        )
    return out


# ============================================================
# 5. CSV helpers
# ============================================================


def _find_dim_columns(fieldnames: Sequence[str], expand_pt_dim: int) -> List[str]:
    """
    Return CSV column names in feature-index order: dim0, dim1, ..., dimN.

    This intentionally ignores the physical order in the CSV file. For example, the RTL
    pfn_layer_pfe_input_dec.csv writes columns as dim8, dim7, ..., dim0, but dim8 is
    still feature index 8. The MAC must consume [dim0..dim8] to match
    s_axis_pfe_data[k*PT_WIDTH +: PT_WIDTH] and weight_row_rom[oc][k*WEIGHT_WIDTH +: WEIGHT_WIDTH].
    """
    lut = {str(name).strip().lower(): name for name in fieldnames}
    cols: List[str] = []
    for i in range(expand_pt_dim):
        key = f"dim{i}"
        if key not in lut:
            raise ValueError(
                f"Missing CSV column {key}; available columns: {list(fieldnames)}"
            )
        cols.append(lut[key])
    return cols


def read_pfe_csv(
    path: str,
    expand_pt_dim: int = 9,
    input_is_float: bool = False,
    rounding: str = "nearest",
    infer_last_if_missing: bool = True,
) -> List[PFNInputPoint]:
    points: List[PFNInputPoint] = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty CSV: {path}")
        dim_cols = _find_dim_columns(reader.fieldnames, expand_pt_dim)
        has_last = "last" in reader.fieldnames
        has_voxel_valid = "voxel_valid" in reader.fieldnames

        for row_idx, row in enumerate(reader):
            vx = int(float(row["voxel_x"]))
            vy = int(float(row["voxel_y"]))
            valid = (
                True if not has_voxel_valid else bool(int(float(row["voxel_valid"])))
            )
            last = False if not has_last else bool(int(float(row["last"])))

            dims: List[int] = []
            for i, col in enumerate(dim_cols):
                if input_is_float:
                    dims.append(
                        float_to_s16_fixed(
                            float(row[col]), pfe_dim_frac_bits(i), rounding=rounding
                        )
                    )
                else:
                    # Accept decimal signed values or hex bit patterns such as 0xff80.
                    text = str(row[col]).strip()
                    if text.lower().startswith("0x"):
                        dims.append(sN(int(text, 16), 16))
                    else:
                        dims.append(sN(int(float(text)), 16))
            points.append(PFNInputPoint(vx, vy, dims, last, valid))

    if not has_last and infer_last_if_missing:
        points = infer_last_for_points(points)
    return points


def write_outputs_csv(
    path: str, outputs: Sequence[PFNOutputVoxel], include_hex: bool = False
) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    out_dim = len(outputs[0].dims_u16) if outputs else 64
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        header = ["voxel_x", "voxel_y", "pt_cnt"] + [f"dim{i}" for i in range(out_dim)]
        if include_hex:
            header += [f"dim{i}_hex" for i in range(out_dim)]
        writer.writerow(header)
        for o in outputs:
            row = [o.voxel_x, o.voxel_y, o.pt_cnt] + [f"{v:.6f}" for v in o.dims_q88]
            if include_hex:
                row += [f"0x{uN(v, 16):04x}" for v in o.dims_u16]
            writer.writerow(row)


def write_packed_hex(path: str, outputs: Sequence[PFNOutputVoxel]) -> None:
    """Write one OUT_PT_DIM*16-bit packed word per output voxel."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for o in outputs:
            width_hex = len(o.dims_u16) * 4
            f.write(f"{o.packed_word:0{width_hex}x}\n")


# ============================================================
# 6. Command-line interface
# ============================================================


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Bit-level simulator for Verilog pfn_layer")
    p.add_argument(
        "--weight", required=True, help="MEM_WEIGHT_FILE used by RTL $readmemh"
    )
    p.add_argument("--bias", required=True, help="MEM_BIAS_FILE used by RTL $readmemh")
    p.add_argument(
        "--input",
        required=True,
        help="Input PFE CSV. Supports RTL pfn_layer_pfe_input_dec.csv column order: time,voxel_x,voxel_y,dim8..dim0",
    )
    p.add_argument("--output", default="pfn_layer_out_py.csv", help="Output CSV path")
    p.add_argument(
        "--packed-hex-output",
        default=None,
        help="Optional packed m_axis_pfn_data hex output path",
    )
    p.add_argument("--expand-pt-dim", type=int, default=9)
    p.add_argument("--out-pt-dim", type=int, default=64)
    p.add_argument(
        "--input-is-float",
        action="store_true",
        help="Quantize dim columns from float to fixed first",
    )
    p.add_argument(
        "--rounding", choices=["nearest", "floor", "trunc"], default="nearest"
    )
    p.add_argument(
        "--include-hex",
        action="store_true",
        help="Also write raw output u16 hex columns",
    )
    return p


def main() -> None:
    args = build_argparser().parse_args()
    sim = PFNLayerBitSim.from_memh(
        weight_file=args.weight,
        bias_file=args.bias,
        expand_pt_dim=args.expand_pt_dim,
        out_pt_dim=args.out_pt_dim,
    )
    points = read_pfe_csv(
        args.input,
        expand_pt_dim=args.expand_pt_dim,
        input_is_float=args.input_is_float,
        rounding=args.rounding,
    )
    outputs = sim.run_stream(points)
    write_outputs_csv(args.output, outputs, include_hex=args.include_hex)
    if args.packed_hex_output:
        write_packed_hex(args.packed_hex_output, outputs)
    print(
        f"PFN bit simulation done. input_points={len(points)}, output_voxels={len(outputs)}, output={args.output}"
    )


if __name__ == "__main__":
    main()


"""

python pfn_layer_bit_sim.py ^
  --weight E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\mem\\pfn_weight.mem ^
  --bias E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\mem\\bias.mem ^
  --input E:\\verilog\\pillarnest\\python\\scripts\\test\\1.csv ^
  --input-is-float ^
  --output pfn_bit_out.csv ^
  --include-hex
  
  
  """