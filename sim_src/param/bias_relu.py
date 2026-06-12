# gen_bias_relu_mem.py
from pathlib import Path

ACC_WIDTH = 32
PT_WIDTH = 16
DEQUANT_MUL = 109
DEQUANT_SHIFT = 23

BIAS_MEM = Path(r"E:\mmdetection3d\my_output_parameters\pfn_layer_fused_int_kitti_fixed_hardware\pfn_bias.mem")
OUT_MEM  = Path(r"E:\mmdetection3d\my_output_parameters\pfn_layer_fused_int_kitti_fixed_hardware\bias_relu.mem")


def sign_extend(value: int, width: int) -> int:
    sign_bit = 1 << (width - 1)
    mask = (1 << width) - 1
    value &= mask
    return value - (1 << width) if (value & sign_bit) else value


def parse_hex_line(line: str):
    # 支持注释：// xxx 或 # xxx
    line = line.split("//")[0].split("#")[0].strip()
    if not line:
        return None
    return int(line, 16)


def main():
    out_vals = []

    with BIAS_MEM.open("r", encoding="utf-8") as f:
        for line in f:
            raw = parse_hex_line(line)
            if raw is None:
                continue

            bias = sign_extend(raw, ACC_WIDTH)

            # 对齐 RTL:
            # if (bias_in < 0) relu = 0;
            # else relu = (bias_in * DEQUANT_MUL)[PT_WIDTH+DEQUANT_SHIFT-1 : DEQUANT_SHIFT]
            if bias < 0:
                y = 0
            else:
                prod = bias * DEQUANT_MUL
                y = (prod >> DEQUANT_SHIFT) & ((1 << PT_WIDTH) - 1)

            out_vals.append(y)

    with OUT_MEM.open("w", encoding="utf-8") as f:
        for y in out_vals:
            f.write(f"{y:04X}\n")

    print(f"Generated {OUT_MEM}")
    print(f"num channels = {len(out_vals)}")
    for i, y in enumerate(out_vals[:8]):
        print(f"ch{i}: 0x{y:04X} = {y / 256.0:.6f} Q8.8")


if __name__ == "__main__":
    main()