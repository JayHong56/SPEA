import argparse
import csv
import re
from pathlib import Path

import torch


def parse_out_data_hex(
    hex_str: str,
    dims: int = 48,
    bits_per_dim: int = 16,
    signed: bool = False,
    strict: bool = False,
):
    """Parse packed hex string into a list of decimal values.

    In the Verilog packing used here, dim 0 is at LSB side, while %h prints MSB first.
    So we split MSB->LSB then reverse to get [dim0, dim1, ...].
    """
    hex_str = hex_str.strip().lower().replace("_", "")
    if hex_str.startswith("0x"):
        hex_str = hex_str[2:]

    # ModelSim/Questa may emit x/z/? for unknown/high-impedance bits.
    # In non-strict mode, map them to 0 so conversion can continue.
    if strict:
        if re.search(r"[^0-9a-f]", hex_str):
            raise ValueError(f"Non-hex character found in out_data_hex: {hex_str}")
    else:
        hex_str = re.sub(r"[^0-9a-f]", "0", hex_str)

    hex_per_dim = bits_per_dim // 4
    total_hex_len = dims * hex_per_dim

    if len(hex_str) < total_hex_len:
        hex_str = hex_str.zfill(total_hex_len)
    elif len(hex_str) > total_hex_len:
        # Keep the least-significant bits if the string is longer than expected.
        hex_str = hex_str[-total_hex_len:]

    msb_to_lsb_chunks = [hex_str[i:i + hex_per_dim] for i in range(0, total_hex_len, hex_per_dim)]
    values = [int(chunk, 16) for chunk in reversed(msb_to_lsb_chunks)]

    if signed:
        sign_bit = 1 << (bits_per_dim - 1)
        mod = 1 << bits_per_dim
        values = [v - mod if (v & sign_bit) else v for v in values]

    return values


def csv_out_data_to_tensor(
    input_csv: Path,
    output_tensor: Path,
    output_float_csv: Path,
    signed: bool = False,
    strict: bool = False,
    scale: float = 1.0,
):
    rows = []
    meta_rows = []

    with input_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required_cols = {"time", "voxel_x", "voxel_y", "out_data_hex"}
        missing = required_cols - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing required columns: {sorted(missing)}")

        for idx, row in enumerate(reader, start=2):  # header is line 1
            try:
                values = parse_out_data_hex(
                    row["out_data_hex"],
                    dims=48,
                    bits_per_dim=16,
                    signed=signed,
                    strict=strict,
                )
            except ValueError as exc:
                raise ValueError(f"CSV line {idx} parse failed: {exc}") from exc
            float_values = [float(v) / scale for v in values]
            rows.append(float_values)
            meta_rows.append((int(row["time"]), int(row["voxel_x"]), int(row["voxel_y"])))

    if not rows:
        raise ValueError("No data rows found in CSV")

    tensor = torch.tensor(rows, dtype=torch.float32)
    output_tensor.parent.mkdir(parents=True, exist_ok=True)
    torch.save(tensor, output_tensor)

    output_float_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_float_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        header = ["time", "voxel_x", "voxel_y"] + [f"dim{i}" for i in range(48)]
        writer.writerow(header)
        for (t, x, y), vals in zip(meta_rows, rows):
            writer.writerow([t, x, y] + vals)

    print(f"Saved float tensor shape={tuple(tensor.shape)} to: {output_tensor}")
    print(f"Saved per-dimension float CSV to: {output_float_csv}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert pfn_layer_out.csv out_data_hex (48x16bit packed) to a decimal tensor"
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("modelsim_pre/pfn_layer_out.csv"),
        help="Input CSV path with out_data_hex column",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("modelsim_pre/pfn_layer_out_tensor.pt"),
        help="Output tensor file path (.pt)",
    )
    parser.add_argument(
        "--output-float-csv",
        type=Path,
        default=Path("modelsim_pre/pfn_layer_out_float.csv"),
        help="Output CSV path with per-dimension decimal float values",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Interpret each 16-bit dimension as signed int16",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail when out_data_hex contains non-hex characters (x/z/? etc.)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Divide each dimension by this value (e.g. 256 for Q8.8)",
    )
    args = parser.parse_args()

    if args.scale == 0:
        raise ValueError("--scale must not be 0")

    csv_out_data_to_tensor(
        args.input,
        args.output,
        args.output_float_csv,
        signed=args.signed,
        strict=args.strict,
        scale=args.scale,
    )


if __name__ == "__main__":
    main()
