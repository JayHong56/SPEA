#!/usr/bin/env python3
"""Convert a text file of 128-bit binary strings (one per line) to hexadecimal."""

from __future__ import annotations

import argparse
from pathlib import Path


def normalize_binary_line(raw: str, line_no: int) -> str:
    """Validate and normalize one line into exactly 128 bits."""
    s = raw.strip()
    if not s:
        raise ValueError(f"Line {line_no}: empty line is not allowed")

    if s.startswith("0b") or s.startswith("0B"):
        s = s[2:]

    if len(s) != 128:
        raise ValueError(f"Line {line_no}: expected 128 bits, got {len(s)}")

    if any(ch not in "01" for ch in s):
        raise ValueError(f"Line {line_no}: contains non-binary characters")

    return s


def bin128_to_hex32(bits: str, uppercase: bool = True, with_prefix: bool = False) -> str:
    """Convert a 128-bit binary string to a 32-digit hex string."""
    value = int(bits, 2)
    hex_body = f"{value:032X}" if uppercase else f"{value:032x}"
    return ("0x" + hex_body) if with_prefix else hex_body


def convert_file(input_path: Path, output_path: Path, uppercase: bool, with_prefix: bool) -> None:
    lines_out: list[str] = []

    with input_path.open("r", encoding="utf-8") as f:
        for idx, raw in enumerate(f, start=1):
            bits = normalize_binary_line(raw, idx)
            lines_out.append(bin128_to_hex32(bits, uppercase=uppercase, with_prefix=with_prefix))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines_out))
        if lines_out:
            f.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert each 128-bit binary line in a txt file to hexadecimal."
    )
    parser.add_argument("input", type=Path, help="Input txt file path")
    parser.add_argument("output", type=Path, help="Output txt file path")
    parser.add_argument(
        "--lower",
        action="store_true",
        help="Use lowercase hex letters (default is uppercase)",
    )
    parser.add_argument(
        "--prefix",
        action="store_true",
        help="Add 0x prefix for each output line",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    convert_file(
        input_path=args.input,
        output_path=args.output,
        uppercase=not args.lower,
        with_prefix=args.prefix,
    )


if __name__ == "__main__":
    main()
