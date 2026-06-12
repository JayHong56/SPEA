"""
将文本文件每行前两列转换为整数（去掉小数部分），其余列保持不变。

用法：
python python/scripts/convert_first_two_cols_to_int.py -i input.txt -o output.txt
"""

from __future__ import annotations

import argparse
from pathlib import Path


def convert_line(line: str, line_no: int) -> str:
    stripped = line.strip()
    if not stripped:
        return "\n"

    parts = stripped.split()
    if len(parts) < 2:
        raise ValueError(f"第 {line_no} 行列数不足 2 列: {stripped}")

    # 前两列转整数；其余列原样保留。
    parts[0] = str(int(float(parts[0])))
    parts[1] = str(int(float(parts[1])))

    return " ".join(parts) + "\n"


def convert_file(input_path: Path, output_path: Path) -> None:
    with input_path.open("r", encoding="utf-8") as fin, output_path.open("w", encoding="utf-8") as fout:
        for line_no, line in enumerate(fin, start=1):
            try:
                fout.write(convert_line(line, line_no))
            except ValueError as exc:
                raise ValueError(f"处理失败，文件: {input_path}，{exc}") from exc


def main() -> None:
    parser = argparse.ArgumentParser(description="将文件前两列转为整数")
    parser.add_argument("-i", "--input", required=True, help="输入文件路径")
    parser.add_argument("-o", "--output", required=True, help="输出文件路径")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"输入文件不存在: {input_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    convert_file(input_path, output_path)
    print(f"转换完成: {output_path}")


if __name__ == "__main__":
    main()
