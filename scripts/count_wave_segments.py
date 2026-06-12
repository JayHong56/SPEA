import argparse
import csv
from typing import List


def load_column_values(csv_path: str, column_index: int, data_start_line: int) -> List[str]:
    """Load one column from CSV, starting at a specific file line (1-based)."""
    values: List[str] = []

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        for line_no, row in enumerate(reader, start=1):
            if line_no < data_start_line:
                continue
            if not row or column_index >= len(row):
                continue
            values.append(row[column_index].strip())

    return values


def count_segments_after_n(values: List[str], n: int) -> int:
    """
    Count contiguous value segments after the n-th data row.

    Example:
    values = [A, A, B, B, A], n = 1
    rows after n => [A, B, B, A]
    segments => [A], [B, B], [A] => 3
    """
    if n < 1:
        raise ValueError("n must be >= 1")

    sliced = values[n:]
    if not sliced:
        return 0

    segments = 1
    prev = sliced[0]

    for current in sliced[1:]:
        if current != prev:
            segments += 1
            prev = current

    return segments


def main() -> None:
    parser = argparse.ArgumentParser(
        description="统计 CSV 第5列在第 n 行之后的连续数据段个数"
    )
    parser.add_argument("csv_path", help="CSV 文件路径")
    parser.add_argument("n", type=int, help="第 n 行（数据行，从 1 开始计数）")
    parser.add_argument(
        "--data-start-line",
        type=int,
        default=3,
        help="CSV 中数据起始文件行号（默认 3，适配 ILA 导出格式）",
    )
    parser.add_argument(
        "--column",
        type=int,
        default=5,
        help="目标列号（从 1 开始，默认 5）",
    )

    args = parser.parse_args()

    if args.n < 1:
        raise ValueError("n must be >= 1")
    if args.column <= 0:
        raise ValueError("column must be >= 1")
    if args.data_start_line <= 0:
        raise ValueError("data-start-line must be >= 1")

    values = load_column_values(
        csv_path=args.csv_path,
        column_index=args.column - 1,
        data_start_line=args.data_start_line,
    )

    # n is 1-based data row index; "after n" means start from row n+1.
    segment_count = count_segments_after_n(values, n=args.n)

    print(f"第 {args.n} 行之后，第 {args.column} 列连续数据段个数: {segment_count}")


if __name__ == "__main__":
    main()
