import csv
import re
from typing import Iterable, List, Optional, Sequence, Tuple

DEFAULT_PATTERN = re.compile(r"Point=\((\d+),\s*(\d+)\).*?PN=(\d+)")
DEFAULT_ENCODINGS = ["utf-8", "utf-8-sig", "gbk", "gb2312", "latin1"]


def read_lines_with_fallback(
    input_file: str,
    encodings: Optional[Sequence[str]] = None,
) -> Tuple[List[str], str]:
    """按候选编码依次读取文本，返回行内容和命中的编码。"""
    selected_encodings = encodings or DEFAULT_ENCODINGS

    for enc in selected_encodings:
        try:
            with open(input_file, "r", encoding=enc) as f:
                return f.readlines(), enc
        except UnicodeDecodeError:
            continue

    raise ValueError("无法识别文件编码，请手动检查文件编码。")


def extract_rows(
    lines: Iterable[str],
    pattern: re.Pattern = DEFAULT_PATTERN,
) -> List[List[int]]:
    """从日志行中提取 [x, y, pn]。"""
    rows: List[List[int]] = []
    for line in lines:
        match = pattern.search(line)
        if match:
            x = int(match.group(1))
            y = int(match.group(2))
            pn = int(match.group(3))
            rows.append([x, y, pn])
    return rows


def convert_sim_to_csv(
    input_file: str = "output_simulation_old.txt",
    output_file: str = "output_voxel_simulation.csv",
    encodings: Optional[Sequence[str]] = None,
    pattern: re.Pattern = DEFAULT_PATTERN,
    verbose: bool = True,
) -> int:
    """将仿真文本提取为 CSV，返回写入记录数。"""
    content, used_encoding = read_lines_with_fallback(input_file, encodings)
    rows = extract_rows(content, pattern)

    with open(output_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "value"])
        writer.writerows(rows)

    if verbose:
        print(f"成功使用编码: {used_encoding}")
        print(f"提取完成，共 {len(rows)} 条，已保存到 {output_file}")

    return len(rows)


if __name__ == "__main__":
    convert_sim_to_csv()
