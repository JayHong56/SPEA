#!/usr/bin/env python3
r"""Compare same-name files in two RTL directories.

Default target:
  A: E:\verilog\pillarnest\rtl\modified_hw_kitti_2oc
  B: E:\verilog\pillarnest\temp\rtl\voxelize_kitti_fix_timing_2oc_final

Examples:
  python python\scripts\compare_rtl_same_name_diff.py
  python python\scripts\compare_rtl_same_name_diff.py --show-diff --context 5
  python python\scripts\compare_rtl_same_name_diff.py --diff-dir temp\rtl_diff_report
"""

from __future__ import annotations

import argparse
import difflib
import filecmp
import hashlib
from pathlib import Path
from typing import Iterable


DEFAULT_DIR_A = Path(r"E:\verilog\pillarnest\rtl\modified_hw_kitti_2oc")
DEFAULT_DIR_B = Path(r"E:\verilog\pillarnest\temp\rtl\voxelize_kitti_fix_timing_2oc_final")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare same-name files between two directories and optionally emit unified diffs."
    )
    parser.add_argument("--dir-a", type=Path, default=DEFAULT_DIR_A, help="First directory.")
    parser.add_argument("--dir-b", type=Path, default=DEFAULT_DIR_B, help="Second directory.")
    parser.add_argument(
        "--pattern",
        default="*",
        help="File glob pattern to compare. Default: all files. Example: *.v",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Compare by relative path recursively instead of only the top directory.",
    )
    parser.add_argument(
        "--show-diff",
        action="store_true",
        help="Print unified diffs for different same-name files to stdout.",
    )
    parser.add_argument(
        "--diff-dir",
        type=Path,
        default=None,
        help="Directory to write one .diff file per different same-name file.",
    )
    parser.add_argument(
        "--context",
        type=int,
        default=3,
        help="Context lines in unified diff. Default: 3.",
    )
    return parser.parse_args()


def iter_files(root: Path, pattern: str, recursive: bool) -> Iterable[Path]:
    iterator = root.rglob(pattern) if recursive else root.glob(pattern)
    for path in iterator:
        if path.is_file():
            yield path


def collect_files(root: Path, pattern: str, recursive: bool) -> dict[Path, Path]:
    files: dict[Path, Path] = {}
    for path in iter_files(root, pattern, recursive):
        key = path.relative_to(root) if recursive else Path(path.name)
        files[key] = path
    return files


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_text_lines(path: Path) -> list[str]:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-8", "gbk", "latin-1"):
        try:
            return data.decode(encoding).splitlines(keepends=True)
        except UnicodeDecodeError:
            pass
    return data.decode("latin-1", errors="replace").splitlines(keepends=True)


def make_diff(path_a: Path, path_b: Path, name: Path, context: int) -> str:
    lines_a = read_text_lines(path_a)
    lines_b = read_text_lines(path_b)
    return "".join(
        difflib.unified_diff(
            lines_a,
            lines_b,
            fromfile=f"A/{name.as_posix()}",
            tofile=f"B/{name.as_posix()}",
            n=context,
        )
    )


def safe_diff_name(name: Path) -> str:
    return "__".join(name.parts) + ".diff"


def main() -> int:
    args = parse_args()
    dir_a = args.dir_a.resolve()
    dir_b = args.dir_b.resolve()

    if not dir_a.is_dir():
        raise SystemExit(f"Directory A not found: {dir_a}")
    if not dir_b.is_dir():
        raise SystemExit(f"Directory B not found: {dir_b}")

    files_a = collect_files(dir_a, args.pattern, args.recursive)
    files_b = collect_files(dir_b, args.pattern, args.recursive)

    names_a = set(files_a)
    names_b = set(files_b)
    common = sorted(names_a & names_b, key=lambda p: p.as_posix())
    only_a = sorted(names_a - names_b, key=lambda p: p.as_posix())
    only_b = sorted(names_b - names_a, key=lambda p: p.as_posix())

    same: list[Path] = []
    different: list[Path] = []

    if args.diff_dir is not None:
        args.diff_dir.mkdir(parents=True, exist_ok=True)

    for name in common:
        path_a = files_a[name]
        path_b = files_b[name]
        if filecmp.cmp(path_a, path_b, shallow=False):
            same.append(name)
            continue

        different.append(name)
        diff_text = ""
        if args.show_diff or args.diff_dir is not None:
            diff_text = make_diff(path_a, path_b, name, args.context)
        if args.show_diff:
            print(diff_text)
        if args.diff_dir is not None:
            (args.diff_dir / safe_diff_name(name)).write_text(diff_text, encoding="utf-8")

    print("=" * 80)
    print("RTL same-name file comparison")
    print(f"A: {dir_a}")
    print(f"B: {dir_b}")
    print(f"Pattern: {args.pattern}")
    print(f"Mode: {'recursive relative-path' if args.recursive else 'top-level same filename'}")
    print("-" * 80)
    print(f"Common files     : {len(common)}")
    print(f"Same files       : {len(same)}")
    print(f"Different files  : {len(different)}")
    print(f"Only in A        : {len(only_a)}")
    print(f"Only in B        : {len(only_b)}")

    if different:
        print("\nDifferent same-name files:")
        for name in different:
            path_a = files_a[name]
            path_b = files_b[name]
            print(f"  {name.as_posix()}")
            print(f"    A sha256: {file_sha256(path_a)[:16]}  size={path_a.stat().st_size}")
            print(f"    B sha256: {file_sha256(path_b)[:16]}  size={path_b.stat().st_size}")

    if only_a:
        print("\nOnly in A:")
        for name in only_a:
            print(f"  {name.as_posix()}")

    if only_b:
        print("\nOnly in B:")
        for name in only_b:
            print(f"  {name.as_posix()}")

    if args.diff_dir is not None:
        print(f"\nDiff files written to: {args.diff_dir.resolve()}")

    return 1 if different or only_a or only_b else 0


if __name__ == "__main__":
    raise SystemExit(main())
