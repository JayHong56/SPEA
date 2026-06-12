#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PFN float/torch functional reference from pfn_layer_pfe_input_dec.csv.

Purpose
-------
Use the floating-point values printed by RTL debug CSV as PFN input,
load the original floating-point weight/bias txt files, and compute:

    y = linear(x) = x @ W.T + b

Optionally apply ReLU and/or consecutive-voxel max-pooling.

Important for your CSV
----------------------
Your pfn_layer_pfe_input_dec.csv columns are:

    time,voxel_x,voxel_y,dim8,dim7,dim6,dim5,dim4,dim3,dim2,dim1,dim0

and you said dim8 is x, dim7 is y, etc. Therefore the default feature order is:

    [dim8, dim7, dim6, dim5, dim4, dim3, dim2, dim1, dim0]

This is usually the order expected by the original PyTorch float weight.
If you want the RTL bit-index order instead, use:

    --feature-order dim_asc

which feeds:

    [dim0, dim1, ..., dim8]
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import torch
import torch.nn.functional as F


def _read_numeric_txt(path: str) -> np.ndarray:
    """Read a numeric txt/csv file robustly."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    errors = []
    for delimiter in (None, ",", "\t", " "):
        try:
            arr = np.loadtxt(path, delimiter=delimiter, dtype=np.float64)
            if arr.size > 0:
                return np.asarray(arr, dtype=np.float64)
        except Exception as e:  # keep trying
            errors.append(f"delimiter={delimiter!r}: {e}")

    # Last fallback: genfromtxt with comments/header tolerance
    for delimiter in (None, ",", "\t"):
        try:
            arr = np.genfromtxt(path, delimiter=delimiter, dtype=np.float64)
            arr = np.asarray(arr, dtype=np.float64)
            arr = arr[~np.isnan(arr)] if arr.ndim == 1 else arr
            if arr.size > 0:
                return arr
        except Exception as e:
            errors.append(f"genfromtxt delimiter={delimiter!r}: {e}")

    raise ValueError(f"Cannot read numeric txt file: {path}\n" + "\n".join(errors[-6:]))


def load_weight_txt(
    path: str,
    in_dim: int,
    out_dim: Optional[int],
    layout: str,
) -> np.ndarray:
    """
    Load floating-point weight.

    Returned shape is always [out_dim, in_dim], matching torch F.linear.
    Supported source shapes:
      - [out_dim, in_dim]
      - [in_dim, out_dim]
      - flat out_dim * in_dim
    """
    arr = _read_numeric_txt(path)

    if arr.ndim == 0:
        raise ValueError("Weight file contains only one scalar; expected a matrix or flat vector.")

    if arr.ndim == 1:
        if out_dim is None:
            if arr.size % in_dim != 0:
                raise ValueError(
                    f"Flat weight length {arr.size} is not divisible by in_dim={in_dim}. "
                    "Please specify --out-dim."
                )
            out_dim = arr.size // in_dim
        if arr.size != out_dim * in_dim:
            raise ValueError(
                f"Flat weight length {arr.size} != out_dim*in_dim = {out_dim}*{in_dim}."
            )
        # For a flat txt, default assumes row-major [out_dim, in_dim].
        w = arr.reshape(out_dim, in_dim)
        return w.astype(np.float32)

    if arr.ndim > 2:
        arr = arr.reshape(arr.shape[0], -1)

    h, w = arr.shape

    if layout == "out_in":
        if w != in_dim:
            raise ValueError(f"--weight-layout out_in requires weight shape [out_dim,{in_dim}], got {arr.shape}")
        return arr.astype(np.float32)

    if layout == "in_out":
        if h != in_dim:
            raise ValueError(f"--weight-layout in_out requires weight shape [{in_dim},out_dim], got {arr.shape}")
        return arr.T.astype(np.float32)

    if layout != "auto":
        raise ValueError(f"Unsupported layout: {layout}")

    # auto
    candidates = []
    if w == in_dim:
        candidates.append(arr)
    if h == in_dim:
        candidates.append(arr.T)

    if out_dim is not None:
        candidates = [c for c in candidates if c.shape == (out_dim, in_dim)]

    if len(candidates) == 1:
        return candidates[0].astype(np.float32)

    if len(candidates) > 1:
        # Ambiguous only for square-ish cases. Prefer [out_dim, in_dim].
        return candidates[0].astype(np.float32)

    raise ValueError(
        f"Cannot infer weight layout from shape {arr.shape}, in_dim={in_dim}, out_dim={out_dim}. "
        "Use --weight-layout out_in or --weight-layout in_out."
    )


def load_bias_txt(path: str, out_dim: int) -> np.ndarray:
    arr = _read_numeric_txt(path).reshape(-1)
    if arr.size != out_dim:
        raise ValueError(f"Bias length {arr.size} != out_dim={out_dim}")
    return arr.astype(np.float32)


def _column_lookup(df: pd.DataFrame) -> dict:
    return {str(c).strip().lower(): c for c in df.columns}


def _find_col(df: pd.DataFrame, names: Sequence[str], required: bool = True) -> Optional[str]:
    lut = _column_lookup(df)
    for name in names:
        key = name.strip().lower()
        if key in lut:
            return lut[key]
    if required:
        raise KeyError(f"Cannot find any column from {names}. Existing columns: {list(df.columns)}")
    return None


def parse_feature_cols(feature_cols: Optional[str], in_dim: int, feature_order: str) -> List[str]:
    if feature_cols:
        cols = [c.strip() for c in feature_cols.split(",") if c.strip()]
        if len(cols) != in_dim:
            raise ValueError(f"--feature-cols must provide {in_dim} columns, got {len(cols)}")
        return cols

    if feature_order == "csv_desc":
        # Your CSV physical/original PyTorch order: dim8, dim7, ..., dim0.
        return [f"dim{k}" for k in range(in_dim - 1, -1, -1)]

    if feature_order == "dim_asc":
        # RTL bit-index order: dim0, dim1, ..., dim8.
        return [f"dim{k}" for k in range(in_dim)]

    raise ValueError(f"Unsupported feature_order: {feature_order}")


def load_pfe_input_csv(
    path: str,
    in_dim: int,
    feature_order: str,
    feature_cols: Optional[str],
) -> Tuple[pd.DataFrame, np.ndarray, List[str]]:
    df = pd.read_csv(path)
    df.columns = [str(c).strip() for c in df.columns]

    wanted_cols = parse_feature_cols(feature_cols, in_dim, feature_order)
    real_cols = []
    for c in wanted_cols:
        real_cols.append(_find_col(df, [c]))

    x = df[real_cols].astype(np.float32).to_numpy()
    return df, x, real_cols


def build_meta_df(df: pd.DataFrame) -> pd.DataFrame:
    cols = []
    for names in (["time"], ["voxel_x", "x_idx", "vx"], ["voxel_y", "y_idx", "vy"]):
        c = _find_col(df, names, required=False)
        if c is not None:
            cols.append(c)
    if not cols:
        return pd.DataFrame({"row": np.arange(len(df), dtype=np.int64)})
    return df[cols].copy()


def save_point_csv(path: str, meta: pd.DataFrame, y: np.ndarray, prefix: str = "dim") -> None:
    out = meta.reset_index(drop=True).copy()
    for i in range(y.shape[1]):
        out[f"{prefix}{i}"] = y[:, i]
    out.to_csv(path, index=False)


def consecutive_voxel_pool(
    df: pd.DataFrame,
    y: np.ndarray,
) -> pd.DataFrame:
    vx_col = _find_col(df, ["voxel_x", "x_idx", "vx"])
    vy_col = _find_col(df, ["voxel_y", "y_idx", "vy"])
    time_col = _find_col(df, ["time"], required=False)

    vx = df[vx_col].to_numpy()
    vy = df[vy_col].to_numpy()

    rows = []
    n = len(df)
    start = 0
    while start < n:
        end = start + 1
        while end < n and vx[end] == vx[start] and vy[end] == vy[start]:
            end += 1

        pooled = y[start:end].max(axis=0)
        row = {}
        if time_col is not None:
            row["time_start"] = df.iloc[start][time_col]
            row["time_end"] = df.iloc[end - 1][time_col]
        row["voxel_x"] = int(vx[start])
        row["voxel_y"] = int(vy[start])
        row["pt_count"] = int(end - start)
        for i, val in enumerate(pooled):
            row[f"dim{i}"] = float(val)
        rows.append(row)
        start = end

    return pd.DataFrame(rows)


def compare_csv(a_path: str, b_path: str, tol: float, max_report: int = 30) -> None:
    """Compare two CSVs by common dim columns and optional voxel_x/y."""
    a = pd.read_csv(a_path)
    b = pd.read_csv(b_path)
    if len(a) != len(b):
        print(f"[COMPARE] row count mismatch: {len(a)} vs {len(b)}")

    n = min(len(a), len(b))
    dim_cols = [c for c in a.columns if str(c).startswith("dim") and c in b.columns]
    if not dim_cols:
        print("[COMPARE] no common dim columns found.")
        return

    av = a.loc[: n - 1, dim_cols].to_numpy(dtype=np.float64)
    bv = b.loc[: n - 1, dim_cols].to_numpy(dtype=np.float64)
    diff = np.abs(av - bv)
    max_diff = float(np.nanmax(diff)) if diff.size else 0.0
    bad = np.argwhere(diff > tol)

    print(f"[COMPARE] common rows={n}, common dims={len(dim_cols)}, max_abs_diff={max_diff:.8g}, tol={tol}")
    print(f"[COMPARE] mismatched elements={len(bad)}")

    if len(bad) > 0:
        for idx, (r, cidx) in enumerate(bad[:max_report]):
            col = dim_cols[cidx]
            print(
                f"  row={r}, col={col}: this={av[r, cidx]:.8g}, "
                f"ref={bv[r, cidx]:.8g}, diff={diff[r, cidx]:.8g}"
            )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Torch float PFN linear reference using pfn_layer_pfe_input_dec.csv."
    )
    parser.add_argument("--input", required=True, help="pfn_layer_pfe_input_dec.csv")
    parser.add_argument("--weight", required=True, help="Original float weight txt, not Q-scaled")
    parser.add_argument("--bias", required=True, help="Original float bias txt, not Q-scaled")
    parser.add_argument("--in-dim", type=int, default=9)
    parser.add_argument("--out-dim", type=int, default=None, help="Optional. Inferred from weight if omitted.")
    parser.add_argument(
        "--feature-order",
        choices=["csv_desc", "dim_asc"],
        default="csv_desc",
        help=(
            "csv_desc: feed [dim8,dim7,...,dim0], matching your CSV/original feature order. "
            "dim_asc: feed [dim0,dim1,...,dim8], matching RTL bit-index order."
        ),
    )
    parser.add_argument(
        "--feature-cols",
        default=None,
        help="Explicit feature columns in order, e.g. dim8,dim7,dim6,dim5,dim4,dim3,dim2,dim1,dim0",
    )
    parser.add_argument(
        "--weight-layout",
        choices=["auto", "out_in", "in_out"],
        default="auto",
        help="Weight txt layout. Returned weight is [out_dim,in_dim] for torch F.linear.",
    )
    parser.add_argument(
        "--activation",
        choices=["none", "relu"],
        default="none",
        help="Apply activation after linear. Default is raw linear result.",
    )
    parser.add_argument(
        "--output-point",
        default="pfn_torch_linear_point.csv",
        help="Per-input-point output CSV.",
    )
    parser.add_argument(
        "--output-pool",
        default=None,
        help="Optional consecutive-voxel max-pooled output CSV. Uses output after optional activation.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        choices=["auto", "cpu", "cuda"],
    )
    parser.add_argument("--dtype", default="float32", choices=["float32", "float64"])
    parser.add_argument(
        "--compare-point",
        default=None,
        help="Optional reference CSV to compare with --output-point.",
    )
    parser.add_argument(
        "--compare-pool",
        default=None,
        help="Optional reference CSV to compare with --output-pool.",
    )
    parser.add_argument("--compare-tol", type=float, default=1e-5)

    args = parser.parse_args()

    df, x_np, used_cols = load_pfe_input_csv(
        args.input,
        in_dim=args.in_dim,
        feature_order=args.feature_order,
        feature_cols=args.feature_cols,
    )

    w_np = load_weight_txt(
        args.weight,
        in_dim=args.in_dim,
        out_dim=args.out_dim,
        layout=args.weight_layout,
    )
    out_dim = w_np.shape[0]
    b_np = load_bias_txt(args.bias, out_dim=out_dim)

    if x_np.shape[1] != w_np.shape[1]:
        raise ValueError(f"Input dim {x_np.shape[1]} != weight in_dim {w_np.shape[1]}")

    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("--device cuda requested but CUDA is not available.")

    dtype = torch.float32 if args.dtype == "float32" else torch.float64

    x = torch.as_tensor(x_np, dtype=dtype, device=device)
    w = torch.as_tensor(w_np, dtype=dtype, device=device)
    b = torch.as_tensor(b_np, dtype=dtype, device=device)

    with torch.no_grad():
        y = F.linear(x, w, b)  # y = x @ w.T + b
        if args.activation == "relu":
            y = torch.relu(y)

    y_np = y.detach().cpu().numpy()

    meta = build_meta_df(df)
    save_point_csv(args.output_point, meta, y_np)

    print("[OK] Torch PFN float reference finished.")
    print(f"  input csv       : {args.input}")
    print(f"  feature order   : {args.feature_order}")
    print(f"  used columns    : {used_cols}")
    print(f"  X shape         : {x_np.shape}")
    print(f"  W shape         : {w_np.shape}  [out_dim, in_dim]")
    print(f"  b shape         : {b_np.shape}")
    print(f"  activation      : {args.activation}")
    print(f"  output point csv: {args.output_point}")

    if args.output_pool is not None:
        pool_df = consecutive_voxel_pool(df, y_np)
        pool_df.to_csv(args.output_pool, index=False)
        print(f"  output pool csv : {args.output_pool}")
        print(f"  pooled voxels   : {len(pool_df)}")

    if args.compare_point is not None:
        compare_csv(args.output_point, args.compare_point, args.compare_tol)

    if args.compare_pool is not None:
        if args.output_pool is None:
            raise ValueError("--compare-pool requires --output-pool")
        compare_csv(args.output_pool, args.compare_pool, args.compare_tol)


if __name__ == "__main__":
    main()


"""
python pfn_torch_float_linear.py ^
  --input E:\\verilog\\pillarnest\\python\\scripts\\test\\1.csv ^
  --weight E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\W_fused_fp32.txt ^
  --bias E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\b_fused_fp32.txt ^
  --output-point pfn_torch_linear_point.csv

 
"""