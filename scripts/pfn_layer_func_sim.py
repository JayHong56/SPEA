#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pfn_layer_func_sim.py

Functional-level simulator for the uploaded Verilog pfn_layer.

Purpose
-------
- Verify PFN data correctness without modeling clock/phase/pipeline/backpressure.
- Keep the same high-level math as RTL:
      mac[oc] = (bias_int32[oc] << 8) + sum_k(input_int16[k] * weight_int16[oc][k])
      act     = ReLU(mac)
      out     = act * REQUANT_MUL / 2**REQUANT_SHIFT
      voxel_out[oc] = max over points in the same voxel

Notes
-----
1. Your weight.mem is assumed to have already compensated for the different Q formats
   of the 9 input dimensions. Therefore this simulator DOES NOT do per-dimension
   weight rescaling.
2. Input can be raw int16, physical float values, or packed hex s_axis_pfe_data.
3. Default output is physical Q8.8 float, matching the RTL CSV debug style:
      out_float = out_int / 256.0

Typical usage
-------------
# Input CSV contains raw signed int16 dim0..dim8:
python pfn_layer_func_sim.py \
  --weight pfn_weight.mem \
  --bias pfn_bias.mem \
  --input pfe_input_raw.csv \
  --output pfn_func_out.csv

# Input CSV is RTL debug pfn_layer_pfe_input_dec.csv, whose dims are physical floats:
python pfn_layer_func_sim.py \
  --weight pfn_weight.mem \
  --bias pfn_bias.mem \
  --input pfn_layer_pfe_input_dec.csv \
  --input-format float \
  --output pfn_func_out.csv \
  --compare-rtl-out pfn_layer_out.csv

Expected input CSV columns
--------------------------
Required:
    voxel_x, voxel_y, and dim0..dim8. Column order does not matter.

The RTL debug CSV pfn_layer_pfe_input_dec.csv is directly supported, e.g.:
    time,voxel_x,voxel_y,dim8,dim7,dim6,dim5,dim4,dim3,dim2,dim1,dim0

Important mapping:
    dim8 is feature index 8, dim7 is feature index 7, ..., dim0 is feature index 0.
    Internally this script always rebuilds vectors as [dim0, dim1, ..., dim8],
    matching RTL bit slicing s_axis_pfe_data[k*PT_WIDTH +: PT_WIDTH].

Optional:
    last / s_axis_pfe_last
    voxel_valid / s_axis_pfe_voxel_valid
    time

If the CSV contains a packed hex column named one of:
    s_axis_pfe_data, data_hex, packed_hex, data
then use --input-format hex. dim0 is decoded from bits [15:0], dim1 from [31:16], etc.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


# -----------------------------------------------------------------------------
# Fixed-point helpers
# -----------------------------------------------------------------------------


def signed_from_unsigned(value: int, width: int) -> int:
    mask = (1 << width) - 1
    value &= mask
    sign = 1 << (width - 1)
    return value - (1 << width) if (value & sign) else value


def unsigned_from_signed(value: int, width: int) -> int:
    return value & ((1 << width) - 1)


def clip_signed(value: int, width: int) -> int:
    lo = -(1 << (width - 1))
    hi = (1 << (width - 1)) - 1
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def quantize_float_to_int16(value: float, frac_bits: int, mode: str = "round", clip: bool = True) -> int:
    scaled = value * (1 << frac_bits)
    if mode == "round":
        q = int(round(scaled))
    elif mode == "floor":
        q = int(math.floor(scaled))
    elif mode == "trunc":
        q = int(scaled)  # toward zero
    else:
        raise ValueError(f"Unsupported quantize mode: {mode}")
    return clip_signed(q, 16) if clip else signed_from_unsigned(q, 16)


def parse_int_auto(text: str) -> int:
    s = str(text).strip()
    if not s:
        raise ValueError("empty integer string")
    s = s.replace("_", "")
    if s.lower().startswith("0x"):
        return int(s, 16)
    return int(s, 10)


def clean_mem_line(line: str) -> str:
    # Remove common comments and whitespace.
    line = line.split("//", 1)[0]
    line = line.split("#", 1)[0]
    return line.strip()


# -----------------------------------------------------------------------------
# MEM file readers
# -----------------------------------------------------------------------------


def readmemh_words(path: str, depth: Optional[int] = None) -> List[int]:
    """
    Read a simple Verilog $readmemh file.
    Supports:
      - one or more hex words per line
      - optional @ADDR directives
      - // and # comments
    """
    words: List[int] = []
    addr: Optional[int] = None

    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = clean_mem_line(raw_line)
            if not line:
                continue
            tokens = line.split()
            for tok in tokens:
                if tok.startswith("@"):
                    addr = int(tok[1:], 16)
                    while len(words) < addr:
                        words.append(0)
                    continue
                tok = tok.replace("_", "")
                if not re.fullmatch(r"[0-9a-fA-F]+", tok):
                    raise ValueError(f"Invalid token in mem file {path}: {tok!r}")
                value = int(tok, 16)
                if addr is None:
                    words.append(value)
                else:
                    while len(words) <= addr:
                        words.append(0)
                    words[addr] = value
                    addr += 1

    if depth is not None:
        if len(words) < depth:
            raise ValueError(f"{path}: expected at least {depth} words, got {len(words)}")
        words = words[:depth]
    return words


def load_weight_rows(path: str, out_dim: int, expand_dim: int, weight_width: int) -> List[List[int]]:
    """
    RTL declaration:
        reg signed [EXPAND_PT_DIM*WEIGHT_WIDTH-1:0] weight_row_rom[0:OUT_PT_DIM-1];
    RTL access:
        weight_row_rom[oc][k*WEIGHT_WIDTH+:WEIGHT_WIDTH]
    Therefore dim0 is in the least significant WEIGHT_WIDTH bits.
    """
    row_words = readmemh_words(path, depth=out_dim)
    weights: List[List[int]] = []
    mask = (1 << weight_width) - 1
    for oc, row in enumerate(row_words):
        w_row = []
        for k in range(expand_dim):
            raw = (row >> (k * weight_width)) & mask
            w_row.append(signed_from_unsigned(raw, weight_width))
        weights.append(w_row)
    return weights


def load_bias(path: str, out_dim: int, acc_width: int) -> List[int]:
    words = readmemh_words(path, depth=out_dim)
    return [signed_from_unsigned(w, acc_width) for w in words]


# -----------------------------------------------------------------------------
# CSV input/output
# -----------------------------------------------------------------------------


@dataclass
class PFEPoint:
    voxel_x: int
    voxel_y: int
    features_int: List[int]
    last: bool = False
    voxel_valid: bool = True
    time: Optional[str] = None


def column_lookup(fieldnames: Sequence[str]) -> Dict[str, str]:
    return {name.strip().lower(): name for name in fieldnames}


def find_column(lut: Dict[str, str], candidates: Sequence[str], required: bool = False) -> Optional[str]:
    for c in candidates:
        if c.lower() in lut:
            return lut[c.lower()]
    if required:
        raise KeyError(f"Missing required CSV column. Tried: {candidates}")
    return None


def infer_input_format(rows: List[Dict[str, str]], dim_cols: List[str], hex_col: Optional[str]) -> str:
    if hex_col is not None:
        return "hex"
    for row in rows[: min(len(rows), 32)]:
        for col in dim_cols:
            s = str(row[col]).strip().lower()
            if "." in s or "e" in s:
                return "float"
    return "raw"


def decode_packed_hex_features(text: str, expand_dim: int, pt_width: int) -> List[int]:
    s = str(text).strip().replace("_", "")
    if s.lower().startswith("0x"):
        value = int(s, 16)
    else:
        value = int(s, 16)
    mask = (1 << pt_width) - 1
    return [signed_from_unsigned((value >> (k * pt_width)) & mask, pt_width) for k in range(expand_dim)]


def load_pfe_points_csv(
    path: str,
    expand_dim: int,
    input_format: str,
    input_frac_bits: Sequence[int],
    quantize_mode: str,
    pt_width: int = 16,
) -> List[PFEPoint]:
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: CSV has no header")
        fieldnames = reader.fieldnames
        lut = column_lookup(fieldnames)
        rows = list(reader)

    vx_col = find_column(lut, ["voxel_x", "x", "s_axis_pfe_voxel_x"], required=True)
    vy_col = find_column(lut, ["voxel_y", "y", "s_axis_pfe_voxel_y"], required=True)
    last_col = find_column(lut, ["last", "s_axis_pfe_last", "is_last"])
    valid_col = find_column(lut, ["voxel_valid", "s_axis_pfe_voxel_valid", "valid"])
    time_col = find_column(lut, ["time", "t"])
    hex_col = find_column(lut, ["s_axis_pfe_data", "data_hex", "packed_hex", "data"])

    # Build columns in feature-index order, not CSV physical order.
    # This supports RTL pfn_layer_pfe_input_dec.csv, whose header is:
    #   time,voxel_x,voxel_y,dim8,dim7,...,dim0
    # dim8 is still feature index 8, so the MAC input vector must be [dim0..dim8].
    dim_cols: List[str] = []
    for k in range(expand_dim):
        col = find_column(lut, [f"dim{k}", f"d{k}", f"f{k}"], required=(hex_col is None))
        if col is not None:
            dim_cols.append(col)

    if input_format == "auto":
        input_format = infer_input_format(rows, dim_cols, hex_col)
    if input_format == "hex" and hex_col is None:
        raise ValueError("--input-format hex requires a packed hex column: s_axis_pfe_data/data_hex/packed_hex/data")

    points: List[PFEPoint] = []
    for row_idx, row in enumerate(rows):
        voxel_x = parse_int_auto(row[vx_col])
        voxel_y = parse_int_auto(row[vy_col])
        voxel_valid = True
        if valid_col is not None:
            voxel_valid = bool(parse_int_auto(row[valid_col]))

        if input_format == "hex":
            features_int = decode_packed_hex_features(row[hex_col], expand_dim, pt_width)
        else:
            features_int = []
            for k in range(expand_dim):
                cell = str(row[dim_cols[k]]).strip()
                if input_format == "raw":
                    features_int.append(signed_from_unsigned(parse_int_auto(cell), pt_width))
                elif input_format == "float":
                    features_int.append(
                        quantize_float_to_int16(float(cell), input_frac_bits[k], mode=quantize_mode, clip=True)
                    )
                else:
                    raise ValueError(f"Unsupported input format: {input_format}")

        last = False
        if last_col is not None:
            last = bool(parse_int_auto(row[last_col]))

        points.append(
            PFEPoint(
                voxel_x=voxel_x,
                voxel_y=voxel_y,
                features_int=features_int,
                last=last,
                voxel_valid=voxel_valid,
                time=row.get(time_col) if time_col is not None else None,
            )
        )

    # If last is absent, infer it from consecutive voxel changes.
    if last_col is None and points:
        for i in range(len(points)):
            if i == len(points) - 1:
                points[i].last = True
            else:
                now = (points[i].voxel_x, points[i].voxel_y)
                nxt = (points[i + 1].voxel_x, points[i + 1].voxel_y)
                points[i].last = (now != nxt)

    return points


def write_output_csv(path: str, rows: List[Dict[str, object]], out_dim: int) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    fieldnames = ["voxel_x", "voxel_y", "pt_count"] + [f"dim{i}" for i in range(out_dim)]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_raw_output_csv(path: str, rows_raw: List[Dict[str, object]], out_dim: int) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    fieldnames = ["voxel_x", "voxel_y", "pt_count"] + [f"dim{i}" for i in range(out_dim)]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows_raw)


# -----------------------------------------------------------------------------
# PFN functional simulation
# -----------------------------------------------------------------------------


@dataclass
class PFNFunctionalConfig:
    expand_dim: int = 9
    out_dim: int = 64
    pt_width: int = 16
    weight_width: int = 16
    acc_width: int = 32
    bias_lshift: int = 8
    requant_mul: int = 779
    requant_shift: int = 20
    output_frac_bits: int = 8
    calc_mode: str = "float"  # float or rtl_int


def point_linear_mac(features_int: Sequence[int], weights: List[List[int]], bias: Sequence[int], cfg: PFNFunctionalConfig) -> List[int]:
    macs: List[int] = []
    for oc in range(cfg.out_dim):
        acc = int(bias[oc]) << cfg.bias_lshift
        w_row = weights[oc]
        for k in range(cfg.expand_dim):
            acc += int(features_int[k]) * int(w_row[k])
        macs.append(acc)
    return macs


def mac_to_output_float(mac: int, cfg: PFNFunctionalConfig) -> float:
    if mac < 0:
        return 0.0
    return (float(mac) * float(cfg.requant_mul)) / float(1 << cfg.requant_shift) / float(1 << cfg.output_frac_bits)


def mac_to_output_int_floor(mac: int, cfg: PFNFunctionalConfig) -> int:
    if mac < 0:
        return 0
    # For non-negative values, RTL bit slice [PT_WIDTH+20-1:20] is equivalent
    # to floor((mac * REQUANT_MUL) / 2**REQUANT_SHIFT), assuming no overflow.
    return int((mac * cfg.requant_mul) >> cfg.requant_shift)


def simulate_pfn_functional(
    points: Sequence[PFEPoint],
    weights: List[List[int]],
    bias: Sequence[int],
    cfg: PFNFunctionalConfig,
) -> Tuple[List[Dict[str, object]], List[Dict[str, object]]]:
    """
    Returns:
      rows_float: dim values in physical output units, i.e. raw / 2**output_frac_bits
      rows_raw:   dim values as integer output codes
    """
    rows_float: List[Dict[str, object]] = []
    rows_raw: List[Dict[str, object]] = []

    max_float = [0.0 for _ in range(cfg.out_dim)]
    max_raw = [0 for _ in range(cfg.out_dim)]
    cur_voxel: Optional[Tuple[int, int]] = None
    pt_count = 0

    for idx, p in enumerate(points):
        if not p.voxel_valid:
            continue

        this_voxel = (p.voxel_x, p.voxel_y)
        if cur_voxel is None:
            cur_voxel = this_voxel
        elif this_voxel != cur_voxel:
            # Safety fallback if last is missing or wrong.
            row_f: Dict[str, object] = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
            row_i: Dict[str, object] = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
            for oc in range(cfg.out_dim):
                row_f[f"dim{oc}"] = max_float[oc]
                row_i[f"dim{oc}"] = max_raw[oc]
            rows_float.append(row_f)
            rows_raw.append(row_i)

            max_float = [0.0 for _ in range(cfg.out_dim)]
            max_raw = [0 for _ in range(cfg.out_dim)]
            cur_voxel = this_voxel
            pt_count = 0

        macs = point_linear_mac(p.features_int, weights, bias, cfg)
        pt_count += 1

        for oc, mac in enumerate(macs):
            out_f = mac_to_output_float(mac, cfg)
            out_i = mac_to_output_int_floor(mac, cfg)
            if out_f > max_float[oc]:
                max_float[oc] = out_f
            if out_i > max_raw[oc]:
                max_raw[oc] = out_i

        if p.last:
            if cur_voxel is None:
                raise RuntimeError("internal error: cur_voxel is None at last")
            row_f = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
            row_i = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
            for oc in range(cfg.out_dim):
                if cfg.calc_mode == "rtl_int":
                    # Easier to compare against RTL CSV: raw integer max, then divide by 2**output_frac_bits.
                    row_f[f"dim{oc}"] = max_raw[oc] / float(1 << cfg.output_frac_bits)
                else:
                    row_f[f"dim{oc}"] = max_float[oc]
                row_i[f"dim{oc}"] = max_raw[oc]
            rows_float.append(row_f)
            rows_raw.append(row_i)

            max_float = [0.0 for _ in range(cfg.out_dim)]
            max_raw = [0 for _ in range(cfg.out_dim)]
            cur_voxel = None
            pt_count = 0

    # Flush if the final voxel did not carry last=1.
    if cur_voxel is not None and pt_count > 0:
        row_f = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
        row_i = {"voxel_x": cur_voxel[0], "voxel_y": cur_voxel[1], "pt_count": pt_count}
        for oc in range(cfg.out_dim):
            row_f[f"dim{oc}"] = (max_raw[oc] / float(1 << cfg.output_frac_bits)) if cfg.calc_mode == "rtl_int" else max_float[oc]
            row_i[f"dim{oc}"] = max_raw[oc]
        rows_float.append(row_f)
        rows_raw.append(row_i)

    return rows_float, rows_raw


# -----------------------------------------------------------------------------
# Compare against RTL output CSV
# -----------------------------------------------------------------------------


def load_output_float_csv(path: str, out_dim: int) -> List[Dict[str, object]]:
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: CSV has no header")
        lut = column_lookup(reader.fieldnames)
        vx_col = find_column(lut, ["voxel_x", "x", "m_axis_pfn_voxel_x"], required=True)
        vy_col = find_column(lut, ["voxel_y", "y", "m_axis_pfn_voxel_y"], required=True)
        pc_col = find_column(lut, ["pt_count", "pt_cnt", "out_pt_cnt"])
        dim_cols = [find_column(lut, [f"dim{i}", f"d{i}"], required=True) for i in range(out_dim)]
        rows: List[Dict[str, object]] = []
        for r in reader:
            row: Dict[str, object] = {
                "voxel_x": parse_int_auto(r[vx_col]),
                "voxel_y": parse_int_auto(r[vy_col]),
                "pt_count": parse_int_auto(r[pc_col]) if pc_col is not None and r.get(pc_col, "") != "" else None,
            }
            for i, col in enumerate(dim_cols):
                row[f"dim{i}"] = float(r[col])
            rows.append(row)
    return rows


def compare_outputs(
    func_rows: List[Dict[str, object]],
    rtl_rows: List[Dict[str, object]],
    out_dim: int,
    tolerance: float,
    compare_csv: Optional[str] = None,
) -> Dict[str, object]:
    n = min(len(func_rows), len(rtl_rows))
    max_abs = 0.0
    max_item: Optional[Tuple[int, int, int, float, float, float]] = None
    mismatch_count = 0
    total_values = n * out_dim
    compare_rows: List[Dict[str, object]] = []

    for i in range(n):
        fx = int(func_rows[i]["voxel_x"])
        fy = int(func_rows[i]["voxel_y"])
        rx = int(rtl_rows[i]["voxel_x"])
        ry = int(rtl_rows[i]["voxel_y"])
        coord_match = (fx == rx and fy == ry)
        for oc in range(out_dim):
            fval = float(func_rows[i][f"dim{oc}"])
            rval = float(rtl_rows[i][f"dim{oc}"])
            diff = abs(fval - rval)
            if diff > max_abs:
                max_abs = diff
                max_item = (i, fx, fy, oc, fval, rval)
            if diff > tolerance or not coord_match:
                mismatch_count += 1
                if len(compare_rows) < 200:
                    compare_rows.append(
                        {
                            "row": i,
                            "func_voxel_x": fx,
                            "func_voxel_y": fy,
                            "rtl_voxel_x": rx,
                            "rtl_voxel_y": ry,
                            "coord_match": int(coord_match),
                            "dim": oc,
                            "func": fval,
                            "rtl": rval,
                            "abs_diff": diff,
                        }
                    )

    if len(func_rows) != len(rtl_rows):
        mismatch_count += abs(len(func_rows) - len(rtl_rows)) * out_dim

    if compare_csv is not None:
        os.makedirs(os.path.dirname(os.path.abspath(compare_csv)) or ".", exist_ok=True)
        fieldnames = [
            "row", "func_voxel_x", "func_voxel_y", "rtl_voxel_x", "rtl_voxel_y",
            "coord_match", "dim", "func", "rtl", "abs_diff"
        ]
        with open(compare_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(compare_rows)

    return {
        "func_rows": len(func_rows),
        "rtl_rows": len(rtl_rows),
        "compared_rows": n,
        "total_values": total_values,
        "mismatch_count": mismatch_count,
        "max_abs_diff": max_abs,
        "max_item": max_item,
    }


def parse_frac_bits(text: Optional[str], expand_dim: int) -> List[int]:
    # Default from your RTL debug comments:
    # dim5: Q1.15; dim2,dim6: Q4.12; others: Q8.8
    frac = [8 for _ in range(expand_dim)]
    if expand_dim > 5:
        frac[5] = 15
    if expand_dim > 2:
        frac[2] = 12
    if expand_dim > 6:
        frac[6] = 12

    if text is None:
        return frac

    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != expand_dim:
        raise ValueError(f"--input-frac-bits expects {expand_dim} comma-separated integers")
    return [int(p) for p in parts]


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Functional-level simulator for pfn_layer")
    p.add_argument("--weight", required=True, help="weight .mem/.hex file used by MEM_WEIGHT_FILE")
    p.add_argument("--bias", required=True, help="bias .mem/.hex file used by MEM_BIAS_FILE")
    p.add_argument("--input", required=True, help="PFE input CSV. Supports RTL pfn_layer_pfe_input_dec.csv order: time,voxel_x,voxel_y,dim8..dim0")
    p.add_argument("--output", default="pfn_func_out.csv", help="functional output CSV, physical Q8.8 float")
    p.add_argument("--output-raw", default=None, help="optional output CSV with raw integer output codes")
    p.add_argument("--compare-rtl-out", default=None, help="optional RTL pfn_layer_out.csv for comparison")
    p.add_argument("--compare-csv", default="pfn_func_compare_mismatch.csv", help="mismatch detail CSV when comparing")

    p.add_argument("--expand-dim", type=int, default=9)
    p.add_argument("--out-dim", type=int, default=64)
    p.add_argument("--pt-width", type=int, default=16)
    p.add_argument("--weight-width", type=int, default=16)
    p.add_argument("--acc-width", type=int, default=32)
    p.add_argument("--bias-lshift", type=int, default=8)
    p.add_argument("--requant-mul", type=int, default=779)
    p.add_argument("--requant-shift", type=int, default=20)
    p.add_argument("--output-frac-bits", type=int, default=8)

    p.add_argument(
        "--input-format",
        choices=["auto", "raw", "float", "hex"],
        default="auto",
        help="raw=int16 dim columns; float=physical dim values; hex=packed s_axis_pfe_data",
    )
    p.add_argument(
        "--input-frac-bits",
        default=None,
        help="fractional bits for dim0..dimN when --input-format float. Default: dim5=15, dim2/dim6=12, others=8",
    )
    p.add_argument("--quantize-mode", choices=["round", "floor", "trunc"], default="round")
    p.add_argument(
        "--calc-mode",
        choices=["float", "rtl_int"],
        default="float",
        help="float: high-level continuous requant; rtl_int: integer floor requant then /256, easier to compare with RTL CSV",
    )
    p.add_argument("--tolerance", type=float, default=1.0 / 256.0, help="comparison tolerance in output float units")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    cfg = PFNFunctionalConfig(
        expand_dim=args.expand_dim,
        out_dim=args.out_dim,
        pt_width=args.pt_width,
        weight_width=args.weight_width,
        acc_width=args.acc_width,
        bias_lshift=args.bias_lshift,
        requant_mul=args.requant_mul,
        requant_shift=args.requant_shift,
        output_frac_bits=args.output_frac_bits,
        calc_mode=args.calc_mode,
    )

    input_frac_bits = parse_frac_bits(args.input_frac_bits, cfg.expand_dim)
    weights = load_weight_rows(args.weight, cfg.out_dim, cfg.expand_dim, cfg.weight_width)
    bias = load_bias(args.bias, cfg.out_dim, cfg.acc_width)
    points = load_pfe_points_csv(
        args.input,
        cfg.expand_dim,
        args.input_format,
        input_frac_bits,
        args.quantize_mode,
        pt_width=cfg.pt_width,
    )

    rows_float, rows_raw = simulate_pfn_functional(points, weights, bias, cfg)
    write_output_csv(args.output, rows_float, cfg.out_dim)
    if args.output_raw is not None:
        write_raw_output_csv(args.output_raw, rows_raw, cfg.out_dim)

    print(f"[OK] loaded points: {len(points)}")
    print(f"[OK] output voxels: {len(rows_float)}")
    print(f"[OK] wrote: {args.output}")
    if args.output_raw is not None:
        print(f"[OK] wrote raw: {args.output_raw}")

    if args.compare_rtl_out is not None:
        rtl_rows = load_output_float_csv(args.compare_rtl_out, cfg.out_dim)
        stats = compare_outputs(rows_float, rtl_rows, cfg.out_dim, args.tolerance, args.compare_csv)
        print("[COMPARE]")
        print(f"  functional rows : {stats['func_rows']}")
        print(f"  rtl rows        : {stats['rtl_rows']}")
        print(f"  compared rows   : {stats['compared_rows']}")
        print(f"  mismatch count  : {stats['mismatch_count']} / {stats['total_values']}")
        print(f"  max abs diff    : {stats['max_abs_diff']}")
        if stats["max_item"] is not None:
            row, vx, vy, oc, fval, rval = stats["max_item"]
            print(f"  max item        : row={row}, voxel=({vx},{vy}), dim{oc}, func={fval}, rtl={rval}")
        print(f"  mismatch csv    : {args.compare_csv}")


if __name__ == "__main__":
    main()



"""


python pfn_layer_func_sim.py ^
  --weight E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\mem\\pfn_weight.mem ^
  --bias E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\mem\\bias.mem ^
  --input E:\\verilog\\pillarnest\\python\\scripts\\test\\1.csv ^
  --input-format float ^
  --calc-mode rtl_int ^
  --output pfn_func_out.csv ^
  --compare-rtl-out E:\\verilog\\pillarnest\\modelsim_pointpillars\\pfn_layer_out.csv
  
  
  """