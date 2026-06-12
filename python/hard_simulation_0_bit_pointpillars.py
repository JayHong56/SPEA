# 4-choice hash / voxel_coor_pipe Python bit-level simulation
# Aligned to the uploaded modified Verilog parameters.

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, TextIO, Dict, Any
from collections import deque
import os
import sys
import struct

try:
    import pandas as pd
except ImportError:  # allow running the core simulator without pandas
    pd = None

# Optional project helper. Keep compatible with your original project layout.
try:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    python_dir = os.path.dirname(current_dir)
    project_root = os.path.dirname(python_dir)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    from python.scripts.csv_sim_process import convert_sim_to_csv
except Exception:
    convert_sim_to_csv = None

# ==========================================================
# 1. Parameters aligned with modified voxel_coor_pipe.v
# ==========================================================
ST_EMPTY = 0b00
ST_OCCU = 0b01
ST_TOMB = 0b10

# Top-level parameters
BRAM_DATA_WIDTH = 640
BRAM_ADDR_WIDTH = 9
BRAM_ADDR_WIDTH_PFE = 6
DRAM_DATA_WIDTH = 128
DRAM_ADDR_WIDTH = 18
HASH_ADDR_WIDTH = 8

THRESHOLD_CLOSE = 0x0100  # currently commented out in Verilog keep_point path
THRESHOLD_BOUDARY_X_LOW = 0x0000
THRESHOLD_BOUDARY_X_HIGH = 0x451F
THRESHOLD_BOUDARY_Y = 0x27AE
THRESHOLD_BOUDARY_Z_LOW = 0x3000
THRESHOLD_BOUDARY_Z_HIGH = 0x1000

LIFE_CYCLE = 100
VN_WIDTH = 5
MAX_VOXEL_NUM = 20

# Localparams
PT_WIDTH_I_XY = 8
PT_WIDTH_F_XY = 8
PT_WIDTH_I_Z = 4
PT_WIDTH_F_Z = 12
PT_WIDTH_I_IS = 1
PT_WIDTH_F_IS = 15

PT_WIDTH = 16
PT_WIDTH_FLOAT32 = 32
VOXEL_WIDTH = 11

POINTS_PER_ROW = BRAM_DATA_WIDTH // (4 * PT_WIDTH)  # 512 / 64 = 8
EXPEND_VOXEL_ROW = (MAX_VOXEL_NUM + POINTS_PER_ROW - 1) // POINTS_PER_ROW  # 4

MULT_FACTOR = 409600  # Verilog: localparam signed [19:0] MULT_FACTOR = 20'd409600
ROUND = 0  # Verilog: localparam integer ROUND = 0

# hash_bucket_table parameters
COORD_WIDTH = VOXEL_WIDTH
TIMER_WIDTH = DRAM_ADDR_WIDTH
NUM_TABLES = 4
BUCKETS_PER_TABLE = 64
TOTAL_CAPACITY = NUM_TABLES * BUCKETS_PER_TABLE
ADDR_WIDTH = HASH_ADDR_WIDTH

CAM_DEPTH = 16


# ==========================================================
# 2. Bit helpers
# ==========================================================
def mask_u(val: int, width: int) -> int:
    return val & ((1 << width) - 1)


def sign_extend(val: int, width: int) -> int:
    val = mask_u(val, width)
    sign = 1 << (width - 1)
    return val - (1 << width) if (val & sign) else val


def u16(val: int) -> int:
    return mask_u(val, 16)


def s16(val: int) -> int:
    return sign_extend(val, 16)


def abs_fix16_signed(val: int) -> int:
    """Verilog abs: x[15] ? (~x + 1'b1) : x, returned as unsigned 16-bit."""
    raw = u16(val)
    return u16((~raw + 1) if (raw & 0x8000) else raw)


def decode_binary_float(bin_str: str) -> float:
    """Decode a 32-bit 01 string as IEEE-754 float32, matching Verilog bit order."""
    return struct.unpack('>f', int(bin_str, 2).to_bytes(4, 'big'))[0]


def float32_to_bits(val: float) -> int:
    return struct.unpack('>I', struct.pack('>f', float(val)))[0]


# ==========================================================
# 3. float_to_fixed and fxp_mul aligned with Verilog modules
# ==========================================================
def float_to_fixed_bittrue(val: float, fixed_width: int, fixed_fractional: int) -> int:
    """
    Bit-true model for your float_to_fixed module.

    It follows the behavior used in your original Python model:
        working_out = {1'b1, float_mantissa, 8'h0}
        shift_dist  = 127 + (FIXED_WIDTH - FIXED_FRACTIONAL) - 1 - float_exp
        fixed_mag   = {1'b0, shifted_out[30 : 32-FIXED_WIDTH]}
        true_fixed_value = fixed_sign ? -fixed_mag : fixed_mag

    Return value is a Python signed integer representing the 16-bit two's-complement output.
    """
    float_in = float32_to_bits(val)

    fixed_sign = (float_in >> 31) & 1
    float_exp = (float_in >> 23) & 0xFF
    float_mantissa = float_in & 0x7FFFFF

    working_out = (1 << 31) | (float_mantissa << 8)

    shift_base = 127 + (fixed_width - fixed_fractional) - 1
    shift_dist = (shift_base - float_exp) & 0xFF
    trunc_shift_dist = 31 if (shift_dist & 0xE0) != 0 else (shift_dist & 0x1F)

    shifted_out = working_out >> trunc_shift_dist
    fixed_mag = (shifted_out >> (32 - fixed_width)) & ((1 << (fixed_width - 1)) - 1)

    true_fixed_value = (-fixed_mag if fixed_sign else fixed_mag) & (
        (1 << fixed_width) - 1
    )
    return sign_extend(true_fixed_value, fixed_width)


def float_to_fixed_xy_q8_8(val: float) -> int:
    return float_to_fixed_bittrue(val, fixed_width=16, fixed_fractional=8)


def float_to_fixed_z_q4_12(val: float) -> int:
    return float_to_fixed_bittrue(val, fixed_width=16, fixed_fractional=12)


def float_to_fixed_i_q1_15(val: float) -> int:
    return float_to_fixed_bittrue(val, fixed_width=16, fixed_fractional=15)


def fxp_mul_q8_8_by_int20_to_s36(ina: int, inb: int, round_en: int = ROUND) -> int:
    """
    Bit-true model for this Verilog instance:
        fxp_mul #(
            .WIIA(8), .WIFA(8), .WIIB(20), .WIFB(0),
            .WOI(36), .WOF(0), .ROUND(0)
        )

    Product has 8 fractional bits, then zooms to WOF=0.
    With ROUND=0, it truncates instead of rounding.
    """
    a = sign_extend(ina, 16)
    b = sign_extend(inb, 20)
    res = a * b
    res_36b = mask_u(res, 36)

    # Equivalent to slicing product[35:8] before sign-extension to 36-bit.
    inr = (res_36b >> 8) & ((1 << 28) - 1)

    if round_en:
        bit_7 = (res_36b >> 7) & 1
        inr_27 = (inr >> 27) & 1
        inr_26_to_0 = inr & ((1 << 27) - 1)
        all_ones = inr_26_to_0 == ((1 << 27) - 1)
        if bit_7 == 1 and not (inr_27 == 0 and all_ones):
            inr = (inr + 1) & ((1 << 28) - 1)

    sign_bit = (inr >> 27) & 1
    out36 = inr | (((1 << 8) - 1) << 28) if sign_bit else inr
    return sign_extend(out36, 36)


# ==========================================================
# 4. Hash function and table simulator
# ==========================================================
def hash_bucket_id_rtl(key_x: int, key_y: int, seed: int) -> int:
    """Python model of hash_func_multiplicative, 4-choice seeds 0..3."""
    BUCKET_AW = 6
    PRIME_X = 10368889
    PRIME_Y = 10000169

    x = sign_extend(key_x, COORD_WIDTH)
    y = sign_extend(key_y, COORD_WIDTH)

    mod_x = x
    mod_y = y

    if seed == 0:
        pass
    elif seed == 1:
        mod_x = ~x
    elif seed == 2:
        mod_y = ~y
    elif seed == 3:
        mod_x = x ^ 0x5A
        mod_y = y ^ 0xA5
    else:
        raise ValueError("seed must be 0, 1, 2, or 3")

    mod_x = sign_extend(mod_x, COORD_WIDTH)
    mod_y = sign_extend(mod_y, COORD_WIDTH)

    mult_x = (mod_x * PRIME_X) & ((1 << 36) - 1)
    mult_y = (mod_y * PRIME_Y) & ((1 << 36) - 1)
    mixed = (mult_x ^ mult_y) & ((1 << 36) - 1)

    return (mixed >> 19) & ((1 << BUCKET_AW) - 1)


@dataclass
class Entry:
    st: int = ST_EMPTY
    kx: int = 0
    ky: int = 0
    pn: int = 0
    ts: int = 0

    def is_free_or_tomb(self) -> bool:
        return self.st in (ST_EMPTY, ST_TOMB)

    def is_occu_and_match(self, x: int, y: int) -> bool:
        return self.st == ST_OCCU and self.kx == x and self.ky == y


class ExpireManagerRTL:
    def __init__(self, life_cycle: int, expire_log_file: Optional[TextIO] = None):
        self.life_cycle = int(life_cycle)
        self.expire_log_file = expire_log_file
        self.dq = deque()
        self.expired_events = []

    def _log_event(
        self,
        kx: int,
        ky: int,
        addr: int,
        pn: int,
        time_now: int,
        dt: int,
        reason: str = "EXPIRE",
    ):
        self.expired_events.append((kx, ky, addr, pn, reason))
        if self.expire_log_file:
            self.expire_log_file.write(
                f"Expire Event: Point=({kx}, {ky}), Addr={addr}, PN={pn}, Time={time_now}, DT={dt}, Reason={reason}\n"
            )

    def on_write_commit(
        self, time_now: int, write_addr: int, tables: List[List[Entry]], cam: List[dict]
    ):
        """
        Approximate the delayed DQ based expiration path.
        write_addr encoding: {bucket_id[5:0], table_id[1:0]}.
        """
        if len(self.dq) < self.life_cycle:
            self.dq.append(write_addr)
            return

        cand_addr = self.dq.popleft()
        self.dq.append(write_addr)

        table_id = cand_addr & 0x3
        bucket_id = (cand_addr >> 2) & 0x3F
        e = tables[table_id][bucket_id]

        dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)
        if e.st == ST_OCCU and dt >= self.life_cycle:
            self._log_event(e.kx, e.ky, cand_addr, e.pn, time_now, dt, reason="EXPIRE")
            tables[table_id][bucket_id] = Entry(st=ST_TOMB)

            for c in cam:
                if c['valid'] and c['mapped_idx'] == cand_addr:
                    c['valid'] = False


class HashBucketTableSim:
    def __init__(
        self, life_cycle: int = LIFE_CYCLE, expire_log_file: Optional[TextIO] = None
    ):
        self.life_cycle = life_cycle
        self.expire_mgr = ExpireManagerRTL(life_cycle, expire_log_file)

        self.tables: List[List[Entry]] = [
            [Entry() for _ in range(BUCKETS_PER_TABLE)] for _ in range(NUM_TABLES)
        ]

        self.global_timer: int = 0
        self.rr_ptr: int = 0

        self.CAM_DEPTH = CAM_DEPTH
        self.cam = [
            {'valid': False, 'kx': 0, 'ky': 0, 'mapped_idx': 0, 'pn': 0}
            for _ in range(self.CAM_DEPTH)
        ]

    def _pack_out_idx(self, table_id: int, bucket_id: int) -> int:
        return ((bucket_id & 0x3F) << 2) | (table_id & 0x3)

    def _is_expired(self, e: Entry) -> bool:
        if e.st != ST_OCCU:
            return False
        return ((self.global_timer - e.ts) & ((1 << TIMER_WIDTH) - 1)) > self.life_cycle

    def _next_pn(self, old_pn: int) -> int:
        # Verilog top allows hash_out_point_number <= MAX_VOXEL_NUM, and MAX_VOXEL_NUM is 32.
        return (
            MAX_VOXEL_NUM if old_pn >= MAX_VOXEL_NUM else mask_u(old_pn + 1, VN_WIDTH)
        )

    def get_overall_loads(self):
        hash_used = 0
        for t in range(NUM_TABLES):
            for b in range(BUCKETS_PER_TABLE):
                e = self.tables[t][b]
                if e.st == ST_OCCU and not self._is_expired(e):
                    hash_used += 1
        hash_pct = (hash_used / TOTAL_CAPACITY) * 100.0

        cam_used = sum(1 for c in self.cam if c['valid'])
        cam_pct = (cam_used / self.CAM_DEPTH) * 100.0
        return hash_used, hash_pct, cam_used, cam_pct

    def process(self, key_x: int, key_y: int) -> Dict[str, Any]:
        x = sign_extend(key_x, COORD_WIDTH)
        y = sign_extend(key_y, COORD_WIDTH)
        time_now = self.global_timer
        hashes = [hash_bucket_id_rtl(x, y, seed=i) for i in range(NUM_TABLES)]

        busy = True
        found = False
        table_full = False
        out_idx = 0
        out_pn = 0
        did_write = False

        # 1. CAM hit path
        cam_hit_idx = -1
        for j in range(self.CAM_DEPTH):
            if (
                self.cam[j]['valid']
                and self.cam[j]['kx'] == x
                and self.cam[j]['ky'] == y
            ):
                cam_hit_idx = j
                break

        if cam_hit_idx != -1:
            found = True
            mapped_idx = self.cam[cam_hit_idx]['mapped_idx']
            out_idx = mapped_idx

            table_id = mapped_idx & 0x3
            bucket_id = (mapped_idx >> 2) & 0x3F

            old_pn = self.cam[cam_hit_idx]['pn']
            new_pn = self._next_pn(old_pn)

            out_pn = new_pn
            self.cam[cam_hit_idx]['pn'] = new_pn
            self.tables[table_id][bucket_id] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True

        else:
            # 2. Base/probe path
            hit_idx = -1
            free_idx = -1
            target_bucket = -1
            probe_offset = 0

            for offset in range(BUCKETS_PER_TABLE):
                eval_buckets = [(h + offset) & 0x3F for h in hashes]

                for i in range(NUM_TABLES):
                    if self.tables[i][eval_buckets[i]].is_occu_and_match(x, y):
                        hit_idx = i
                        target_bucket = eval_buckets[i]
                        break

                if hit_idx != -1:
                    probe_offset = offset
                    break

                check_order = [
                    (self.rr_ptr + i) % NUM_TABLES for i in range(NUM_TABLES)
                ]
                for i in check_order:
                    e = self.tables[i][eval_buckets[i]]
                    if e.is_free_or_tomb() or self._is_expired(e):
                        free_idx = i
                        target_bucket = eval_buckets[i]
                        self.rr_ptr = (self.rr_ptr + 1) % NUM_TABLES
                        break

                if free_idx != -1:
                    probe_offset = offset
                    break

            if hit_idx != -1:
                found = True
                old_e = self.tables[hit_idx][target_bucket]
                new_pn = self._next_pn(old_e.pn)

                out_pn = new_pn
                out_idx = self._pack_out_idx(hit_idx, target_bucket)
                self.tables[hit_idx][target_bucket] = Entry(
                    st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
                )
                did_write = True

            elif free_idx != -1:
                found = True
                out_pn = 1
                out_idx = self._pack_out_idx(free_idx, target_bucket)

                self.tables[free_idx][target_bucket] = Entry(
                    st=ST_OCCU, kx=x, ky=y, pn=1, ts=mask_u(time_now, TIMER_WIDTH)
                )
                did_write = True

                if probe_offset > 0:
                    cam_free_idx = -1
                    for j in range(self.CAM_DEPTH):
                        if not self.cam[j]['valid']:
                            cam_free_idx = j
                            break

                    target_cam_idx = (
                        cam_free_idx
                        if cam_free_idx != -1
                        else (time_now % self.CAM_DEPTH)
                    )
                    self.cam[target_cam_idx] = {
                        'valid': True,
                        'kx': x,
                        'ky': y,
                        'mapped_idx': out_idx,
                        'pn': 1,
                    }
            else:
                table_full = True

        if did_write:
            self.expire_mgr.on_write_commit(time_now, out_idx, self.tables, self.cam)
            self.global_timer = mask_u(self.global_timer + 1, TIMER_WIDTH)

        return {
            "busy": busy,
            "found": found,
            "out_idx": out_idx,
            "out_point_number": out_pn,
            "table_full": table_full,
            "time_now": time_now,
        }

    def flush_all(self):
        """
        Optional frame_end model: force-output all valid table entries once at the end of a frame.
        The exact cycle-level handshake of flush_done is not modeled here; this is a functional dump.
        """
        time_now = self.global_timer
        for bucket_id in range(BUCKETS_PER_TABLE):
            for table_id in range(NUM_TABLES):
                e = self.tables[table_id][bucket_id]
                if e.st == ST_OCCU:
                    addr = self._pack_out_idx(table_id, bucket_id)
                    dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)
                    self.expire_mgr._log_event(
                        e.kx, e.ky, addr, e.pn, time_now, dt, reason="FRAME_FLUSH"
                    )
                    self.tables[table_id][bucket_id] = Entry(st=ST_TOMB)
        for c in self.cam:
            c['valid'] = False


# ==========================================================
# 5. BRAM row/slot mapping aligned with modified Verilog
# ==========================================================
def slot_bwen_64(slot: int) -> int:
    """Verilog: mask[slot*8+:8] = 8'hFF for BRAM_DATA_WIDTH/8 = 64."""
    if not (0 <= slot < POINTS_PER_ROW):
        raise ValueError(f"slot must be 0..{POINTS_PER_ROW - 1}, got {slot}")
    return (0xFF << (slot * 8)) & ((1 << (BRAM_DATA_WIDTH // 8)) - 1)


def calc_bram_write(
    hash_out_idx: int, hash_out_point_number: int, point_proc_u64: int
) -> Dict[str, Any]:
    """
    Verilog mapping:
        pt_idx_minus_1 = hash_out_point_number - 1
        row_offset     = pt_idx_minus_1[4:3]
        bram_row_addr  = {hash_out_idx, 2'b00} + row_offset
        bram_slot_idx  = pt_idx_minus_1[2:0]
        bwen           = slot_bwen_64(bram_slot_idx)
        wrdata         = {POINTS_PER_ROW{pproc_d3}}
    """
    wr_en = (hash_out_point_number != 0) and (hash_out_point_number <= MAX_VOXEL_NUM)
    if not wr_en:
        return {
            "wr_en": False,
            "bram_addr": 0,
            "row_offset": 0,
            "slot": 0,
            "bwen": 0,
            "wrdata": 0,
        }

    pt_idx_minus_1 = (hash_out_point_number - 1) & ((1 << (VN_WIDTH - 1)) - 1)
    row_offset = (pt_idx_minus_1 >> 3) & 0x3
    bram_addr = (((hash_out_idx & 0xFF) << 2) + row_offset) & (
        (1 << BRAM_ADDR_WIDTH) - 1
    )
    slot = pt_idx_minus_1 & 0x7
    bwen = slot_bwen_64(slot)

    wrdata = 0
    p = point_proc_u64 & ((1 << 64) - 1)
    for i in range(POINTS_PER_ROW):
        wrdata |= p << (i * 64)

    return {
        "wr_en": True,
        "bram_addr": bram_addr,
        "row_offset": row_offset,
        "slot": slot,
        "bwen": bwen,
        "wrdata": wrdata,
    }


# ==========================================================
# 6. Voxel coordinate pipeline
# ==========================================================
class VoxelCoorPipe:
    def __init__(
        self, hash_sim: HashBucketTableSim, out_log_file: Optional[TextIO] = None
    ):
        self.hash_sim = hash_sim
        self.out_log_file = out_log_file

    @staticmethod
    def pack_point_proc(x_fix: int, y_fix: int, z_fix: int, intensity_fix: int) -> int:
        # Verilog: point_proc_comb = {pt_x_fix16, pt_y_fix16, pt_z_fix16, pt_intensity_fix16}
        return (
            (u16(x_fix) << 48)
            | (u16(y_fix) << 32)
            | (u16(z_fix) << 16)
            | u16(intensity_fix)
        )

    def process_point(
        self,
        x_f32: float,
        y_f32: float,
        z_f32: float,
        intensity_f32: float,
        raw_zero: bool = False,
    ) -> Dict[str, Any]:
        # Verilog float_to_fixed instances
        x_fix = float_to_fixed_xy_q8_8(x_f32)
        y_fix = float_to_fixed_xy_q8_8(y_f32)
        z_fix = float_to_fixed_z_q4_12(z_f32)
        intensity_fix = float_to_fixed_i_q1_15(intensity_f32)
        point_proc_u64 = self.pack_point_proc(x_fix, y_fix, z_fix, intensity_fix)

        abs_x = abs_fix16_signed(x_fix)
        abs_y = abs_fix16_signed(y_fix)
        abs_z = abs_fix16_signed(z_fix)

        # Verilog keep_point path:
        # 1) point_raw == 0 -> drop
        # 2) pt_x_fix16[15] -> drop
        # 3) ABS_x > X_HIGH or ABS_y > Y -> drop
        # 4) z negative ? ABS_z > Z_LOW : ABS_z > Z_HIGH -> drop
        drop_reason = ""
        keep_point = True
        if raw_zero:
            keep_point = False
            drop_reason = "RAW_ZERO"
        elif u16(x_fix) & 0x8000:
            keep_point = False
            drop_reason = "X_NEGATIVE"
        elif abs_x > THRESHOLD_BOUDARY_X_HIGH or abs_y > THRESHOLD_BOUDARY_Y:
            keep_point = False
            drop_reason = "XY_OUT_OF_RANGE"
        elif (u16(z_fix) & 0x8000) and (abs_z > THRESHOLD_BOUDARY_Z_LOW):
            keep_point = False
            drop_reason = "Z_LOW_OUT_OF_RANGE"
        elif not (u16(z_fix) & 0x8000) and (abs_z > THRESHOLD_BOUDARY_Z_HIGH):
            keep_point = False
            drop_reason = "Z_HIGH_OUT_OF_RANGE"

        if not keep_point:
            return {
                "keep": False,
                "drop_reason": drop_reason,
                "x_fix": x_fix,
                "y_fix": y_fix,
                "z_fix": z_fix,
                "intensity_fix": intensity_fix,
                "point_proc_u64": point_proc_u64,
            }

        # Verilog coordinate transform:
        # X: no boundary add in the modified code.
        fxp_mul_x_out = fxp_mul_q8_8_by_int20_to_s36(x_fix, MULT_FACTOR, ROUND)
        voxel_x = (fxp_mul_x_out >> 16) & ((1 << VOXEL_WIDTH) - 1)

        # Y: add THRESHOLD_BOUDARY_Y first.
        fxp_add_y_out = s16(y_fix + THRESHOLD_BOUDARY_Y)
        fxp_mul_y_out = fxp_mul_q8_8_by_int20_to_s36(fxp_add_y_out, MULT_FACTOR, ROUND)
        voxel_y = (fxp_mul_y_out >> 16) & ((1 << VOXEL_WIDTH) - 1)

        hash_res = self.hash_sim.process(voxel_x, voxel_y)

        bram = (
            calc_bram_write(
                hash_out_idx=hash_res["out_idx"],
                hash_out_point_number=hash_res["out_point_number"],
                point_proc_u64=point_proc_u64,
            )
            if (hash_res["found"] and not hash_res["table_full"])
            else {
                "wr_en": False,
                "bram_addr": 0,
                "row_offset": 0,
                "slot": 0,
                "bwen": 0,
                "wrdata": 0,
            }
        )

        result = {
            "keep": True,
            "drop_reason": "",
            "x_fix": x_fix,
            "y_fix": y_fix,
            "z_fix": z_fix,
            "intensity_fix": intensity_fix,
            "point_proc_u64": point_proc_u64,
            "fxp_mul_x_out": fxp_mul_x_out,
            "fxp_add_y_out": fxp_add_y_out,
            "fxp_mul_y_out": fxp_mul_y_out,
            "voxel_x": voxel_x,
            "voxel_y": voxel_y,
            "hash": hash_res,
            "bram": bram,
        }

        if self.out_log_file:
            self.out_log_file.write(
                f"{voxel_x}, {voxel_y}, {hash_res['out_idx']}, {hash_res['out_point_number']}, "
                f"{bram['bram_addr']}, {bram['slot']}, 0x{bram['bwen']:016x}, "
                f"0x{point_proc_u64:016x}, {1 if bram['wr_en'] else 0}\n"
            )
        else:
            print(
                f"voxel=({voxel_x},{voxel_y}) idx={hash_res['out_idx']} pn={hash_res['out_point_number']} "
                f"bram_addr={bram['bram_addr']} slot={bram['slot']} bwen=0x{bram['bwen']:016x} "
                f"pproc=0x{point_proc_u64:016x} wr={int(bram['wr_en'])}"
            )

        return result


# ==========================================================
# 7. Run pipeline from 128-bit txt
# ==========================================================
def run_pipeline_from_txt(
    txt_file: str,
    voxel_out_file: str = "voxel_output.txt",
    expire_out_file: str = "expire_output.txt",
    flush_at_end: bool = True,
):
    os.makedirs(os.path.dirname(voxel_out_file) or ".", exist_ok=True)
    os.makedirs(os.path.dirname(expire_out_file) or ".", exist_ok=True)

    with open(txt_file, 'r') as f_in, open(voxel_out_file, 'w') as f_vox, open(
        expire_out_file, 'w'
    ) as f_exp:

        hash_sim = HashBucketTableSim(expire_log_file=f_exp)
        pipe = VoxelCoorPipe(hash_sim, out_log_file=f_vox)

        f_vox.write(
            "Voxel_X, Voxel_Y, Out_Idx, Out_PN, BRAM_Addr, BRAM_Slot, BWEN_HEX, PointProc_HEX, BRAM_WR\n"
        )

        point_count = 0
        valid_count = 0
        bram_wr_count = 0
        dropped_count = 0
        hash_load_sum = 0
        max_cam_load = 0

        for line in f_in:
            line = line.strip().replace("_", "")
            if len(line) != 128 or any(ch not in "01" for ch in line):
                continue

            raw_zero = int(line, 2) == 0
            x_f32 = decode_binary_float(line[0:32])
            y_f32 = decode_binary_float(line[32:64])
            z_f32 = decode_binary_float(line[64:96])
            i_f32 = decode_binary_float(line[96:128])

            res = pipe.process_point(x_f32, y_f32, z_f32, i_f32, raw_zero=raw_zero)

            point_count += 1
            if res.get("keep"):
                valid_count += 1
                if res.get("bram", {}).get("wr_en"):
                    bram_wr_count += 1
            else:
                dropped_count += 1

            hash_used, _, cam_used, _ = hash_sim.get_overall_loads()
            hash_load_sum += hash_used
            max_cam_load = max(max_cam_load, cam_used)

            if point_count % 1000 == 0:
                avg_hash_load = hash_load_sum / 1000.0
                avg_hash_pct = (avg_hash_load / TOTAL_CAPACITY) * 100.0
                max_cam_pct = (max_cam_load / hash_sim.CAM_DEPTH) * 100.0
                print(
                    f"[Monitor] Processed {point_count:06d} points | "
                    f"Valid: {valid_count} | Dropped: {dropped_count} | BRAM_WR: {bram_wr_count} | "
                    f"Avg Hash Load: {avg_hash_load:5.1f}/{TOTAL_CAPACITY} ({avg_hash_pct:5.2f}%) | "
                    f"Max CAM Load: {max_cam_load:2d}/{hash_sim.CAM_DEPTH} ({max_cam_pct:5.2f}%)"
                )
                hash_load_sum = 0
                max_cam_load = 0

        remainder = point_count % 1000
        if remainder != 0:
            avg_hash_load = hash_load_sum / remainder
            avg_hash_pct = (avg_hash_load / TOTAL_CAPACITY) * 100.0
            max_cam_pct = (max_cam_load / hash_sim.CAM_DEPTH) * 100.0
            print(
                f"[Monitor] Processed {point_count:06d} points (Final) | "
                f"Valid: {valid_count} | Dropped: {dropped_count} | BRAM_WR: {bram_wr_count} | "
                f"Avg Hash Load: {avg_hash_load:5.1f}/{TOTAL_CAPACITY} ({avg_hash_pct:5.2f}%) | "
                f"Max CAM Load: {max_cam_load:2d}/{hash_sim.CAM_DEPTH} ({max_cam_pct:5.2f}%)"
            )

        if flush_at_end:
            hash_sim.flush_all()

    print("-" * 60)
    print("Simulation done")
    print(f"Input points : {point_count}")
    print(f"Valid points : {valid_count}")
    print(f"Dropped      : {dropped_count}")
    print(f"BRAM writes  : {bram_wr_count}")
    print(f"Voxel log    : {voxel_out_file}")
    print(f"Expire log   : {expire_out_file}")
    print("-" * 60)


# ==========================================================
# 8. CSV helpers
# ==========================================================
def merge_voxel_csv(
    input_csv: str,
    output_csv: str = "merged_data_sim.csv",
    clip_upper: int = MAX_VOXEL_NUM,
):
    """Group by x/y and clip count to MAX_VOXEL_NUM=32 by default."""
    if pd is None:
        raise RuntimeError("pandas is required for merge_voxel_csv")

    df = pd.read_csv(input_csv)
    merged_df = df.groupby(['x', 'y'], as_index=False)['value'].sum()
    merged_df['value'] = merged_df['value'].clip(upper=clip_upper)
    merged_df.to_csv(output_csv, index=False)
    print(f"合并完成！已保存为 {output_csv}")
    return merged_df


def interactive_voxel_debug():
    dummy_hash = HashBucketTableSim()
    pipe = VoxelCoorPipe(dummy_hash)

    print("=" * 60)
    print("VoxelCoorPipe interactive debug, aligned to modified Verilog")
    print("Input: x, y, z, intensity   example: 12.5, -5.2, 0.3, 0.8")
    print("Input 'q' or 'exit' to quit")
    print("=" * 60)

    while True:
        try:
            user_input = input("\n> x, y, z, i = ")
            if user_input.strip().lower() in ['q', 'exit', 'quit']:
                print("退出调试。")
                break

            parts = [p.strip() for p in user_input.split(',')]
            if len(parts) not in (2, 4):
                print("格式错误：请输入 x,y 或 x,y,z,i")
                continue

            x_f32 = float(parts[0])
            y_f32 = float(parts[1])
            z_f32 = float(parts[2]) if len(parts) == 4 else 0.0
            i_f32 = float(parts[3]) if len(parts) == 4 else 0.0

            res = pipe.process_point(x_f32, y_f32, z_f32, i_f32, raw_zero=False)

            print(f"x_fix Q8.8  = {res['x_fix']} (0x{u16(res['x_fix']):04X})")
            print(f"y_fix Q8.8  = {res['y_fix']} (0x{u16(res['y_fix']):04X})")
            print(f"z_fix Q4.12 = {res['z_fix']} (0x{u16(res['z_fix']):04X})")
            print(
                f"i_fix Q1.15 = {res['intensity_fix']} (0x{u16(res['intensity_fix']):04X})"
            )
            print(f"point_proc  = 0x{res['point_proc_u64']:016X}")
            if not res['keep']:
                print(f"DROP: {res['drop_reason']}")
                continue
            print(
                f"mul_x       = {res['fxp_mul_x_out']} (0x{mask_u(res['fxp_mul_x_out'], 36):09X})"
            )
            print(
                f"add_y       = {res['fxp_add_y_out']} (0x{u16(res['fxp_add_y_out']):04X})"
            )
            print(
                f"mul_y       = {res['fxp_mul_y_out']} (0x{mask_u(res['fxp_mul_y_out'], 36):09X})"
            )
            print(f"voxel       = ({res['voxel_x']}, {res['voxel_y']})")
            print(
                f"hash idx/pn = {res['hash']['out_idx']} / {res['hash']['out_point_number']}"
            )
            print(
                f"bram        = wr={int(res['bram']['wr_en'])}, addr={res['bram']['bram_addr']}, "
                f"slot={res['bram']['slot']}, bwen=0x{res['bram']['bwen']:016X}"
            )
        except Exception as e:
            print(f"运行报错: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].lower() in ("debug", "-d", "--debug"):
        interactive_voxel_debug()
        raise SystemExit(0)

    point_txt_path = "E:\\mmdetection3d\\data\\kitti\\scripts\\data\\points_sim_bin.txt"
    killed_txt_path = (
        r"python\output_sim_bitlevel_pointpillars\output_simulation_soft_bitlevel_killed.txt"
    )

    run_pipeline_from_txt(
        point_txt_path,
        voxel_out_file=r"python\output_sim_bitlevel_pointpillars\output_simulation_hard_voxel.txt",
        expire_out_file=killed_txt_path,
        flush_at_end=True,
    )

    # Optional post-processing, kept compatible with your original flow.
    if convert_sim_to_csv is not None:
        killed_csv_path = r"E:\verilog\pillarnest\python\output_sim_bitlevel_pointpillars\output_simulation_sim_bitlevel_killed.csv"
        merged_csv_path = r"E:\verilog\pillarnest\python\output_sim_bitlevel_pointpillars\merged_data_sim_bitlevel.csv"

        count = convert_sim_to_csv(
            input_file=killed_txt_path,
            output_file=killed_csv_path,
            verbose=True,
        )

        merge_voxel_csv(
            input_csv=killed_csv_path,
            output_csv=merged_csv_path,
            clip_upper=MAX_VOXEL_NUM,
        )
