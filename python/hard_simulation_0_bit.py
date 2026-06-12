# 4-choice hash

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import sys

import os

# --- 动态添加项目根目录到环境变量 ---
# 获取当前脚本所在目录: python/hard_simulation
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取 python 目录
python_dir = os.path.dirname(current_dir)
# 获取项目根目录: pillarnest
project_root = os.path.dirname(python_dir)

# 将项目根目录加入 sys.path，优先级设为最高 (索引 0)
sys.path.insert(0, project_root)
# ------------------------------------

import pandas as pd
from collections import deque


from python.scripts.csv_sim_process import convert_sim_to_csv

import struct
import math

# -----------------------------
# 1. 之前你提供的 HashBucketTableSim 核心 (略加精简)
# -----------------------------
ST_EMPTY = 0b00
ST_OCCU = 0b01
ST_TOMB = 0b10

COORD_WIDTH = 11
VN_WIDTH = 5
TIMER_WIDTH = 16

NUM_TABLES = 4
BUCKETS_PER_TABLE = 64


def mask_u(val: int, width: int) -> int:
    return val & ((1 << width) - 1)


def sign_extend(val: int, width: int) -> int:
    val = mask_u(val, width)
    sign = 1 << (width - 1)
    return val - (1 << width) if (val & sign) else val


def hash_bucket_id_rtl(key_x: int, key_y: int, seed: int) -> int:
    x = sign_extend(key_x, COORD_WIDTH)
    y = sign_extend(key_y, COORD_WIDTH)
    mod_x, mod_y = x, y
    if seed == 1:
        mod_x = ~x
    elif seed == 2:
        mod_y = ~y
    elif seed == 3:
        mod_x = x ^ 0x5A
        mod_y = y ^ 0xA5
    mod_x = sign_extend(mod_x, COORD_WIDTH)
    mod_y = sign_extend(mod_y, COORD_WIDTH)

    mult_x = (mod_x * 10368889) & ((1 << 36) - 1)
    mult_y = (mod_y * 10000169) & ((1 << 36) - 1)
    mixed = (mult_x ^ mult_y) & ((1 << 36) - 1)
    return (mixed >> 19) & ((1 << 6) - 1)


class Entry:
    def __init__(self, st=ST_EMPTY, kx=0, ky=0, pn=0, ts=0):
        self.st, self.kx, self.ky, self.pn, self.ts = st, kx, ky, pn, ts

    def is_free_or_tomb(self):
        return self.st in (ST_EMPTY, ST_TOMB)

    def is_occu_and_match(self, x, y):
        return self.st == ST_OCCU and self.kx == x and self.ky == y


class HashBucketTableSim:
    def __init__(self, life_cycle=100):
        self.life_cycle = life_cycle
        self.tables = [
            [Entry() for _ in range(BUCKETS_PER_TABLE)] for _ in range(NUM_TABLES)
        ]
        self.global_timer = 0
        self.rr_ptr = 0

    def process(self, key_x: int, key_y: int):
        x = sign_extend(key_x, COORD_WIDTH)
        y = sign_extend(key_y, COORD_WIDTH)
        hashes = [hash_bucket_id_rtl(x, y, seed=i) for i in range(NUM_TABLES)]
        entries = [self.tables[i][h] for i, h in enumerate(hashes)]

        hit_idx, free_idx = -1, -1

        # HIT 检查
        for i in range(NUM_TABLES):
            if entries[i].is_occu_and_match(x, y):
                hit_idx = i
                break

        # Insert 检查
        if hit_idx == -1:
            check_order = [(self.rr_ptr + i) % NUM_TABLES for i in range(NUM_TABLES)]
            for i in check_order:
                e = entries[i]
                if e.is_free_or_tomb() or (
                    e.st == ST_OCCU and (self.global_timer - e.ts) > self.life_cycle
                ):
                    free_idx = i
                    self.rr_ptr = (self.rr_ptr + 1) % NUM_TABLES
                    break

        table_full = False
        out_idx, out_pn = 0, 0

        if hit_idx != -1:
            old_e = self.tables[hit_idx][hashes[hit_idx]]
            out_pn = 20 if old_e.pn == 20 else mask_u(old_e.pn + 1, VN_WIDTH)
            out_idx = ((hashes[hit_idx] & 0x3F) << 2) | (hit_idx & 0x3)
            self.tables[hit_idx][hashes[hit_idx]] = Entry(
                ST_OCCU, x, y, out_pn, mask_u(self.global_timer, TIMER_WIDTH)
            )
        elif free_idx != -1:
            out_pn = 1
            out_idx = ((hashes[free_idx] & 0x3F) << 2) | (free_idx & 0x3)
            self.tables[free_idx][hashes[free_idx]] = Entry(
                ST_OCCU, x, y, 1, mask_u(self.global_timer, TIMER_WIDTH)
            )
        else:
            table_full = True

        if hit_idx != -1 or free_idx != -1:
            self.global_timer = mask_u(self.global_timer + 1, TIMER_WIDTH)

        return {
            "out_idx": out_idx,
            "out_point_number": out_pn,
            "table_full": table_full,
        }


# -----------------------------
# 2. 体素化流水线计算
# -----------------------------
class VoxelCoorPipe:
    def __init__(self, hash_sim, out_log_file=None):
        self.hash_sim = hash_sim
        self.out_log_file = out_log_file
        self.VOXEL_WIDTH = 11
        self.BOUNDARY_FIX16 = 0x3600  # 54.0m Q8.8
        self.MULT_FACTOR = 436907  # 6.66666

    def float_to_fixed_q8_8(self, val: float) -> int:
        """
        完全对齐硬件 float_to_fixed 模块的比特级行为
        参数: FIXED_WIDTH = 16, FIXED_FRACTIONAL = 8
        """
        # 1. 将 Python 的浮点数转化为 IEEE 754 32-bit 二进制表示
        # '>f' 代表大端单精度浮点，'>I' 代表大端 32-bit 无符号整数
        packed = struct.pack('>f', val)
        float_in = struct.unpack('>I', packed)[0]

        # 2. 解包 IEEE 754 字段
        # assign {fixed_sign, float_exp, float_mantissa} = float_in;
        fixed_sign = (float_in >> 31) & 1
        float_exp = (float_in >> 23) & 0xFF
        float_mantissa = float_in & 0x7FFFFF

        # 3. 补充隐藏的高位 '1'
        # wire [31:0] working_out = {1'b1, float_mantissa, 8'h0};
        working_out = (1 << 31) | (float_mantissa << 8)

        # 4. 计算移位距离
        # wire [7:0] shift_dist = 8'd127 + (FIXED_WIDTH - FIXED_FRACTIONAL) - 1 - float_exp;
        # 代入参数: 127 + (16 - 8) - 1 = 134
        shift_dist = (134 - float_exp) & 0xFF

        # 5. 移位截断判断
        # wire [4:0] trunc_shift_dist = (|shift_dist[7:5]) ? 5'b11111 : shift_dist[4:0];
        if (shift_dist & 0xE0) != 0:  # 如果高3位 (7,6,5) 不全为0，则钳位到 31
            trunc_shift_dist = 31
        else:
            trunc_shift_dist = shift_dist & 0x1F

        # 6. 执行右移
        # wire [31:0] shifted_out = working_out >> trunc_shift_dist;
        shifted_out = working_out >> trunc_shift_dist

        # 7. 提取幅值
        # assign fixed_mag = {1'b0, shifted_out[30:32-FIXED_WIDTH]};
        # 对于 16-bit 宽度，提取 [30:16]，共 15 bit，最高位强行补 0 确保是正数
        fixed_mag = (shifted_out >> 16) & 0x7FFF

        # 8. 赋予符号
        # assign true_fixed_value = fixed_sign ? -fixed_mag : fixed_mag;
        if fixed_sign == 1:
            true_fixed_value = (-fixed_mag) & 0xFFFF
        else:
            true_fixed_value = fixed_mag & 0xFFFF

        # 9. 将硬件的 16-bit 补码形式转换为 Python 识别的带符号十进制整数返回
        if true_fixed_value & 0x8000:
            out_signed = true_fixed_value - 0x10000
        else:
            out_signed = true_fixed_value

        return out_signed

    def fxp_mul_bit_true(self, ina: int, inb: int) -> int:
        """
        完全对齐硬件 fxp_mul 与 fxp_zoom 模块的比特级行为
        针对实例化参数: WIIA=8, WIFA=8, WIIB=20, WIFB=0, WOI=36, WOF=0, ROUND=1
        """
        # ==================== fxp_mul 模块逻辑 ====================
        # 1. 有符号乘法，结果应为 36-bit (WRI=28, WRF=8)
        res = ina * inb

        # 将 Python 的无限精度整数，转为 36-bit 的二进制补码形式 (对应 wire [WRI+WRF-1:0] res)
        res_36b = res & ((1 << 36) - 1)

        # ==================== fxp_zoom 模块逻辑 ====================
        # 进入 zoom，参数为: WII=28, WIF=8, WOI=36, WOF=0

        # inr = in[WII+WIF-1 : WIF-WOF]; 即 in[35:8]
        inr = (res_36b >> 8) & ((1 << 28) - 1)

        # in[WIF-WOF-1]; 即进位判断位 in[7]
        bit_7 = (res_36b >> 7) & 1

        # 硬件防溢出逻辑: ~(~inr[27] & (&inr[26:0]))
        inr_27 = (inr >> 27) & 1
        inr_26_to_0 = inr & ((1 << 27) - 1)
        all_ones = inr_26_to_0 == ((1 << 27) - 1)

        # if (in[7] & ~(~inr[27] & (&inr[26:0]))) inr = inr + 1;
        if bit_7 == 1 and not (inr_27 == 0 and all_ones):
            inr = (inr + 1) & ((1 << 28) - 1)

        # 符号位扩展生成 outi (WOI=36, WII=28)
        # outi = ini[WII-1] ? {WOI{1'b1}} : 0;
        # outi[WII-1:0] = ini;
        ini = inr
        sign_bit = (ini >> 27) & 1
        if sign_bit == 1:
            # 高 8 位 (36-28) 补 1
            outi = ini | (((1 << 8) - 1) << 28)
        else:
            outi = ini

        # 将 36-bit 的硬件补码转换回 Python 能看懂的带符号十进制整数
        if outi & (1 << 35):
            out_signed = outi - (1 << 36)
        else:
            out_signed = outi

        return out_signed

    def process_point(self, x_f32, y_f32, z_f32, intensity_f32):
        x_fix = self.float_to_fixed_q8_8(x_f32)
        y_fix = self.float_to_fixed_q8_8(y_f32)
        z_fix = self.float_to_fixed_q8_8(z_f32)

        # 边界检查
        if abs(x_fix) > self.BOUNDARY_FIX16 or abs(y_fix) > self.BOUNDARY_FIX16:
            return False
        if z_fix < 0 and abs(z_fix) > 0x0500:
            return False
        if z_fix >= 0 and abs(z_fix) > 0x0300:
            return False

        # 3. 计算体素坐标 (Voxel Coordinate)
        add_x = x_fix + self.BOUNDARY_FIX16
        # 使用严格比特级对齐的硬件乘法器
        fxp_mul_x_out = self.fxp_mul_bit_true(add_x, self.MULT_FACTOR)
        voxel_x = (fxp_mul_x_out >> 16) & ((1 << self.VOXEL_WIDTH) - 1)

        add_y = y_fix + self.BOUNDARY_FIX16
        # 使用严格比特级对齐的硬件乘法器
        fxp_mul_y_out = self.fxp_mul_bit_true(add_y, self.MULT_FACTOR)
        voxel_y = (fxp_mul_y_out >> 16) & ((1 << self.VOXEL_WIDTH) - 1)
        # 喂入哈希表
        hash_res = self.hash_sim.process(voxel_x, voxel_y)

        if not hash_res["table_full"]:
            out_idx = hash_res["out_idx"]

            if self.out_log_file:
                self.out_log_file.write(f"{voxel_x}, {voxel_y}, {out_idx}\n")
            else:
                print(f"{voxel_x}, {voxel_y}, {out_idx}")
            return True
        return False


# -----------------------------
# 3. 解析 128-bit 字符串数据并运行
# -----------------------------
def decode_binary_float(bin_str):
    """将 32位 二进制01字符串解析为 float32"""
    # 转为整数 -> 转为4字节bytes -> unpack为float
    return struct.unpack('>f', int(bin_str, 2).to_bytes(4, 'big'))[0]


def run_pipeline_from_txt(
    txt_file, voxel_out_file="voxel_output.txt", expire_out_file="expire_output.txt"
):
    with open(txt_file, 'r') as f_in, open(voxel_out_file, 'w') as f_vox, open(
        expire_out_file, 'w'
    ) as f_exp:

        hash_sim = HashBucketTableSim(expire_log_file=f_exp)
        pipe = VoxelCoorPipe(hash_sim, out_log_file=f_vox)

        f_vox.write("Voxel_X, Voxel_Y, Out_Idx\n")

        # ==========================================
        # 新增：窗口统计变量
        # ==========================================
        point_count = 0
        hash_load_sum = 0
        max_cam_load = 0

        for line in f_in:
            line = line.strip()
            if len(line) != 128:
                continue

            # 按照 Verilog 的位宽截断 (高位到低位：X, Y, Z, Intensity)
            x_bin = line[0:32]
            y_bin = line[32:64]
            z_bin = line[64:96]
            i_bin = line[96:128]

            x_f32 = decode_binary_float(x_bin)
            y_f32 = decode_binary_float(y_bin)
            z_f32 = decode_binary_float(z_bin)
            i_f32 = decode_binary_float(i_bin)

            pipe.process_point(x_f32, y_f32, z_f32, i_f32)

            # ==========================================
            # 每处理一个点，获取当前负载并进行窗口统计
            # ==========================================
            point_count += 1
            hash_used, hash_pct, cam_used, cam_pct = hash_sim.get_overall_loads()

            # 累加 Hash 负载算平均
            hash_load_sum += hash_used
            # 记录 CAM 负载算峰值
            if cam_used > max_cam_load:
                max_cam_load = cam_used

            # 每 1000 个点打印一次
            if point_count % 1000 == 0:
                avg_hash_load = hash_load_sum / 1000.0
                avg_hash_pct = (avg_hash_load / TOTAL_CAPACITY) * 100.0
                max_cam_pct = (max_cam_load / hash_sim.CAM_DEPTH) * 100.0

                print(
                    f"[Monitor] Processed {point_count:06d} points | "
                    f"Avg Hash Load: {avg_hash_load:5.1f}/{TOTAL_CAPACITY} ({avg_hash_pct:5.2f}%) | "
                    f"Max CAM Load: {max_cam_load:2d}/{hash_sim.CAM_DEPTH} ({max_cam_pct:5.2f}%)"
                )

                # 打印完后清零，为下一个 1000 点重新统计
                hash_load_sum = 0
                max_cam_load = 0

        # ==========================================
        # 循环结束后，处理最后不足 1000 的零头数据
        # ==========================================
        remainder = point_count % 1000
        if remainder != 0:
            avg_hash_load = hash_load_sum / remainder
            avg_hash_pct = (avg_hash_load / TOTAL_CAPACITY) * 100.0
            max_cam_pct = (max_cam_load / hash_sim.CAM_DEPTH) * 100.0

            print(
                f"[Monitor] Processed {point_count:06d} points (Final) | "
                f"Avg Hash Load: {avg_hash_load:5.1f}/{TOTAL_CAPACITY} ({avg_hash_pct:5.2f}%) | "
                f"Max CAM Load: {max_cam_load:2d}/{hash_sim.CAM_DEPTH} ({max_cam_pct:5.2f}%)"
            )


# -----------------------------
# 常量定义
# -----------------------------
ST_EMPTY = 0b00
ST_OCCU = 0b01
ST_TOMB = 0b10

COORD_WIDTH = 11
VN_WIDTH = 5
TIMER_WIDTH = 16

# 4-Choice 架构配置
NUM_TABLES = 4  # 对应 4 个 Seed
BUCKETS_PER_TABLE = 64  # 对应 BUCKET_AW = 6
TOTAL_CAPACITY = NUM_TABLES * BUCKETS_PER_TABLE  # 256
ADDR_WIDTH = 8  # 2 bits Table ID + 6 bits Bucket ID

LIFE_CYCLE = 100


# -----------------------------
# 工具函数
# -----------------------------
def mask_u(val: int, width: int) -> int:
    return val & ((1 << width) - 1)


def sign_extend(val: int, width: int) -> int:
    val = mask_u(val, width)
    sign = 1 << (width - 1)
    return val - (1 << width) if (val & sign) else val


# -----------------------------
# 哈希函数 (对齐 Verilog 4-Choice)
# -----------------------------
def hash_bucket_id_rtl(key_x: int, key_y: int, seed: int) -> int:
    """
    Python 仿真版 hash_func_multiplicative
    支持 SEED 参数 (0, 1, 2, 3)
    """
    COORD_WIDTH = 11
    BUCKET_AW = 6
    PRIME_X = 10368889
    PRIME_Y = 10000169

    # 1. 输入扰动 (Input Perturbation)
    # 模拟 signed [10:0] 运算
    x = sign_extend(key_x, COORD_WIDTH)
    y = sign_extend(key_y, COORD_WIDTH)

    mod_x = x
    mod_y = y

    if seed == 0:
        pass  # 原始
    elif seed == 1:
        mod_x = ~x  # 取反
    elif seed == 2:
        mod_y = ~y  # 取反
    elif seed == 3:
        # 异或 8'h5A / 8'hA5
        # 注意：Verilog 中 8'h5A 会被符号扩展还是零扩展取决于具体写法，
        # 通常 x ^ 8'h5A 会将 0x5A 视为同位宽整数。
        # 这里假设 Verilog: mod_x = key_x ^ 'h5A (低8位异或)
        mod_x = x ^ 0x5A
        mod_y = y ^ 0xA5

    # 再次确保位宽 (模拟寄存器截断)
    mod_x = sign_extend(mod_x, COORD_WIDTH)
    mod_y = sign_extend(mod_y, COORD_WIDTH)

    # 2. 乘法核心
    mult_x = mod_x * PRIME_X
    mult_y = mod_y * PRIME_Y

    # 截断为 36-bit
    mult_x &= (1 << 36) - 1
    mult_y &= (1 << 36) - 1

    # 3. 混合
    mixed = mult_x ^ mult_y
    mixed &= (1 << 36) - 1

    # 4. 输出高位 (mixed[24:19])
    hash_out = (mixed >> 19) & ((1 << BUCKET_AW) - 1)
    # print(f"hash_out={hash_out:02x}, mixed={mixed:09x} (mult_x={mult_x:09x}, mult_y={mult_y:09x})")

    return hash_out


# -----------------------------
# 表项结构
# -----------------------------
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

    # -----------------------------
    # 过期管理器
    # -----------------------------


class ExpireManagerRTL:
    def __init__(self, life_cycle: int, expire_log_file=None):
        self.life_cycle = int(life_cycle)
        self.expire_log_file = expire_log_file
        self.dq = deque()
        self.expired_events = []

    def _calc_hash_loads(self, time_now: int, tables: List[List[Entry]]):
        """
        统计当前 4 张 hash table 的有效负载。
        返回:
            table_loads: [table0_load, table1_load, table2_load, table3_load]
            total_load : 0 ~ 256
        """
        table_loads = [0] * NUM_TABLES

        for t in range(NUM_TABLES):
            for b in range(BUCKETS_PER_TABLE):
                e = tables[t][b]

                if e.st != ST_OCCU:
                    continue

                dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)

                # 这里统计“有效负载”：OCCU 且还没过期
                # 注意这里和 expire 判断保持一致，dt >= life_cycle 认为已经过期
                if dt < self.life_cycle:
                    table_loads[t] += 1

        total_load = sum(table_loads)
        return table_loads, total_load

    def on_write_commit(
        self, time_now: int, write_addr: int, tables: List[List[Entry]], cam: List[dict]
    ):
        """
        tables: 4个 list，每个 list 是 buckets
        write_addr: 8-bit {bucket_id[5:0], table_id[1:0]}
        cam: 用于同步废除悬空指针
        """
        # push
        if len(self.dq) < self.life_cycle:
            self.dq.append(write_addr)
            return

        # pop cand
        cand_addr = self.dq.popleft()
        self.dq.append(write_addr)

        # Decode address
        table_id = cand_addr & 0x3
        bucket_id = (cand_addr >> 2) & 0x3F

        e = tables[table_id][bucket_id]

        # expired 判断
        dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)

        if e.st == ST_OCCU and dt >= self.life_cycle:
            # 先保存被 killed 的信息
            killed_kx = e.kx
            killed_ky = e.ky
            killed_pn = e.pn

            # 产生过期事件
            self.expired_events.append((killed_kx, killed_ky, cand_addr, killed_pn))

            # 写 tomb
            tables[table_id][bucket_id] = Entry(st=ST_TOMB, kx=0, ky=0, pn=0, ts=0)

            # 同步追踪：防悬空指针，清理 CAM
            for c in cam:
                if c['valid'] and c['mapped_idx'] == cand_addr:
                    c['valid'] = False

            # 在 kill 之后统计当前 hash 负载
            table_loads, total_load = self._calc_hash_loads(time_now, tables)

            if self.expire_log_file:
                self.expire_log_file.write(
                    f"Expire Event: "
                    f"Point=({killed_kx}, {killed_ky}), "
                    f"Addr={cand_addr}, "
                    f"PN={killed_pn}, "
                    f"Time={time_now}, "
                    f"DT={dt}, "
                    f"HashLoad={total_load}/{TOTAL_CAPACITY}, "
                    f"TableLoads={table_loads}\n"
                )


# -----------------------------
# 4-Choice 仿真器主体
# -----------------------------
# -----------------------------
# 4-Choice 仿真器主体
# -----------------------------
class HashBucketTableSim:
    def __init__(self, life_cycle: int = LIFE_CYCLE, expire_log_file=None):
        self.life_cycle = life_cycle
        self.expire_mgr = ExpireManagerRTL(life_cycle, expire_log_file)

        # 4 个表，每个表 64 个 Bucket
        self.tables: List[List[Entry]] = [
            [Entry() for _ in range(BUCKETS_PER_TABLE)] for _ in range(NUM_TABLES)
        ]

        self.global_timer: int = 0
        self.rr_ptr: int = 0

        # 【新增】内部 TLB 地址映射表 (CAM 结构)
        self.CAM_DEPTH = 16
        self.cam = [
            {'valid': False, 'kx': 0, 'ky': 0, 'mapped_idx': 0, 'pn': 0}
            for _ in range(self.CAM_DEPTH)
        ]

    def _pack_out_idx(self, table_id: int, bucket_id: int) -> int:
        return ((bucket_id & 0x3F) << 2) | (table_id & 0x3)

    def _is_expired(self, e: Entry) -> bool:
        if e.st != ST_OCCU:
            return False
        return (self.global_timer - e.ts) > self.life_cycle

    def get_bram_loads(self):
        loads = [0] * NUM_TABLES
        for t in range(NUM_TABLES):
            for b in range(BUCKETS_PER_TABLE):
                e = self.tables[t][b]
                if e.st == ST_OCCU and (not self._is_expired(e)):
                    loads[t] += 1

        capacity_each = BUCKETS_PER_TABLE
        pct = [100.0 * x / capacity_each for x in loads]
        return loads, pct

    def get_overall_loads(self):
        """获取 Hash 表和 CAM 整体的容量和百分比"""
        # 1. 计算 Hash Table 容量
        hash_used = 0
        for t in range(NUM_TABLES):
            for b in range(BUCKETS_PER_TABLE):
                e = self.tables[t][b]
                # 必须占用且未过期才算作有效负载
                if e.st == ST_OCCU and (not self._is_expired(e)):
                    hash_used += 1
        hash_pct = (hash_used / TOTAL_CAPACITY) * 100.0

        # 2. 计算 CAM 容量
        cam_used = sum(1 for c in self.cam if c['valid'])
        cam_pct = (cam_used / self.CAM_DEPTH) * 100.0

        return hash_used, hash_pct, cam_used, cam_pct

    def process(self, key_x: int, key_y: int):
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

        # ---------------------------------------------------------
        # 1. 查询 CAM (等效于 action_hit_cam)
        # ---------------------------------------------------------
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
            # CAM Hit 透写更新底层 BRAM
            found = True
            mapped_idx = self.cam[cam_hit_idx]['mapped_idx']
            out_idx = mapped_idx

            table_id = mapped_idx & 0x3
            bucket_id = (mapped_idx >> 2) & 0x3F

            # 提前计算点数
            old_pn = self.cam[cam_hit_idx]['pn']
            new_pn = 20 if old_pn >= 20 else mask_u(old_pn + 1, VN_WIDTH)

            out_pn = new_pn
            self.cam[cam_hit_idx]['pn'] = new_pn

            # 写入底层表
            self.tables[table_id][bucket_id] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True

        else:
            # ---------------------------------------------------------
            # 2. 正常查询 + Linear Probing 探测循环
            # ---------------------------------------------------------
            hit_idx = -1
            free_idx = -1
            target_bucket = -1
            probe_offset = 0

            # 模拟硬件探测：从 0(Base) 探到 63 (BUCKET_AW 一整圈)
            for offset in range(BUCKETS_PER_TABLE):
                eval_buckets = [(h + offset) & 0x3F for h in hashes]

                # 检查此 Offset 下是否有 Hit
                for i in range(NUM_TABLES):
                    if self.tables[i][eval_buckets[i]].is_occu_and_match(x, y):
                        hit_idx = i
                        target_bucket = eval_buckets[i]
                        break

                if hit_idx != -1:
                    probe_offset = offset
                    break

                # 检查此 Offset 下是否有 Free/Tomb
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

            # ---------------------------------------------------------
            # 3. 根据探测结果执行动作
            # ---------------------------------------------------------
            if hit_idx != -1:
                # Base Hit 或 Probe Hit 更新 (等效于 action_hit_base)
                found = True
                old_e = self.tables[hit_idx][target_bucket]
                new_pn = 20 if old_e.pn >= 20 else mask_u(old_e.pn + 1, VN_WIDTH)

                out_pn = new_pn
                out_idx = self._pack_out_idx(hit_idx, target_bucket)
                self.tables[hit_idx][target_bucket] = Entry(
                    st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
                )
                did_write = True

            elif free_idx != -1:
                # 找到空位插入 (等效于 action_free)
                found = True
                out_pn = 1
                out_idx = self._pack_out_idx(free_idx, target_bucket)

                self.tables[free_idx][target_bucket] = Entry(
                    st=ST_OCCU, kx=x, ky=y, pn=1, ts=mask_u(time_now, TIMER_WIDTH)
                )
                did_write = True

                # 如果是 Probing 找到的空位，必须追加记录到 CAM
                if probe_offset > 0:
                    cam_free_idx = -1
                    for j in range(self.CAM_DEPTH):
                        if not self.cam[j]['valid']:
                            cam_free_idx = j
                            break

                    # 硬件逻辑：如果 CAM 满，使用 global_timer 的伪随机进行替换
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
                # 真满 (等效于 action_true_full)
                table_full = True

        # ---------------------------------------------------------
        # 4. 写提交 & 全局计时
        # ---------------------------------------------------------
        if did_write:
            # 传入 cam 列表用于同步清理
            self.expire_mgr.on_write_commit(time_now, out_idx, self.tables, self.cam)
            self.global_timer = mask_u(self.global_timer + 1, TIMER_WIDTH)

        return {
            "busy": busy,
            "found": found,
            "out_idx": out_idx,
            "out_point_number": out_pn,
            "table_full": table_full,
        }


def merge_voxel_csv(
    input_csv: str, output_csv: str = "merged_data_sim.csv", clip_upper: int = 20
):
    """按 x/y 聚合 value 并裁剪上限，最后输出新 CSV。"""
    df2 = pd.read_csv(input_csv)
    merged_df_2 = df2.groupby(['x', 'y'], as_index=False)['value'].sum()
    merged_df_2['value'] = merged_df_2['value'].clip(upper=clip_upper)

    print(merged_df_2.head())
    merged_df_2.to_csv(output_csv, index=False)
    print(f"合并完成！已保存为 {output_csv}")

    return merged_df_2


def interactive_voxel_debug():
    """
    交互式体素化单步调试工具。
    允许从命令行输入 x, y 坐标，输出流水线每个节点的中间硬件状态。
    """
    # 实例化一个空的哈希表，仅作为 VoxelCoorPipe 的依赖
    dummy_hash = HashBucketTableSim()
    pipe = VoxelCoorPipe(dummy_hash)

    print("=" * 50)
    print("🚀 启动体素化模块 (VoxelCoorPipe) 交互调试器")
    print("请输入浮点坐标 x, y (例如: 12.5, -5.2)")
    print("输入 'q' 或 'exit' 退出调试")
    print("=" * 50)

    while True:
        try:
            user_input = input("\n> x, y = ")
            if user_input.strip().lower() in ['q', 'exit', 'quit']:
                print("退出调试。")
                break

            # 解析输入
            parts = user_input.split(',')
            if len(parts) != 2:
                print("❌ 格式错误！请输入用逗号分隔的两个数字。")
                continue

            x_f32, y_f32 = float(parts[0].strip()), float(parts[1].strip())

            # -------------------------
            # 提取流水线各个阶段的数据
            # -------------------------
            # 1. 浮点转定点 (Q8.8)
            x_fix = pipe.float_to_fixed_q8_8(x_f32)
            y_fix = pipe.float_to_fixed_q8_8(y_f32)

            # 2. 加上边界偏移
            add_x = x_fix + pipe.BOUNDARY_FIX16
            add_y = y_fix + pipe.BOUNDARY_FIX16

            # 3. 硬件乘法
            fxp_mul_x_out = pipe.fxp_mul_bit_true(add_x, pipe.MULT_FACTOR)
            fxp_mul_y_out = pipe.fxp_mul_bit_true(add_y, pipe.MULT_FACTOR)

            # 4. 移位截断得到体素坐标
            voxel_x = (fxp_mul_x_out >> 16) & ((1 << pipe.VOXEL_WIDTH) - 1)
            voxel_y = (fxp_mul_y_out >> 16) & ((1 << pipe.VOXEL_WIDTH) - 1)

            # -------------------------
            # 打印硬件级细节
            # -------------------------
            print(f"【输入浮点】 x = {x_f32:.6f}, y = {y_f32:.6f}")
            print(
                f"【Q8.8定点】 x_fix = {x_fix} (0x{x_fix & 0xFFFF:04X}), y_fix = {y_fix} (0x{y_fix & 0xFFFF:04X})"
            )
            print(
                f"【偏移处理】 add_x = {add_x} (0x{add_x & 0xFFFF:04X}), add_y = {add_y} (0x{add_y & 0xFFFF:04X})"
            )
            print(
                f"【定点乘法】 mul_x = {fxp_mul_x_out} (0x{fxp_mul_x_out & 0xFFFFFFFFF:09X})"
            )
            print(
                f"             mul_y = {fxp_mul_y_out} (0x{fxp_mul_y_out & 0xFFFFFFFFF:09X})"
            )
            print(f"【最终Voxel】 voxel_x = {voxel_x}, voxel_y = {voxel_y}")
            print("-" * 50)

        except ValueError:
            print("❌ 数据解析失败，请确保输入的是数字！")
        except Exception as e:
            print(f"❌ 运行报错: {e}")


# ================= 测试你的数据 =================
if __name__ == "__main__":
    # 运行解析并打印

    # 如果在命令行后加了 "debug" 参数，则进入交互模式 (例如：python script.py debug)
    # interactive_voxel_debug()

    point_txt_path = r"E:\verilog\pillarnest\rtl\.pointcloud\points_bits_128.txt"
    killed_txt_path = (
        "python\output_sim_bitlevel\output_simulation_soft_bitlevel_killed.txt"
    )

    run_pipeline_from_txt(
        point_txt_path,
        voxel_out_file="python\\output_sim_bitlevel\\output_simulation_hard_voxel.txt",
        expire_out_file=killed_txt_path,
    )

    killed_csv_path = r"E:\verilog\pillarnest\python\output_sim_bitlevel\output_simulation_sim_bitlevel_killed.csv"
    merged_csv_path = (
        r"E:\verilog\pillarnest\python\output_sim_bitlevel\merged_data_sim_bitlevel.csv"
    )

    count = convert_sim_to_csv(
        input_file=killed_txt_path,
        output_file=killed_csv_path,
        verbose=True,
    )

    merge_voxel_csv(
        input_csv=killed_csv_path,
        output_csv=merged_csv_path,
        clip_upper=20,
    )
