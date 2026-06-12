# 4-choice hash

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import pandas as pd
from collections import deque
from python.scripts.csv_sim_process import convert_sim_to_csv

import struct
import math

class VoxelCoorPipe:
    # 增加 out_log_file 参数
    def __init__(self, hash_sim, out_log_file=None):
        self.hash_sim = hash_sim
        self.out_log_file = out_log_file
        self.VOXEL_WIDTH = 11
        
        # 物理边界参数 (对应原代码中的 Q8.8 格式硬编码)
        self.X_MIN = 0
        self.X_MAX = 69.12
        self.Y_MIN = -39.68
        self.Y_MAX = 39.68
        self.Z_MIN = -3.0
        self.Z_MAX = 1.0
        
        # 体素大小 (原始 MULT_FACTOR 436907 对应的是 1/0.15 的定点近似)
        self.VOXEL_SIZE = 0.16

    def process_point(self, x_f32: float, y_f32: float, z_f32: float, intensity_f32: float) -> bool:
        if not (self.X_MIN <= x_f32 <= self.X_MAX): return False
        if not (self.Y_MIN <= y_f32 <= self.Y_MAX): return False
        if not (self.Z_MIN <= z_f32 <= self.Z_MAX): return False

        voxel_x = int((x_f32 - self.X_MIN) / self.VOXEL_SIZE) & ((1 << self.VOXEL_WIDTH) - 1)
        voxel_y = int((y_f32 - self.Y_MIN) / self.VOXEL_SIZE) & ((1 << self.VOXEL_WIDTH) - 1)

        hash_res = self.hash_sim.process(voxel_x, voxel_y)
        
        if not hash_res["table_full"]:
            out_idx = hash_res["out_idx"]
            
            # 【修改点】写入到指定文件，并手动加上换行符 \n
            if self.out_log_file:
                self.out_log_file.write(f"{voxel_x}, {voxel_y}, {out_idx}\n")
            return True
        return False


# -----------------------------
# 常量定义
# -----------------------------
ST_EMPTY = 0b00
ST_OCCU  = 0b01
ST_TOMB  = 0b10

COORD_WIDTH = 11
VN_WIDTH = 6
TIMER_WIDTH = 18

# 4-Choice 架构配置
NUM_TABLES = 4         # 对应 4 个 Seed
BUCKETS_PER_TABLE = 64 # 对应 BUCKET_AW = 6
TOTAL_CAPACITY = NUM_TABLES * BUCKETS_PER_TABLE # 256
ADDR_WIDTH = 8         # 2 bits Table ID + 6 bits Bucket ID

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
# 3. 解析 128-bit 字符串数据并运行
# -----------------------------
def decode_binary_float(bin_str):
    """将 32位 二进制01字符串解析为 float32"""
    # 转为整数 -> 转为4字节bytes -> unpack为float
    return struct.unpack('>f', int(bin_str, 2).to_bytes(4, 'big'))[0]

def run_pipeline_from_txt(txt_file, voxel_out_file="voxel_output.txt", expire_out_file="expire_output.txt"):
    # 【修改点】同时打开输入文件和两个输出文件
    with open(txt_file, 'r') as f_in, \
         open(voxel_out_file, 'w') as f_vox, \
         open(expire_out_file, 'w') as f_exp:
             
        # 将文件句柄传入实例
        hash_sim = HashBucketTableSim(expire_log_file=f_exp)
        pipe = VoxelCoorPipe(hash_sim, out_log_file=f_vox)
        
        # 将表头也写入 voxel 记录文件
        f_vox.write("Voxel_X, Voxel_Y, Out_Idx\n")
        f_vox.write("-" * 30 + "\n")

        for line in f_in:
            line = line.strip()
            if len(line) != 128:
                continue
            
            x_bin = line[0:32]
            y_bin = line[32:64]
            z_bin = line[64:96]
            i_bin = line[96:128]

            x_f32 = decode_binary_float(x_bin)
            y_f32 = decode_binary_float(y_bin)
            z_f32 = decode_binary_float(z_bin)
            i_f32 = decode_binary_float(i_bin)
            
            pipe.process_point(x_f32, y_f32, z_f32, i_f32)


def merge_voxel_csv(input_csv: str, output_csv: str = "merged_data_sim.csv", clip_upper: int = 20):
    """按 x/y 聚合 value 并裁剪上限，最后输出新 CSV。"""
    df2 = pd.read_csv(input_csv)
    merged_df_2 = df2.groupby(['x', 'y'], as_index=False)['value'].sum()
    merged_df_2['value'] = merged_df_2['value'].clip(upper=clip_upper)

    print(merged_df_2.head())
    merged_df_2.to_csv(output_csv, index=False)
    print(f"合并完成！已保存为 {output_csv}")

    return merged_df_2

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
        pass # 原始
    elif seed == 1:
        mod_x = ~x # 取反
    elif seed == 2:
        mod_y = ~y # 取反
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
    # 增加 expire_log_file 参数
    def __init__(self, life_cycle: int, expire_log_file=None):
        self.life_cycle = int(life_cycle)
        self.expire_log_file = expire_log_file
        self.dq = deque()
        self.expired_events = []

    def on_write_commit(self, time_now: int, write_addr: int, tables: List[List[Entry]]):
        if len(self.dq) < self.life_cycle:
            self.dq.append(write_addr)
            return

        cand_addr = self.dq.popleft()
        self.dq.append(write_addr)

        table_id = cand_addr & 0x3
        bucket_id = (cand_addr & 0xFC) >> 2
        
        e = tables[table_id][bucket_id]

        dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)
        if e.st == ST_OCCU and dt >= self.life_cycle:
            self.expired_events.append((e.kx, e.ky, cand_addr, e.pn))
            
            # 【修改点】写入到指定文件，并手动加上换行符 \n
            if self.expire_log_file:
                self.expire_log_file.write(f"Expire Event: Point=({e.kx}, {e.ky}), Addr={cand_addr}, PN={e.pn}, Time={time_now}, DT={dt}\n")
                
            tables[table_id][bucket_id] = Entry(st=ST_TOMB, kx=0, ky=0, pn=0, ts=0)
# -----------------------------
# 4-Choice 仿真器主体
# -----------------------------
class HashBucketTableSim:
    # 增加 expire_log_file 参数
    def __init__(self, life_cycle: int = LIFE_CYCLE, expire_log_file=None):
        self.life_cycle = life_cycle
        # 传递给 ExpireManagerRTL
        self.expire_mgr = ExpireManagerRTL(life_cycle, expire_log_file)

        self.tables: List[List[Entry]] = [
            [Entry() for _ in range(BUCKETS_PER_TABLE)]
            for _ in range(NUM_TABLES)
        ]

        self.global_timer: int = 0
        self.rr_ptr: int = 0

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
    
    def process(self, key_x: int, key_y: int):
        # 1. 坐标符号扩展
        x = sign_extend(key_x, COORD_WIDTH)
        y = sign_extend(key_y, COORD_WIDTH)

        time_now = self.global_timer

        # 2. 并行计算 4 个哈希值
        hashes = [hash_bucket_id_rtl(x, y, seed=i) for i in range(NUM_TABLES)]
        
        # 3. 并行读取
        entries = [self.tables[i][h] for i, h in enumerate(hashes)]

        busy = True
        found = False
        table_full = False
        out_idx = 0
        out_pn = 0
        did_write = False

        hit_idx = -1
        free_idx = -1

        # ---------------------------------------------------------
        # 4. 检查 HIT (优先级最高，不受 RR 影响)
        # ---------------------------------------------------------
        for i in range(NUM_TABLES):
            if entries[i].is_occu_and_match(x, y):
                hit_idx = i
                break 
        
        # ---------------------------------------------------------
        # 5. 检查 INSERT (受 RR 指针控制)
        # ---------------------------------------------------------
        if hit_idx == -1:
            # 【核心修改】根据 rr_ptr 生成查找顺序
            # 比如 rr_ptr=1 -> check_order=[1, 2, 3, 0]
            check_order = [(self.rr_ptr + i) % NUM_TABLES for i in range(NUM_TABLES)]
            
            for i in check_order:
                e = entries[i]
                # 检查是否为空位 (Empty/Tomb) 或已过期
                if e.is_free_or_tomb() or self._is_expired(e):
                    free_idx = i
                    # print(f"Free slot found at table {i} (rr_ptr={self.rr_ptr})")
                    # 【核心修改】模拟 Verilog: rr_ptr <= rr_ptr + 1
                    # 只有在发生 Free Insert 时才更新指针
                    self.rr_ptr = (self.rr_ptr + 1) % NUM_TABLES
                    break
        
        # 6. 执行写入/更新
        target_table = -1
        target_bucket = -1

        if hit_idx != -1:
            # HIT: Update (不更新 rr_ptr)
            found = True
            target_table = hit_idx
            target_bucket = hashes[hit_idx]
            old_e = self.tables[target_table][target_bucket]
            
            # 当 pn 等于 32 时，不再加 1，保持为 32
            if old_e.pn == 32:
                new_pn = 32
            else:
                new_pn = mask_u(old_e.pn + 1, VN_WIDTH)
            
            out_pn = new_pn
            out_idx = self._pack_out_idx(target_table, target_bucket)
            
            self.tables[target_table][target_bucket] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True

        elif free_idx != -1:
            # MISS but Free: Insert (rr_ptr 已在上面更新)
            found = True
            target_table = free_idx
            target_bucket = hashes[free_idx]
            
            out_pn = 1
            out_idx = self._pack_out_idx(target_table, target_bucket)
            # print(f"Inserting at table {target_table}, bucket {target_bucket} out_idx={out_idx} (rr_ptr={self.rr_ptr})")
            
            self.tables[target_table][target_bucket] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=1, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True
        
        else:
            # Full
            table_full = True

        # 7. 写提交 & 全局计时
        if did_write:
            self.expire_mgr.on_write_commit(time_now, out_idx, self.tables)
            self.global_timer = mask_u(self.global_timer + 1, TIMER_WIDTH)
            # print(x, y, out_idx)

        return {
            "busy": busy,
            "found": found,
            "out_idx": out_idx,
            "out_point_number": out_pn,
            "table_full": table_full
        }
    
# ================= 测试你的数据 =================
if __name__ == "__main__":
    # 运行解析并打印
    point_txt_path = r"E:\verilog\pillarnest\rtl\.pointcloud\points_bits_128.txt"
    killed_txt_path= "python\output_sim\output_simulation_soft_killed.txt"
    
    run_pipeline_from_txt(point_txt_path, voxel_out_file="python\output_sim\output_simulation_soft_voxel.txt", expire_out_file=killed_txt_path)

    killed_csv_path = r"E:\verilog\pillarnest\python\output_sim\output_simulation_soft_killed.csv"
    merged_csv_path = r"E:\verilog\pillarnest\python\output_sim\merged_data_sim.csv"

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
