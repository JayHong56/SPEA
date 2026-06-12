# 单hash
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import pandas as pd
from collections import deque

# -----------------------------
# 常量定义
# -----------------------------
ST_EMPTY = 0b00
ST_OCCU  = 0b01
ST_TOMB  = 0b10

COORD_WIDTH = 11
VN_WIDTH = 5
TIMER_WIDTH = 16

# 4 张表结构不变
NUM_TABLES = 4
BUCKETS_PER_TABLE = 64
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
# 单一哈希函数
# 4 张表共用同一个 bucket_id
# -----------------------------
def hash_bucket_id_rtl(key_x: int, key_y: int) -> int:
    """
    Python 仿真版单 hash_func_multiplicative
    输出 6-bit bucket_id，供 4 张表共同使用
    """
    BUCKET_AW = 6
    PRIME_X = 10368889
    PRIME_Y = 10000169

    x = sign_extend(key_x, COORD_WIDTH)
    y = sign_extend(key_y, COORD_WIDTH)

    mult_x = x * PRIME_X
    mult_y = y * PRIME_Y

    # 截断为 36-bit
    mult_x &= (1 << 36) - 1
    mult_y &= (1 << 36) - 1

    mixed = mult_x ^ mult_y
    mixed &= (1 << 36) - 1

    # 仍取 mixed[24:19] 作为 6-bit bucket
    hash_out = (mixed >> 19) & ((1 << BUCKET_AW) - 1)
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
    def __init__(self, life_cycle: int):
        self.life_cycle = int(life_cycle)
        self.dq = deque()
        self.expired_events = []

    def on_write_commit(self, time_now: int, write_addr: int, tables: List[List[Entry]]):
        """
        tables: 4个 list，每个 list 是 buckets
        write_addr: 8-bit {table_id[1:0], bucket_id[5:0]}
        """
        if len(self.dq) < self.life_cycle:
            self.dq.append(write_addr)
            return

        cand_addr = self.dq.popleft()
        self.dq.append(write_addr)

        # Decode address
        table_id = cand_addr & 0x3
        bucket_id = (cand_addr & 0xFC) >> 2

        e = tables[table_id][bucket_id]

        dt = (time_now - e.ts) & ((1 << TIMER_WIDTH) - 1)
        if e.st == ST_OCCU and dt >= self.life_cycle:
            self.expired_events.append((e.kx, e.ky, cand_addr, e.pn))
            tables[table_id][bucket_id] = Entry(st=ST_TOMB, kx=0, ky=0, pn=0, ts=0)

# -----------------------------
# 单 hash + 4 表仿真器主体
# -----------------------------
class HashBucketTableSim:
    def __init__(self, life_cycle: int = LIFE_CYCLE):
        self.life_cycle = life_cycle
        self.expire_mgr = ExpireManagerRTL(life_cycle)

        # 4 个表，每个表 64 个 Bucket
        self.tables: List[List[Entry]] = [
            [Entry() for _ in range(BUCKETS_PER_TABLE)]
            for _ in range(NUM_TABLES)
        ]

        self.global_timer: int = 0

        # Round-Robin 指针依然保留
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

        # 2. 只计算一个 hash，4 张表共用
        bucket_id = hash_bucket_id_rtl(x, y)

        # 3. 并行读取 4 张表的同一个 bucket
        entries = [self.tables[i][bucket_id] for i in range(NUM_TABLES)]

        busy = True
        found = False
        table_full = False
        out_idx = 0
        out_pn = 0
        did_write = False

        hit_idx = -1
        free_idx = -1

        # ---------------------------------------------------------
        # 4. 检查 HIT（优先级最高，不受 RR 影响）
        # ---------------------------------------------------------
        for i in range(NUM_TABLES):
            if entries[i].is_occu_and_match(x, y):
                hit_idx = i
                break

        # ---------------------------------------------------------
        # 5. 检查 INSERT（受 RR 指针控制）
        #    4 张表检查的是同一个 bucket_id
        # ---------------------------------------------------------
        if hit_idx == -1:
            check_order = [(self.rr_ptr + i) % NUM_TABLES for i in range(NUM_TABLES)]

            for i in check_order:
                e = entries[i]
                if e.is_free_or_tomb() or self._is_expired(e):
                    free_idx = i
                    self.rr_ptr = (self.rr_ptr + 1) % NUM_TABLES
                    break

        # 6. 执行写入/更新
        target_table = -1
        target_bucket = bucket_id

        if hit_idx != -1:
            # HIT: Update
            found = True
            target_table = hit_idx
            old_e = self.tables[target_table][target_bucket]

            new_pn = mask_u(old_e.pn + 1, VN_WIDTH)
            out_pn = new_pn
            out_idx = self._pack_out_idx(target_table, target_bucket)

            self.tables[target_table][target_bucket] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=new_pn, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True

        elif free_idx != -1:
            # MISS but Free: Insert
            found = True
            target_table = free_idx

            out_pn = 1
            out_idx = self._pack_out_idx(target_table, target_bucket)

            self.tables[target_table][target_bucket] = Entry(
                st=ST_OCCU, kx=x, ky=y, pn=1, ts=mask_u(time_now, TIMER_WIDTH)
            )
            did_write = True

        else:
            # 4 张表这个 bucket 都占满
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

# -----------------------------
# 主程序示例
# -----------------------------
if __name__ == "__main__":
    CSV_FILE = r"E:\verilog\pillarnest\modelsim_pre\ssimulation_result.csv"
    REPORT_INTERVAL = 500

    try:
        df = pd.read_csv(CSV_FILE)
        df.columns = df.columns.str.strip()
        if "Key_X" not in df.columns or "Key_Y" not in df.columns:
            raise ValueError("CSV 缺少 Key_X 或 Key_Y 列")

        sim = HashBucketTableSim()
        print(f"开始仿真: Single Hash + 4 Tables, Total Capacity={TOTAL_CAPACITY}")

        cnt = 0
        collisions = 0

        # 新增：累计占用率
        sum_pct_each = [0.0] * NUM_TABLES
        sum_total_pct = 0.0
        sample_count = 0

        for i, row in enumerate(df.itertuples(index=False), start=1):
            kx = int(getattr(row, "Key_X"))
            ky = int(getattr(row, "Key_Y"))

            res = sim.process(kx, ky)

            if res["table_full"]:
                collisions += 1

            cnt += 1

            # 每处理一次，统计一次当前占用率
            loads, pct = sim.get_bram_loads()

            for t in range(NUM_TABLES):
                sum_pct_each[t] += pct[t]

            total_load = sum(loads)
            total_pct = 100.0 * total_load / TOTAL_CAPACITY
            sum_total_pct += total_pct

            sample_count += 1

            if i % REPORT_INTERVAL == 0:
                avg_pct_each = [x / sample_count for x in sum_pct_each]
                avg_total_pct = sum_total_pct / sample_count
                print(
                    f"Points={i} Timer={sim.global_timer} | "
                    f"Loads={loads} | "
                    f"CurrPct={[round(x,1) for x in pct]}% | "
                    f"AvgPct={[round(x,2) for x in avg_pct_each]}% | "
                    f"AvgTotalPct={avg_total_pct:.2f}% | "
                    f"Collisions={collisions}"
                )

        avg_pct_each = [x / sample_count for x in sum_pct_each] if sample_count else [0.0] * NUM_TABLES
        avg_total_pct = sum_total_pct / sample_count if sample_count else 0.0

        print(f"仿真结束。总点数: {cnt}, 总冲突数: {collisions}, 冲突率: {collisions/cnt*100:.2f}%")
        print(f"各 BRAM 平均占用率: {[round(x, 2) for x in avg_pct_each]}%")
        print(f"总体平均占用率: {avg_total_pct:.2f}%")

    except FileNotFoundError:
        print(f"错误: 找不到文件 {CSV_FILE}。请先生成数据文件。")

