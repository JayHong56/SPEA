import os
import sys

import csv
import re
from typing import Iterable, List, Optional, Sequence, Tuple
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
import re

def hw_log_to_csv(input_txt_path, output_csv_path, verbose=True):
    # 检查输入文件是否存在
    if not os.path.exists(input_txt_path):
        raise FileNotFoundError(f"找不到输入文件: {input_txt_path}")

    # 1. 编译正则表达式，精准匹配 r_pn, x, y 后面的数值
    pattern = re.compile(r'r_pn:\s*(\d+),\s*x:\s*(\d+),\s*y:\s*(\d+)')
    
    extracted_data = []

    # 2. 逐行读取 txt 文件并解析
    with open(input_txt_path, 'r', encoding='utf-8') as infile:
        for line in infile:
            match = pattern.search(line)
            if match:
                r_pn = int(match.group(1))
                x = int(match.group(2))
                y = int(match.group(3))
                # 按照要求的列顺序 [x, y, r_pn] 添加到列表
                extracted_data.append([x, y, r_pn])

    if not extracted_data:
        if verbose:
            print("未在文件中匹配到有效数据！请确认日志内容格式。")
        return 0

    # 3. 将提取的数据写入 CSV 文件
    with open(output_csv_path, 'w', newline='', encoding='utf-8') as outfile:
        writer = csv.writer(outfile)
        
        # 写入表头 (列名)
        writer.writerow(['x', 'y', 'r_pn'])
        
        # 批量写入所有数据行
        writer.writerows(extracted_data)

    if verbose:
        print(f"处理完成！共提取了 {len(extracted_data)} 条数据。")
        print(f"数据已成功保存至: {output_csv_path}")

    return len(extracted_data)


def merge_voxel_csv(input_csv: str, output_csv: str = "merged_data_sim.csv", clip_upper: int = 20):
    """按 x/y 聚合 r_pn 并裁剪上限，最后输出新 CSV。"""
    df2 = pd.read_csv(input_csv)
    merged_df_2 = df2.groupby(['x', 'y'], as_index=False)['r_pn'].sum()
    merged_df_2['r_pn'] = merged_df_2['r_pn'].clip(upper=clip_upper)

    print(merged_df_2.head())
    merged_df_2.to_csv(output_csv, index=False)
    print(f"合并完成！已保存为 {output_csv}")

    return merged_df_2


if __name__ == "__main__":
    input_file = r'E:\verilog\pillarnest\modelsim_pre\ssimulation_result_killed.csv'
    output_file = r'E:\verilog\pillarnest\python\output_hard\output_simulation_hard_killed.csv'
    hw_log_to_csv(input_file, output_file, verbose=True)

    merged_csv_path = r"E:\verilog\pillarnest\python\output_hard\merged_data_hard.csv"

    merge_voxel_csv(
        input_csv=output_file,
        output_csv=merged_csv_path,
        clip_upper=20,
    )