# 比较硬件和软件的哈希输出结果一致性

import pandas as pd
import sys
import io
import os

# 解决输出编码问题
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def load_data_robust(file_path):
    if not os.path.exists(file_path):
        print(f"错误：找不到文件 {file_path}")
        return None
    
    data = []
    # 尝试多种编码方案
    for enc in ['gbk', 'utf-8', 'utf-16']:
        try:
            with open(file_path, 'r', encoding=enc) as f:
                for line in f:
                    clean_line = line.replace(',', ' ').strip()
                    if not clean_line: continue
                    parts = clean_line.split()
                    # 关键：只取前3列，防止列数过多报错
                    if len(parts) >= 3:
                        try:
                            # 兼容 154.0 这种浮点格式
                            data.append([int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))])
                        except ValueError: continue
            break # 成功读取则跳出编码尝试
        except: continue
            
    if not data:
        print(f"错误：无法从 {file_path} 提取有效数据。")
        return None
    return pd.DataFrame(data, columns=['x', 'y', 'val'])

def convert_csv_to_txt_and_load(file_path):
    """
    拦截器：如果输入是 csv，先转成以空格分隔的 txt，再进行处理。
    如果是 txt，直接处理。
    """
    if file_path.lower().endswith('.csv'):
        txt_path = file_path[:-4] + '.txt'
        print(f"🔄 检测到 CSV 文件，正在生成 TXT: {txt_path}")
        try:
            # 读取 CSV 并存为以空格分隔的 TXT 文件，不保留索引
            df_temp = pd.read_csv(file_path)
            df_temp.to_csv(txt_path, sep=' ', index=False)
            file_path = txt_path  # 更新文件路径，供后续加载
        except Exception as e:
            print(f"❌ CSV 转换 TXT 失败: {e}")
            return None
    
    # 调用原有的 robust 函数加载数据
    return load_data_robust(file_path)


# =====================================================================
# 主逻辑
# =====================================================================
# 假设 df1 是软件输出的 TXT
df1_path = r'E:\verilog\pillarnest\python\output_sim_bitlevel\output_simulation_hard_voxel.txt'
# 假设 df2 是硬件输出的 CSV (将其传入我们的新函数即可自动转换)
df2_path = r'E:\verilog\pillarnest\modelsim_pre\ssimulation_result.csv'

df1 = convert_csv_to_txt_and_load(df1_path)
df2 = convert_csv_to_txt_and_load(df2_path)

if df1 is not None and df2 is not None:
    # 核心操作：重置索引并横向合并
    # axis=1 表示左右拼接，不需要找共同的 x,y，只看行号
    df1 = df1.reset_index(drop=True)
    df2 = df2.reset_index(drop=True)
    
    # 将两张表并列排放
    result = pd.concat([df1, df2], axis=1)
    
    # 重新命名列名以区分
    result.columns = ['x1', 'y1', 'val1', 'x2', 'y2', 'val2']

    # 打印表头
    print(f"\n{'Row':<6} {'(x1,y1)':<15} {'(x2,y2)':<15} {'Val1':<8} {'Val2':<8} {'Status'}")
    print("-" * 85)

    diff_count = 0
    # 遍历每一行进行对比
    for idx, row in result.iterrows():
        # 同时检查坐标和数值是否都一致
        coords_match = (row['x1'] == row['x2']) and (row['y1'] == row['y2'])
        val_match = (row['val1'] == row['val2'])
        
        if coords_match and val_match:
            status = "OK"
        elif not coords_match:
            status = f"COORD_ERR({row['x1']},{row['y1']} vs {row['x2']},{row['y2']})"
            diff_count += 1
        else:
            status = f"VAL_ERR({row['val1']-row['val2']})"
            diff_count += 1
            
        # 打印前500行或只打印错误行，避免输出文件过大
        if diff_count < 100 or not val_match or not coords_match:
            print(f"{idx:<6} ({row['x1']},{row['y1']}){'':<6}({row['x2']},{row['y2']}){'':<6}{row['val1']:<8} {row['val2']:<8} {status}")

    print("-" * 85)
    print(f"总行数: {len(result)} | 错误行数: {diff_count}")