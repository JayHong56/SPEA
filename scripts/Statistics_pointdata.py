import pandas as pd
import os

def analyze_point_data(file_path):
    if not os.path.exists(file_path):
        print(f"错误: 找不到文件 {file_path}")
        return

    # 读取文件，按空格或制表符分割，无表头
    try:
        df = pd.read_csv(file_path, sep='\s+', header=None)
        
        # 自动给列命名 (例如 Col_0, Col_1...)
        # 第四列对应的将是 Col_3
        df.columns = [f"Col_{i}" for i in range(df.shape[1])]
        
        print(f"========== {os.path.basename(file_path)} 统计结果 ==========")
        print(f"总行数: {len(df)}, 总列数: {df.shape[1]}\n")
        
        # 打印每一列的范围和均值
        for col in df.columns:
            col_min = df[col].min()
            col_max = df[col].max()
            col_mean = df[col].mean()
            print(f"{col:<6} | 范围: [{col_min:10.6f}, {col_max:10.6f}] | 均值: {col_mean:10.6f}")
        
        print("-" * 65)
        
        # 专项统计：第四列 (Col_3) 大于 127 的点数
        if df.shape[1] >= 4:
            intensity_col = 'Col_3'
            # (df[intensity_col] > 127) 会返回一个 True/False 的序列，.sum() 会将 True 视为 1 进行累加
            high_intensity_count = (df[intensity_col] > 127).sum()
            high_intensity_pct = (high_intensity_count / len(df)) * 100
            
            print(f"🎯 Intensity 统计: 第四列 ({intensity_col}) 大于 127 的点共有 {high_intensity_count} 个 (占总数的 {high_intensity_pct:.2f}%)")
        else:
            print("⚠️ 数据不足 4 列，无法进行 Intensity 专项统计。")
            
        print("==========================================================")
            
    except Exception as e:
        print(f"解析文件出错: {e}")

# 替换成你实际的文件路径进行测试
analyze_point_data(r"E:\verilog\pillarnest\rtl\.pointcloud\points_float32.txt")