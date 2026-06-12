# 比较硬件和软件仿真得到的voxel中的点个数是否一致
import pandas as pd

# 1. 读取两个 CSV 文件
# # 假设硬件数据的列名是 x, y, r_pn
# df_hw = pd.read_csv('merged_data_hard.csv') 

# # 假设 PyTorch 数据的列名是 x, y, value
# df_pt = pd.read_csv('merged_data_sim.csv') 


# 假设硬件数据的列名是 x, y, r_pn
df_hw = pd.read_csv(r'E:\verilog\pillarnest\python\output_hard\merged_data_hard.csv') 

# 假设 PyTorch 数据的列名是 x, y, value
df_pt = pd.read_csv(r'E:\verilog\pillarnest\python\output_sim\merged_data_sim.csv') 

# 为了方便区分，我们在合并前统一一下目标值的列名，或者直接合并
# 这里我们假设 df_hw 的值列叫 'r_pn'，df_pt 的值列叫 'value'

# 2. 根据 'x' 和 'y' 列进行合并 (Inner Join)
# how='inner' 表示只保留在两个表中都存在的 x, y 坐标
# 如果列名相同（比如都叫 value），可以用 suffixes 参数自动加后缀，如 suffixes=('_hw', '_pt')
merged_df = pd.merge(df_hw, df_pt, on=['x', 'y'], how='inner')

# 3. 比较差异
# 增加一列表明它们差值 (例如: 硬件值 - 软件值)
merged_df['diff'] = merged_df['r_pn'] - merged_df['value']

# 增加一列表明它们是否完全相等 (True/False)
merged_df['is_equal'] = merged_df['r_pn'] == merged_df['value']

# 4. 查看结果
print("========== 比较结果预览 ==========")
print(merged_df.head(10)) # 打印前 10 行看看

# 统计有多少个坐标点的值是不一致的
diff_count = len(merged_df[merged_df['is_equal'] == False])
print(f"\n共有 {len(merged_df)} 个匹配的Voxel。")
print(f"其中有 {diff_count} 个点的值不一致。")

# 5. (可选) 找出不一致的数据并保存为新的 CSV
if diff_count > 0:
    diff_df = merged_df[merged_df['is_equal'] == False]
    diff_df.to_csv('diff_report.csv', index=False)
    print("已将不一致的数据保存到 'diff_report.csv'")
else:
    print("恭喜！所有匹配坐标的值完全一致！")

# 6. (可选) 保存完整的对比表
merged_df.to_csv('comparison_full.csv', index=False)