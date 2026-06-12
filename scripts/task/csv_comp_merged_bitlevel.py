# 比较硬件和软件仿真得到的voxel中的点个数是否一致
import pandas as pd

# 1. 读取两个 CSV 文件
df_hw = pd.read_csv(r'E:\verilog\pillarnest\python\output_hard\merged_data_hard.csv') 
df_pt = pd.read_csv(r'E:\verilog\pillarnest\python\output_sim_bitlevel\merged_data_sim_bitlevel.csv') 

# 将数值列重命名，方便对比
df_hw = df_hw.rename(columns={'r_pn': 'hard_result'})
df_pt = df_pt.rename(columns={'value': 'soft_result'})

# 2. 根据 'x' 和 'y' 列进行合并 (Inner Join)
merged_df = pd.merge(df_hw, df_pt, on=['x', 'y'], how='inner')

# 3. 比较差异
# ==============================================================
# 修改点：将 diff 变为 True/False 
# True 代表有差异（两者不等），False 代表无差异（两者相等）
# ==============================================================
merged_df['diff'] = merged_df['hard_result'] != merged_df['soft_result']

# 制定我们要重点打印的列
display_columns = ['x', 'y', 'soft_result', 'hard_result', 'diff']

# 解除 Pandas 的打印行数限制，强制打印所有数据
pd.set_option('display.max_rows', None)

print("========== 完整比对结果明细 (软仿 vs 硬仿) ==========")
# 直接打印所有匹配的点
print(merged_df[display_columns]) 

# 恢复默认打印限制
pd.reset_option('display.max_rows')

# 4. 统计结果
# 因为 diff 是 bool 类型，True 的数量就是不一致的点数，可以直接 sum()
diff_count = merged_df['diff'].sum()

print(f"\n========================================================")
print(f"统计摘要：")
print(f"共有 {len(merged_df)} 个匹配的 Voxel。")
print(f"其中有 {diff_count} 个点的值不一致 (diff 为 True)。")
print(f"========================================================\n")

# 5. 保存结果
if diff_count > 0:
    # 提取所有 diff 为 True 的行
    diff_df = merged_df[merged_df['diff'] == True][display_columns]
    diff_df.to_csv('diff_report.csv', index=False)
    print("发现不一致！已将出错的详细数据单独提取并保存到 'diff_report.csv'")
else:
    print("恭喜！所有匹配坐标的值完全一致！")

# 6. 保存完整的对比表
merged_df[display_columns].to_csv('comparison_full.csv', index=False)