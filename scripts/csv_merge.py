# 合并csv文件位于最前端和最后端属于同一Voxel的点

import pandas as pd

# 1. 读取原始 CSV 文件
# 假设列名是 'x', 'y', 'value'
df = pd.read_csv('output_voxel_hard.csv')

# 2. 按 'x' 和 'y' 分组，并将同组的 'value' 相加
# as_index=False 的作用是保持 x, y 作为普通的列，而不是变成行索引
merged_df = df.groupby(['x', 'y'], as_index=False)['r_pn'].sum()

# 2.5 将 r_pn 大于 20 的值限制为 20
merged_df['r_pn'] = merged_df['r_pn'].clip(upper=20)
# 3. 查看合并后的结果
print(merged_df.head())

# 4. 保存为新的 CSV 文件
merged_df.to_csv('merged_data_hard.csv', index=False)
print("合并完成！已保存为 merged_data_hard.csv")

##########################################################################################
# 1. 读取原始 CSV 文件
# 假设列名是 'x', 'y', 'value'
df2 = pd.read_csv('output_voxel_simulation.csv')

# 2. 按 'x' 和 'y' 分组，并将同组的 'value' 相加
# as_index=False 的作用是保持 x, y 作为普通的列，而不是变成行索引
merged_df_2 = df2.groupby(['x', 'y'], as_index=False)['value'].sum()

# 2.5 将value大于20的值限制为20
merged_df_2['value'] = merged_df_2['value'].clip(upper=20)

# 3. 查看合并后的结果
print(merged_df_2.head())

# 4. 保存为新的 CSV 文件
merged_df_2.to_csv('merged_data_sim.csv', index=False)
print("合并完成！已保存为 merged_data_sim.csv")