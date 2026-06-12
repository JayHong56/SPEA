import csv

# ================= 配置文件路径 =================
txt_file_path = 'bev_48d_nonzero.txt'     # 替换为你的输入 txt 文件路径
csv_file_path = 'output_voxel_simulation.csv'     # 替换为你的输入 csv 文件路径
output_txt_path = 'output.txt'  # 替换为处理后的输出文件路径
# ===============================================

def process_files():
    value_map = {}
    
    # 使用 'utf-8-sig' 编码，自动去除隐藏的 BOM 字符
    with open(csv_file_path, mode='r', encoding='utf-8-sig') as csv_file:
        reader = csv.DictReader(csv_file)
        
        # 清理表头可能带有的前后空格
        if reader.fieldnames:
            reader.fieldnames = [field.strip() for field in reader.fieldnames]
            
        for row in reader:
            if not row or 'x' not in row or row['x'] is None:
                continue
                
            x = int(row['x'])
            y = int(row['y'])
            value_map[(x, y)] = row['value']

    with open(txt_file_path, mode='r', encoding='utf-8') as txt_file, \
         open(output_txt_path, mode='w', encoding='utf-8') as out_file:
        
        for line in txt_file:
            line = line.strip()
            if not line:
                out_file.write('\n')
                continue
                
            parts = line.split()
            
            if len(parts) >= 2:
                # 提取 txt 中的坐标并转为整数
                x_txt = int(float(parts[0]))
                y_txt = int(float(parts[1]))
                
                # 更新前两列，去除小数点和零（例如将 '301.000000' 变为 '301'）
                parts[0] = str(x_txt)
                parts[1] = str(y_txt)
                
                # 获取匹配的 value，插入到第三列
                matched_value = value_map.get((x_txt, y_txt), '0')
                parts.insert(2, str(matched_value))
                
                # 拼接并写入文件
                out_file.write(' '.join(parts) + '\n')
            else:
                out_file.write(line + '\n')

    print(f"处理完成！结果已保存至: {output_txt_path}")

if __name__ == '__main__':
    process_files()