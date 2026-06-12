# 将硬件仿真结果提取成x,y,r_pn的csv文件
import re
import csv
import os

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


def main():
    input_file = r'E:\verilog\pillarnest\modelsim_pre\ssimulation_result_killed.csv'
    output_file = 'output_voxel_hard.csv'
    hw_log_to_csv(input_file, output_file, verbose=True)


if __name__ == "__main__":
    main()