# 比较硬件csv备份结果和最新结果的差异
import csv

file1 = "E:\\verilog\\pillarnest\\modelsim_pointpillars2\\ssimulation_result.csv"
file2 = "E:\\verilog\\pillarnest\\modelsim_pointpillars\\ssimulation_result.csv"

def normalize_row(row):
    return [cell.strip() for cell in row]

with open(file1, "r", encoding="utf-8-sig", newline="") as f1, \
     open(file2, "r", encoding="utf-8-sig", newline="") as f2:

    reader1 = [normalize_row(row) for row in csv.reader(f1)]
    reader2 = [normalize_row(row) for row in csv.reader(f2)]

len1 = len(reader1)
len2 = len(reader2)

if len1 != len2:
    print(f"行数不同: {file1} 有 {len1} 行, {file2} 有 {len2} 行")

min_len = min(len1, len2)
all_equal = True

for i in range(min_len):
    if reader1[i] != reader2[i]:
        all_equal = False
        print(f"\n第 {i+1} 行不同:")
        print(f"{file1}: {reader1[i]}")
        print(f"{file2}: {reader2[i]}")

if len1 > min_len:
    all_equal = False
    print(f"\n{file1} 多出的行:")
    for i in range(min_len, len1):
        print(f"第 {i+1} 行: {reader1[i]}")

if len2 > min_len:
    all_equal = False
    print(f"\n{file2} 多出的行:")
    for i in range(min_len, len2):
        print(f"第 {i+1} 行: {reader2[i]}")

if all_equal:
    print("两个 CSV 每一行都相等（已忽略单元格首尾空格）。")
else:
    print("\n比较完成：两个 CSV 存在差异。")
