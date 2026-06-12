
"""
分析txt文件中第三列及以后的数的最大绝对值
"""

def analyze_file(file_path):
    """
    分析文件中第三列及以后的数的最大绝对值
    """
    max_abs_value = 0
    max_value_row = None
    max_value_col = None
    total_values = 0
    
    with open(file_path, 'r') as f:
        for row_idx, line in enumerate(f, start=1):
            # 分割行
            values = line.strip().split()
            
            # 从第三列（索引2）及以后的数
            data_values = [float(v) for v in values[2:]]
            total_values += len(data_values)
            
            # 找最大绝对值
            for col_idx, val in enumerate(data_values, start=3):  # 列从第3开始
                abs_val = abs(val)
                if abs_val > max_abs_value:
                    max_abs_value = abs_val
                    max_value_row = row_idx
                    max_value_col = col_idx
    
    return {
        'max_absolute_value': max_abs_value,
        'row': max_value_row,
        'column': max_value_col,
        'total_values_analyzed': total_values
    }


if __name__ == '__main__':
    # 分析文件
    file_path = r'e:\mmdetection3d\bev_48d_nonzero.txt'
    
    result = analyze_file(file_path)
    
    print("=" * 60)
    print("分析结果")
    print("=" * 60)
    print(f"最大绝对值: {result['max_absolute_value']:.6f}")
    print(f"出现位置 - 第 {result['row']} 行, 第 {result['column']} 列")
    print(f"总分析数值个数: {result['total_values_analyzed']}")
    print("=" * 60)
