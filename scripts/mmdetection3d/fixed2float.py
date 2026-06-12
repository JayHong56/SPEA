def q_to_float(value, total_bits, fractional_bits):
    """
    将 Q 格式定点数转换为浮点数。
    
    参数:
        value: 输入值，可以是十六进制字符串 (如 "0xFF80") 或 整数 (如 65408)。
        total_bits: 总位宽 (例如 Q9.7 是 16位)。
        fractional_bits: 小数位宽 (n)。

    返回:
        float: 转换后的浮点数。
    """
    # 1. 如果输入是十六进制字符串，先转为整数
    if isinstance(value, str):
        # 自动识别 '0x' 前缀，或者无前缀的 16 进制
        base = 16 if value.lower().startswith('0x') or any(c in 'abcdef' for c in value.lower()) else 10
        # 这种判断可能不完全准确，建议显式指定。为了通用性，这里假设字符串输入且带0x或者是纯数字字符串
        try:
             value = int(value, 16)
        except ValueError:
             value = int(value)

    # 2. 确保数值在 total_bits 范围内 (截断高位垃圾数据)
    mask = (1 << total_bits) - 1
    value = value & mask

    # 3. 处理符号位 (Two's Complement)
    # 检查最高位 (Sign Bit) 是否为 1
    sign_bit = 1 << (total_bits - 1)
    if value & sign_bit:
        # 如果是负数，进行补码转原码操作
        # 方法：当前无符号值减去 2^total_bits
        value -= (1 << total_bits)

    # 4. 计算浮点数
    # 公式: float = integer / 2^n
    scale_factor = 2 ** fractional_bits
    return value / scale_factor

def float_to_q(value, total_bits, fractional_bits):
    """
    (可选) 将浮点数转换为 Q 格式十六进制字符串。
    """
    scale_factor = 2 ** fractional_bits
    int_val = int(round(value * scale_factor))
    
    # 处理负数补码表示
    if int_val < 0:
        int_val = int_val + (1 << total_bits)
        
    # 截断到指定位宽
    mask = (1 << total_bits) - 1
    int_val = int_val & mask
    
    # 格式化为十六进制，自动补齐位数
    hex_digits = (total_bits + 3) // 4
    return f"0x{int_val:0{hex_digits}x}"

# ==========================================
# 测试用例 (基于你之前的提问)
# ==========================================
if __name__ == "__main__":
    # # 测试 1: 你问过的 -1
    # hex_val1 = "0xFF80"
    # res1 = q_to_float(hex_val1, 16, 7)
    # print(f"Hex: {hex_val1} -> Float: {res1}") 
    # # 预期: -1.0

    # # 测试 2: 你问过的 fd4c
    # hex_val2 = "0xfd4c"
    # res2 = q_to_float(hex_val2, 16, 7)
    # print(f"Hex: {hex_val2} -> Float: {res2}") 
    # # 预期: -5.40625

    # # 测试 3: 正数测试 (例如 1.5 = 1.5 * 128 = 192 = 0x00C0)
    # hex_val3 = "0x00C0"
    # res3 = q_to_float(hex_val3, 16, 7)
    # print(f"Hex: {hex_val3} -> Float: {res3}")
    
    # print("-" * 30)
    # print("反向转换测试 (Float -> Q9.7):")
    print(float_to_q(0.8, 20, 12))
    print(q_to_float("0x0076", 16, 8))  # 应该是 54.0
    print(q_to_float("0x0012e", 20, 12))  # 应该是 54.0

    print(q_to_float("0x1e6ed", 20, 12))  # 应该是 54.0
    print(q_to_float("0x000cc", 20, 12))  # 应该是 54.0

    print(q_to_float("0x1e747", 20, 12))  # 应该是 54.0
    print(q_to_float("0x00191", 20, 12))  # 应该是 54.0

    print(q_to_float("0xfffd3", 20, 12))  # 应该是 54.0
    print(q_to_float("0x0002d", 20, 12))  # 应该是 54.0

    print(q_to_float("0xfe98", 16, 15))  # 应该是 54.0
    print(q_to_float("0x0168", 16, 15))  # 应该是 54.0
