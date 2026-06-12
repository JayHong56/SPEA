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
    # 1. 字符串输入统一按十六进制解析
    if isinstance(value, str):
        value = value.strip().lower()
        if value.startswith("0x"):
            value = value[2:]
        if value == "":
            raise ValueError("empty input")
        value = int(value, 16)

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

if __name__ == "__main__":
    print("输入 16 位十六进制（可带或不带 0x），按 Q8.8 转十进制浮点。")
    print("示例: 输入 1234 -> 按 0x1234 解析。输入 q 退出。")

    while True:
        raw = input("hex> ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            print("bye")
            break

        try:
            result = q_to_float(raw, total_bits=16, fractional_bits=8)
            print(f"0x{int(raw, 16):04X} (Q8.8) -> {result}")
        except ValueError:
            print("输入无效，请输入十六进制，例如 1234 或 0x1234")