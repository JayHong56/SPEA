def parse_hash_entry(hex_str):
    # 移除可能带有 Verilog 格式的前缀，如 "45'h" 或 "0x"
    if "'h" in hex_str:
        hex_str = hex_str.split("'h")[1]
    elif hex_str.startswith("0x"):
        hex_str = hex_str[2:]
        
    val = int(hex_str, 16)
    
    # 从 LSB 到 MSB 依次截取各个字段 (位操作)
    # 1. Timestamp (16 bits)
    ts = val & ((1 << 16) - 1)
    val >>= 16
    
    # 2. Point Number (5 bits)
    pn = val & ((1 << 5) - 1)
    val >>= 5
    
    # 3. Key Y (11 bits)
    ky = val & ((1 << 11) - 1)
    val >>= 11
    
    # 4. Key X (11 bits)
    kx = val & ((1 << 11) - 1)
    val >>= 11
    
    # 5. Status (2 bits)
    st = val & ((1 << 2) - 1)
    
    # ==========================================
    # 处理补码 (Two's Complement) 转换为有符号数
    # 11 bit 的符号位是第 10 位 (1 << 10)
    # ==========================================
    if kx & (1 << 10):
        kx -= (1 << 11)
        
    if ky & (1 << 10):
        ky -= (1 << 11)
        
    # 映射状态枚举
    st_map = {0: "ST_EMPTY", 1: "ST_OCCU", 2: "ST_TOMB", 3: "INVALID"}
    st_str = st_map.get(st, "UNKNOWN")
    
    return {
        "Status": f"{st} ({st_str})",
        "Key_X": kx,
        "Key_Y": ky,
        "Point_Num": pn,
        "Timestamp": ts
    }

# 测试你的数据
test_val = "45'h094c2d010225"
result = parse_hash_entry(test_val)

print(f"解析 {test_val} :")
print("-" * 30)
for k, v in result.items():
    print(f"{k:<12}: {v}")