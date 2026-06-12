# with open("points_sim_bin.txt", "r") as fin, open("INPUT.bin", "wb") as fout:
#     for line in fin:
#         line = line.strip()
#         if len(line) != 128:
#             raise ValueError("某一行不是128 bit")

#         byte_array = bytearray()
#         for i in reversed(range(0, 128, 8)):  
#             byte = int(line[i:i+8], 2)
#             byte_array.append(byte)

#         fout.write(byte_array)


# with open("points_sim_bin.txt", "r") as fin, open("output_hex_le.txt", "w") as fout:
#     for line in fin:
#         line = line.strip()
#         if len(line) != 128:
#             raise ValueError("某一行不是128 bit")

#         # 1. 转成 byte（小端：反转顺序）
#         bytes_list = [
#             int(line[i:i+8], 2)
#             for i in reversed(range(0, 128, 8))
#         ]

#         # 2. 转成 hex 字符串
#         hex_str = ''.join(f'{b:02x}' for b in bytes_list)

#         # 3. 写入 txt
#         fout.write(hex_str + '\n')


with open("points_sim_bin.txt", "r") as fin, open("INPUT.bin", "wb") as fout:
    for line in fin:
        line = line.strip()
        if len(line) != 128:
            raise ValueError("某一行不是128 bit")

        byte_array = bytearray()
        # 删除了 reversed()，按正序处理 0-128
        for i in range(0, 128, 8):  
            byte = int(line[i:i+8], 2)
            byte_array.append(byte)

        fout.write(byte_array)


with open("points_sim_bin.txt", "r") as fin, open("output_hex_le.txt", "w") as fout:
    for line in fin:
        line = line.strip()
        if len(line) != 128:
            raise ValueError("某一行不是128 bit")

        # 1. 转成 byte（正序）
        bytes_list = [
            int(line[i:i+8], 2)
            # 删除了 reversed()，按正序处理 0-128
            for i in range(0, 128, 8)
        ]

        # 2. 转成 hex 字符串
        hex_str = ''.join(f'{b:02x}' for b in bytes_list)

        # 3. 写入 txt
        fout.write(hex_str + '\n')