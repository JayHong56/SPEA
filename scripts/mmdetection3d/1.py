import numpy as np
import glob
import os


def read_and_interleave_clouds(folder_path, part_size=32, target_frames=10, output_dim=4):
    """
    读取多个 bin 点云文件，并按 part/frame 交织。

    原始 bin 中每个维度都是 float32。
    默认最终只保留前 4 维：x, y, z, intensity。
    """

    file_list = sorted(glob.glob(os.path.join(folder_path, "*.bin")))

    if len(file_list) < target_frames:
        print(f"警告：文件夹内只有 {len(file_list)} 个文件，无法满足 {target_frames} 帧的需求！")
        target_frames = len(file_list)

    file_list = file_list[:target_frames]
    all_frames = []

    for f in file_list:
        raw_data = np.fromfile(f, dtype=np.float32)

        # 自动判断原始 bin 是 4 维还是 5 维
        if raw_data.size % 5 == 0:
            dim = 5
        elif raw_data.size % 4 == 0:
            dim = 4
        else:
            print(f"文件 {os.path.basename(f)} 数据大小 {raw_data.size} 无法被 4 或 5 整除，跳过。")
            continue

        points = raw_data.reshape(-1, dim)

        if points.shape[1] < output_dim:
            print(f"文件 {os.path.basename(f)} 只有 {points.shape[1]} 维，无法输出 {output_dim} 维，跳过。")
            continue

        # 统一只保留前 output_dim 维
        points = points[:, :output_dim].astype(np.float32)

        all_frames.append(points)

    if not all_frames:
        print("错误：没有成功读取任何点云。")
        return None

    # 取所有帧中的最小点数，并对齐到 part_size 的整数倍
    min_pts = min(frame.shape[0] for frame in all_frames)
    num_parts = min_pts // part_size
    final_pts_per_frame = num_parts * part_size

    if final_pts_per_frame == 0:
        print("错误：点数不足一个 part。")
        return None

    trimmed_frames = [frame[:final_pts_per_frame, :] for frame in all_frames]

    # shape: [Frames, Points, Dim]
    stack = np.stack(trimmed_frames, axis=0)

    # shape: [Frames, NumParts, PartSize, Dim]
    stack = stack.reshape(target_frames, num_parts, part_size, output_dim)

    # interleave:
    # [Frame, Part, Point, Dim] -> [Part, Frame, Point, Dim]
    interleaved = stack.transpose(1, 0, 2, 3)

    # 展平为 [N, Dim]
    result = interleaved.reshape(-1, output_dim).astype(np.float32)

    print("-" * 30)
    print("诊断报告:")
    print(f"1. 处理帧数: {target_frames}")
    print(f"2. 输出维度: {output_dim}")
    print(f"3. 每帧 Part 数量: {num_parts}")
    print(f"4. 每帧最终点数: {final_pts_per_frame}")
    print(f"5. 最终输出形状: {result.shape}")
    print(f"6. 数据类型: {result.dtype}")
    print("-" * 30)

    return result


def save_point_cloud_txt_for_fpga(interleaved_data, base_filename):
    """
    将点云保存为 hex txt、binary txt 和 decimal txt。

    每个维度都是 float32。
    每行一个点。
    """

    if interleaved_data is None:
        print("错误：无数据可保存。")
        return

    # 保证每维都是 float32
    final_data = interleaved_data.astype(np.float32)

    # IEEE-754 float32 位模式
    bit_view = final_data.view(np.uint32)

    hex_txt_path = base_filename + "_hex.txt"
    bin_txt_path = base_filename + "_bin.txt"
    dec_txt_path = base_filename + "_decimal.txt"

    # --------------------------------------------------
    # 1. 保存 Hex TXT
    # 每个 float32 显示为 8 个 hex 字符
    # 例如一行：x y z i -> 32 个 hex 字符
    # --------------------------------------------------
    with open(hex_txt_path, "w") as f_hex:
        for point in bit_view:
            hex_line = "".join(f"{val:08x}" for val in point)
            f_hex.write(hex_line + "\n")

    # --------------------------------------------------
    # 2. 保存 Binary TXT
    # 每个 float32 显示为 32 个 0/1 字符
    # --------------------------------------------------
    with open(bin_txt_path, "w") as f_bin:
        for point in bit_view:
            bin_line = "".join(f"{val:032b}" for val in point)
            f_bin.write(bin_line + "\n")

    # --------------------------------------------------
    # 3. 保存 Decimal TXT
    # 每行一个点，显示为十进制浮点数
    # 格式：x y z intensity
    # --------------------------------------------------
    with open(dec_txt_path, "w") as f_dec:
        for point in final_data:
            dec_line = " ".join(f"{val:.8f}" for val in point)
            f_dec.write(dec_line + "\n")

    print("-" * 30)
    print("TXT 导出完成：")
    print(f"Hex 文本: {hex_txt_path}")
    print(f"Bin 文本: {bin_txt_path}")
    print(f"Decimal 文本: {dec_txt_path}")
    print(f"点数: {final_data.shape[0]}")
    print(f"每点维度: {final_data.shape[1]}")
    print(f"每维格式: float32 / IEEE-754 / 32-bit")
    print("-" * 30)


def save_interleaved_to_bin(interleaved_data, output_path):
    """
    保存为真正的二进制 bin 文件。

    bin 文件内容是连续的 float32：

        x0 y0 z0 i0 x1 y1 z1 i1 x2 y2 z2 i2 ...

    每个维度 4 字节，也就是 32 bit float。
    """

    if interleaved_data is None:
        print("错误：没有可保存的数据。")
        return

    # 强制保存为 float32
    final_data = interleaved_data.astype(np.float32)

    # 直接保存 float32 原始字节
    final_data.tofile(output_path)

    print("-" * 30)
    print(f"二进制文件已生成: {output_path}")
    print(f"最终形状: {final_data.shape}")
    print(f"数据类型: {final_data.dtype}")
    print(f"每维大小: 32 bit float")
    print(f"文件大小: {os.path.getsize(output_path) / 1024:.2f} KB")
    print("-" * 30)


# ==========================================================
# 主程序
# ==========================================================
data_folder = r"E:\mmdetection3d\data\kitti\scripts\data"

output_bin = r"E:\mmdetection3d\data\kitti\scripts\data\interleaved_pts_4d.bin"
output_txt_base = r"E:\mmdetection3d\data\kitti\scripts\data\points_sim"

# output_dim=4 表示只保存 x, y, z, intensity
# 如果你想保留 5 维，可以改成 output_dim=5
interleaved_result = read_and_interleave_clouds(
    folder_path=data_folder,
    part_size=32,
    target_frames=1,
    output_dim=4
)

save_point_cloud_txt_for_fpga(interleaved_result, output_txt_base)

save_interleaved_to_bin(interleaved_result, output_bin)