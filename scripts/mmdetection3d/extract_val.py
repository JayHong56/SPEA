
import os
import shutil

# ================= 配置区 =================
# val.txt 的路径
val_txt_path = r'E:\mmdetection3d\data\kitti\ImageSets\val.txt'

# 原始 velodyne 文件夹路径（包含所有 .bin 文件）
source_dir = r'E:\mmdetection3d\data\kitti\training\velodyne'

# 提取出的 val 数据存放的新文件夹路径
target_dir = r'E:\mmdetection3d\data\kitti\training\velodyne_val'

# 文件后缀名（KITTI 点云默认为 .bin）
file_extension = '.bin'
# ==========================================

def extract_val_files():
    # 1. 检查 val.txt 和原始文件夹是否存在
    if not os.path.exists(val_txt_path):
        print(f"错误: 找不到列表文件 {val_txt_path}")
        return
    if not os.path.exists(source_dir):
        print(f"错误: 找不到源文件夹 {source_dir}")
        return

    # 2. 如果目标文件夹不存在，则自动创建
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f"已创建目标文件夹: {target_dir}")

    # 3. 读取 val.txt 中的文件编号
    with open(val_txt_path, 'r') as f:
        # 去除每行首尾的空白字符（包括换行符）
        frame_ids = [line.strip() for line in f.readlines() if line.strip()]

    total_files = len(frame_ids)
    print(f"在 val.txt 中共找到 {total_files} 个文件编号。开始提取...\n")

    # 4. 遍历编号，执行复制操作
    success_count = 0
    missing_files = []

    for idx, frame_id in enumerate(frame_ids):
        # 拼接源文件和目标文件的完整路径
        file_name = f"{frame_id}{file_extension}"
        src_file = os.path.join(source_dir, file_name)
        dst_file = os.path.join(target_dir, file_name)

        if os.path.exists(src_file):
            shutil.copy2(src_file, dst_file) # copy2 会保留文件的原始元数据（如修改时间）
            success_count += 1
            # 每处理 500 个文件打印一次进度
            if success_count % 500 == 0:
                print(f"进度: {success_count} / {total_files} 已复制...")
        else:
            missing_files.append(file_name)

    # 5. 打印最终结果报告
    print("\n================ 提取完成 ================")
    print(f"成功复制: {success_count} 个文件。")
    print(f"目标路径: {os.path.abspath(target_dir)}")
    
    if missing_files:
        print(f"警告: 有 {len(missing_files)} 个文件在源文件夹中未找到！")
        # 打印前 5 个丢失的文件作为示例
        print("未找到的文件示例:", missing_files[:5])

if __name__ == "__main__":
    extract_val_files()