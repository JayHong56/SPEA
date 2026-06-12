import os
import shutil

# 原仓库路径
MMDET3D_ROOT = r'C:\Users\86170\mmdetection3d'  # ← 这里填原始 mmdet3d 路径

# 目标新工程路径
EXPORT_ROOT = r'C:\Users\86170\mmdetection3d-new'  # ← 这里填新工程路径

# 读取生成的文件列表
NEEDED_LIST = os.path.join(MMDET3D_ROOT, 'needed_files_mmdet3d_init_model.txt')

def main():
    # 读取所需的文件路径
    with open(NEEDED_LIST, 'r', encoding='utf-8') as fp:
        paths = [line.strip() for line in fp if line.strip()]

    for src in paths:
        # 保留相对结构，从 MMDET3D_ROOT 开始计算相对路径
        rel = os.path.relpath(src, MMDET3D_ROOT)
        dst = os.path.join(EXPORT_ROOT, rel)

        # 确保目标目录存在
        os.makedirs(os.path.dirname(dst), exist_ok=True)

        # 拷贝文件
        shutil.copy2(src, dst)
        print(f'复制: {src} -> {dst}')

    print(f'完成，共复制 {len(paths)} 个文件到 {EXPORT_ROOT}')

if __name__ == '__main__':
    main()
