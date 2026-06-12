import os
import numpy as np
import pandas as pd
from PIL import Image

# =========================================================
# 配置参数
# =========================================================
CSV_PATH = r"E:\verilog\pillarnest\modelsim_pointpillars\pfn_layer_out.csv"      # 你的输入 CSV 文件
OUT_DIR = "feature_maps_out_kitti"       # 输出文件夹
H = 496                            # 高
W = 432                            # 宽
C = 64                             # 通道数
FLIP_Y = False                     # 如果想上下翻转图像，改成 True

# 保存图片时的归一化方式:
# "per_channel" : 每个通道单独归一化到 [0,255]
# "global"      : 所有通道使用同一个全局最小最大值归一化
NORM_MODE = "per_channel"


def normalize_to_uint8(img: np.ndarray, vmin=None, vmax=None):
    """
    将二维浮点图归一化到 uint8 [0,255]
    """
    img = img.astype(np.float32)

    if vmin is None:
        vmin = np.min(img)
    if vmax is None:
        vmax = np.max(img)

    if vmax - vmin < 1e-12:
        return np.zeros_like(img, dtype=np.uint8)

    img_norm = (img - vmin) / (vmax - vmin)
    img_uint8 = (img_norm * 255.0).clip(0, 255).astype(np.uint8)
    return img_uint8


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # =====================================================
    # 1. 读取 CSV
    # =====================================================
    df = pd.read_csv(CSV_PATH)

    # 检查列是否存在
    required_cols = ["voxel_x", "voxel_y"] + [f"dim{i}" for i in range(C)]
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"CSV 中缺少必要列: {col}")

    # =====================================================
    # 2. 初始化特征图张量 [C, H, W]
    # =====================================================
    feature_maps = np.zeros((C, H, W), dtype=np.float32)

    # =====================================================
    # 3. 填充特征图
    #    默认规则：
    #    feature_maps[c, voxel_y, voxel_x] = dimc
    # =====================================================
    invalid_cnt = 0

    for idx, row in df.iterrows():
        x = int(row["voxel_x"])
        y = int(row["voxel_y"])

        # 检查坐标合法性
        if not (0 <= x < W and 0 <= y < H):
            invalid_cnt += 1
            continue

        yy = H - 1 - y if FLIP_Y else y

        for c in range(C):
            feature_maps[c, yy, x] = float(row[f"dim{c}"])

    # =====================================================
    # 4. 保存原始特征张量
    # =====================================================
    npy_path = os.path.join(OUT_DIR, "feature_maps.npy")
    np.save(npy_path, feature_maps)
    print(f"[INFO] 已保存原始特征张量: {npy_path}")
    print(f"[INFO] 特征张量形状: {feature_maps.shape}")  # (64, 496, 432)

    if invalid_cnt > 0:
        print(f"[WARN] 有 {invalid_cnt} 行坐标越界，已跳过。")

    # =====================================================
    # 5. 保存 64 张特征图
    # =====================================================
    if NORM_MODE == "global":
        global_min = np.min(feature_maps)
        global_max = np.max(feature_maps)
        print(f"[INFO] 全局归一化范围: min={global_min:.6f}, max={global_max:.6f}")
    else:
        global_min, global_max = None, None

    for c in range(C):
        img = feature_maps[c]

        if NORM_MODE == "per_channel":
            img_uint8 = normalize_to_uint8(img)
        elif NORM_MODE == "global":
            img_uint8 = normalize_to_uint8(img, global_min, global_max)
        else:
            raise ValueError("NORM_MODE 只能是 'per_channel' 或 'global'")

        out_path = os.path.join(OUT_DIR, f"dim{c:02d}.png")
        Image.fromarray(img_uint8, mode="L").save(out_path)

    print(f"[INFO] 已保存 64 张特征图到: {OUT_DIR}")


if __name__ == "__main__":
    main()