import numpy as np
import pandas as pd
import os

def evaluate_pseudo_image(sw_pseudo_img: np.ndarray, hw_pseudo_img: np.ndarray):
    """
    评估软硬件伪图 (Pseudo-image) 误差
    """
    assert sw_pseudo_img.shape == hw_pseudo_img.shape, \
        f"❌ Shape 不一致! SW: {sw_pseudo_img.shape}, HW: {hw_pseudo_img.shape}"
    
    print("="*60)
    print("🎯 Pseudo-image 软硬一致性对齐报告")
    print("="*60)

    # 指标 5: 空间掩码对齐度 (Mask IoU)
    sw_mask = np.max(np.abs(sw_pseudo_img), axis=0) > 1e-5
    hw_mask = np.max(np.abs(hw_pseudo_img), axis=0) > 1e-5
    
    intersection = np.logical_and(sw_mask, hw_mask).sum()
    union = np.logical_or(sw_mask, hw_mask).sum()
    iou = intersection / union if union > 0 else 1.0
    
    print(f"📍 空间投影 IoU (Mask IoU) : {iou:.6f} ", end="")
    if iou == 1.0:
        print("✅ (体素坐标散射完全正确)")
    else:
        print("❌ (警告：坐标计算有错位或丢失！)")
        
    # 过滤背景
    valid_mask = np.logical_or(sw_mask, hw_mask)
    sw_valid_features = sw_pseudo_img[:, valid_mask] 
    hw_valid_features = hw_pseudo_img[:, valid_mask]

    if sw_valid_features.size == 0:
        print("\n⚠️ 警告：两张伪图都是全 0，无可对比特征！")
        return

    # 指标 1: 余弦相似度 (Cosine Similarity)
    dot_product = np.sum(sw_valid_features * hw_valid_features, axis=0)
    norm_sw = np.linalg.norm(sw_valid_features, axis=0)
    norm_hw = np.linalg.norm(hw_valid_features, axis=0)
    
    valid_norms = (norm_sw > 0) & (norm_hw > 0)
    cos_sims = dot_product[valid_norms] / (norm_sw[valid_norms] * norm_hw[valid_norms])
    mean_cos_sim = np.mean(cos_sims) if len(cos_sims) > 0 else 0.0
    
    print(f"📐 余弦相似度 (Cos Sim)    : {mean_cos_sim:.6f}")

    # 指标 2 & 3: Max Error, MAE, MSE
    diff = sw_valid_features - hw_valid_features
    max_err = np.max(np.abs(diff))
    mae = np.mean(np.abs(diff))
    mse = np.mean(diff ** 2)
    
    print(f"💥 最大绝对误差 (Max Err)  : {max_err:.6f}")
    print(f"📉 平均绝对误差 (MAE)      : {mae:.6f}")
    print(f"📉 均方误差 (MSE)          : {mse:.6f}")

    # 指标 4: 信号量化噪声比 (SQNR)
    signal_power = np.mean(sw_valid_features ** 2)
    noise_power = mse
    if noise_power > 1e-12:
        sqnr = 10 * np.log10(signal_power / noise_power)
    else:
        sqnr = float('inf') 
        
    print(f"📡 信号量化噪声比 (SQNR)   : {sqnr:.2f} dB")
    
    print("="*60)
    if mean_cos_sim > 0.99 and iou == 1.0:
        print("🎉 诊断结果: 底层模块与软件金标准完美对齐，准备接入 2D CNN Backbone！")
    elif iou < 1.0:
        print("🚨 诊断结果: Scatter 过程出现坐标映射错误，请检查 X/Y 坐标的索引生成逻辑。")
    elif max_err > 1.0:
        print("🚨 诊断结果: 发现极大的数值刺突！可能存在定点数溢出 (Overflow) 或截断错误。")
    else:
        print("💡 诊断结果: 存在一定量化损耗，但无致命错误，可继续评估整体 mAP 表现。")
    print("="*60)


def parse_txt_to_pseudo_image(filepath: str, grid_h: int = 720, grid_w: int = 720, channels: int = 48, drop_log_path: str = "sw_dropped_data.txt") -> np.ndarray:
    """
    将文本文件解析并散射为伪图。丢弃的越界数据将写入 drop_log_path。
    """
    pseudo_image = np.zeros((channels, grid_h, grid_w), dtype=np.float32)

    if not os.path.exists(filepath):
        print(f"⚠️ 错误: 找不到文件 {filepath}")
        return pseudo_image

    valid_count = 0
    drop_count = 0

    print(f"⏳ 正在解析并散射 {filepath} 到 [{channels}, {grid_h}, {grid_w}] 的伪图...")

    # 同时打开读取文件和丢弃日志写入文件
    with open(filepath, 'r') as f, open(drop_log_path, 'w') as f_drop:
        for line_idx, line in enumerate(f):
            line_strip = line.strip()
            
            if not line_strip or line_strip[0].isalpha():
                # 如果是表头，也写进丢弃日志中，方便查看
                if line_strip and line_strip[0].isalpha():
                    f_drop.write(line_strip + " (HEADER)\n")
                continue

            parts = line_strip.replace(',', ' ').split()

            if len(parts) < channels + 2:
                continue

            features = np.array(parts[-channels:], dtype=np.float32)

            if len(parts) == 50:
                voxel_x = int(float(parts[0]))
                voxel_y = int(float(parts[1]))
            elif len(parts) >= 52:
                voxel_x = int(float(parts[1]))
                voxel_y = int(float(parts[2]))
            else:
                voxel_x = int(float(parts[0]))
                voxel_y = int(float(parts[1]))

            # 坐标验证与记录
            if 0 <= voxel_x < grid_w and 0 <= voxel_y < grid_h:
                pseudo_image[:, voxel_y, voxel_x] = features
                valid_count += 1
            else:
                # 越界丢弃：记录原文
                drop_count += 1
                f_drop.write(line_strip + f" (REASON: Out of Bounds X:{voxel_x}, Y:{voxel_y})\n")

    print(f"✅ 散射完成！成功填充了 {valid_count} 个 Pillar。")
    if drop_count > 0:
        print(f"🗑️ 过滤提醒: 有 {drop_count} 个越界点被丢弃，已保存至 -> {drop_log_path}")
        
    return pseudo_image


def parse_csv_to_pseudo_image(csv_path: str, grid_h: int = 720, grid_w: int = 720, channels: int = 48, max_by_dim: int = 0, drop_log_path: str = "hw_dropped_data.csv") -> np.ndarray:
    """
    读取 CSV 文件并散射到伪图中。将丢弃的越界及去重数据写入独立的 CSV 中。
    """
    pseudo_image = np.zeros((channels, grid_h, grid_w), dtype=np.float32)

    if not os.path.exists(csv_path):
        print(f"⚠️ 错误: 找不到文件 {csv_path}")
        return pseudo_image

    print(f"⏳ 正在使用 Pandas 读取 {csv_path} ...")
    
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"⚠️ 读取 CSV 失败: {e}")
        return pseudo_image

    if 'voxel_x' not in df.columns or 'voxel_y' not in df.columns:
        print("⚠️ 错误: CSV 中找不到 'voxel_x' 或 'voxel_y' 列！")
        return pseudo_image

    total_voxels = len(df)
    
    # --- 1. 处理越界数据 ---
    # 构建合法坐标掩码
    bounds_mask = (df['voxel_x'] >= 0) & (df['voxel_x'] < grid_w) & (df['voxel_y'] >= 0) & (df['voxel_y'] < grid_h)
    
    df_valid = df[bounds_mask].copy()
    df_dropped_bounds = df[~bounds_mask].copy()  # 提炼出被抛弃的越界行
    if not df_dropped_bounds.empty:
        df_dropped_bounds['drop_reason'] = 'out_of_bounds'
    
    # --- 2. 处理重复/冲突坐标 ---
    df_dropped_dups = pd.DataFrame()
    if df_valid.duplicated(subset=['voxel_x', 'voxel_y']).any():
        target_col = f'dim{max_by_dim}'
        if target_col not in df_valid.columns:
            print(f"⚠️ 警告: 找不到指定的比较列 {target_col}，回退到保留最后一条结果。")
            dup_mask = df_valid.duplicated(subset=['voxel_x', 'voxel_y'], keep='last')
        else:
            # 按目标维度降序排列
            df_valid = df_valid.sort_values(by=target_col, ascending=False)
            # 记录将要被丢弃的行（keep='first' 保留最大值，标记剩下的重复项为 True）
            dup_mask = df_valid.duplicated(subset=['voxel_x', 'voxel_y'], keep='first')
        
        # 提取被丢弃的重复项
        df_dropped_dups = df_valid[dup_mask].copy()
        if not df_dropped_dups.empty:
            df_dropped_dups['drop_reason'] = 'duplicate_conflict'
        
        # 过滤掉这些重复行
        df_valid = df_valid[~dup_mask]
        
    # --- 3. 合并保存所有丢弃的数据 ---
    df_dropped_all = pd.concat([df_dropped_bounds, df_dropped_dups])
    valid_count = len(df_valid)
    drop_count = len(df_dropped_all)

    if drop_count > 0:
        df_dropped_all.to_csv(drop_log_path, index=False)
        print(f"🗑️ 过滤提醒: 共剔除 {len(df_dropped_bounds)} 个越界点，{len(df_dropped_dups)} 个重复冗余点，已保存至 -> {drop_log_path}")

    if valid_count == 0:
        print("⚠️ 警告: CSV 中没有任何合法的体素坐标！")
        return pseudo_image

    # --- 4. 提取坐标和特征 ---
    x_coords = df_valid['voxel_x'].astype(int).values
    y_coords = df_valid['voxel_y'].astype(int).values

    feature_cols = [f'dim{i}' for i in range(channels)]
    missing_cols = [col for col in feature_cols if col not in df_valid.columns]
    if missing_cols:
        print(f"⚠️ 错误: CSV 中缺失以下特征列: {missing_cols[:5]} ...")
        return pseudo_image

    features = df_valid[feature_cols].values.astype(np.float32)

    # --- 5. 极速向量化散射 ---
    pseudo_image[:, y_coords, x_coords] = features.T

    print(f"✅ 硬件伪图生成完毕！最终映射了 {valid_count} 个独立 Pillar (共丢弃/过滤 {drop_count} 个点)。")
    return pseudo_image


def find_max_error_coordinates(sw_pseudo_img: np.ndarray, hw_pseudo_img: np.ndarray, top_k: int = 5):
    # ... 原有代码不变 ...
    assert sw_pseudo_img.shape == hw_pseudo_img.shape, "Shape 必须一致"
    
    abs_diff = np.abs(sw_pseudo_img - hw_pseudo_img)
    flat_indices = np.argsort(abs_diff.flatten())[::-1][:top_k]
    
    print("\n" + "="*60)
    print(f"🚨 误差最大的前 {top_k} 个坐标点分析报告 (Top-{top_k} Errors)")
    print("="*60)
    
    for rank, flat_idx in enumerate(flat_indices):
        c, y, x = np.unravel_index(flat_idx, abs_diff.shape)
        err_val = abs_diff[c, y, x]
        sw_val = sw_pseudo_img[c, y, x]
        hw_val = hw_pseudo_img[c, y, x]
        
        hw_q88 = int(hw_val * 256.0)
        
        print(f"[{rank+1}] 📍 坐标: (X={x}, Y={y}) | 通道: Ch[{c:02d}]")
        print(f"    -> 软件金标准 (SW) : {sw_val:10.5f}")
        print(f"    -> 硬件计算值 (HW) : {hw_val:10.5f}  (底层Q8.8定点整数: {hw_q88} / 16进制: 0x{hw_q88&0xFFFF:04X})")
        print(f"    -> 绝对误差 (Diff) : {err_val:10.5f}")
        print("-" * 50)
        
        if err_val < 0.5 and rank > 0:
            print("💡 后续误差已小于 0.5，属于正常量化波动，停止打印。")
            break

if __name__ == "__main__":

    hw_file = r"E:\verilog\pillarnest\temp\modelsim\pfn_layer_out.csv"  
    sw_file = r"E:\mmdetection3d\bev_48d_nonzero.txt" 

    print("\n--- 读取软件特征 ---")
    # 可以通过 drop_log_path 指定软硬件不同的日志文件位置
    sw_fake = parse_txt_to_pseudo_image(sw_file, grid_h=720, grid_w=720, channels=48, drop_log_path="sw_dropped_log.txt")
    print("\n--- 读取硬件特征 ---")
    hw_fake = parse_csv_to_pseudo_image(hw_file, grid_h=720, grid_w=720, channels=48, drop_log_path="hw_dropped_log.csv")

    evaluate_pseudo_image(sw_fake, hw_fake)
    find_max_error_coordinates(sw_fake, hw_fake, top_k=25)