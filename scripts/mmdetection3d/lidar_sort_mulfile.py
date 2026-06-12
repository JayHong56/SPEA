import numpy as np
import os
import glob
import multiprocessing as mp  # 改用原生 multiprocessing
from functools import partial
from numba import njit
from tqdm import tqdm  # 【新增】引入进度条库

def load_kitti_bin(bin_path):
    """
    KITTI Velodyne bin:
        float32, shape = [N, 4]
        columns = x, y, z, reflectance
    """
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return pts


def save_kitti_bin(bin_path, pts):
    pts.astype(np.float32).tofile(bin_path)

# 加上这个装饰器，Numba 会在第一次调用时将其编译为机器码
@njit 
def estimate_ring_id_by_elevation(points_xyz, num_rings=64, num_iter=30):
    """
    使用 Numba 加速的垂直角估计 ring id 函数 (内存极简版)
    """
    x = points_xyz[:, 0]
    y = points_xyz[:, 1]
    z = points_xyz[:, 2]

    horizontal_dist = np.sqrt(x * x + y * y)
    elevation = np.arctan2(z, horizontal_dist)  # rad

    valid = np.isfinite(elevation)
    elev_valid = elevation[valid]

    # 1D k-means 初始化：用分位数初始化 64 个中心
    percentiles = np.linspace(0, 100, num_rings)
    centers = np.percentile(elev_valid, percentiles)

    num_valid_pts = elev_valid.shape[0]
    labels_valid = np.zeros(num_valid_pts, dtype=np.int64)

    for _ in range(num_iter):
        
        # 【核心修改点 1】：手写 C 风格循环代替 np.argmin(axis=1)
        # Numba 编译后速度极快，且省去了 N*64 大矩阵的内存开销
        for i in range(num_valid_pts):
            min_dist = 1e10  # 初始一个很大的距离
            best_k = 0
            for k in range(num_rings):
                d = np.abs(elev_valid[i] - centers[k])
                if d < min_dist:
                    min_dist = d
                    best_k = k
            labels_valid[i] = best_k

        # update centers
        new_centers = centers.copy()
        for k in range(num_rings):
            mask = labels_valid == k
            if np.any(mask):
                new_centers[k] = np.mean(elev_valid[mask])

        if np.max(np.abs(new_centers - centers)) < 1e-8:
            break

        centers = new_centers

    # 按 elevation 从低到高重新编号
    sort_idx = np.argsort(centers)
    remap = np.zeros(num_rings, dtype=np.int64)
    for new_id, old_id in enumerate(sort_idx):
        remap[old_id] = new_id

    num_all_pts = elevation.shape[0]
    labels_all = np.zeros(num_all_pts, dtype=np.int64)
    
    # 【核心修改点 2】：最后的所有点分配也用循环代替
    for i in range(num_all_pts):
        min_dist = 1e10
        best_k = 0
        for k in range(num_rings):
            d = np.abs(elevation[i] - centers[k])
            if d < min_dist:
                min_dist = d
                best_k = k
        labels_all[i] = best_k
        
    ring_id = remap[labels_all]
    centers_sorted = centers[sort_idx]

    return ring_id, centers_sorted

def circular_azimuth_key(azimuth, start_angle=0.0, direction="ccw"):
    """
    把 azimuth 映射成从 start_angle 开始的扫描时间顺序 key。

    KITTI Velodyne 坐标通常：
        x: 前
        y: 左
        z: 上

    azimuth = atan2(y, x)
    """
    two_pi = 2.0 * np.pi

    if direction == "ccw":
        key = (azimuth - start_angle) % two_pi
    elif direction == "cw":
        key = (start_angle - azimuth) % two_pi
    else:
        raise ValueError("direction must be 'ccw' or 'cw'")

    return key


def reorder_kitti_to_rotating_scan(
    pts,
    num_rings=64,
    start_angle_deg=0.0,
    direction="ccw",
):
    """
    近似恢复旋转扫描顺序。

    排序规则：
        primary key   = azimuth scanning order
        secondary key = ring_id

    输出：
        pts_reordered: 重排后的点云
        order: 原始索引到新顺序
        ring_id: 每个原始点估计出的 ring
        elev_centers: 估计出的 64 个垂直角中心
    """
    xyz = pts[:, :3]

    ring_id, elev_centers = estimate_ring_id_by_elevation(
        xyz,
        num_rings=num_rings,
    )

    x = xyz[:, 0]
    y = xyz[:, 1]

    azimuth = np.arctan2(y, x)
    azimuth = np.mod(azimuth, 2.0 * np.pi)

    start_angle = np.deg2rad(start_angle_deg)

    az_key = circular_azimuth_key(
        azimuth,
        start_angle=start_angle,
        direction=direction,
    )

    # np.lexsort 最后一个 key 是主 key
    # 所以这里主排序是 az_key，次排序是 ring_id
    order = np.lexsort((ring_id, az_key))

    pts_reordered = pts[order]

    return pts_reordered, order, ring_id, elev_centers

def process_single_file(in_bin, out_dir):
    """处理单个文件的独立函数，用于多进程调用"""
    try:
        filename = os.path.basename(in_bin)
        out_bin = os.path.join(out_dir, filename)
        
        # 读取点云
        pts = load_kitti_bin(in_bin)

        # 重新排序
        pts_reordered, order, ring_id, elev_centers = reorder_kitti_to_rotating_scan(
            pts,
            num_rings=64,
            start_angle_deg=0.0,
            direction="ccw",
        )

        # 保存结果
        save_kitti_bin(out_bin, pts_reordered)
        return True, filename
    except Exception as e:
        return False, f"{filename} 报错: {str(e)}"

def main():
    in_dir = r"E:\mmdetection3d\data\kitti\training\test"
    out_dir = r"E:\mmdetection3d\data\kitti\training\velodyne_test"

    os.makedirs(out_dir, exist_ok=True)
    
    bin_files = glob.glob(os.path.join(in_dir, "*.bin"))
    
    if not bin_files:
        print(f"[WARNING] 没有找到 .bin 文件！")
        return

    total_files = len(bin_files)
    print(f"[INFO] 找到 {total_files} 个文件准备处理...")
    
    num_workers = min(12, max(1, os.cpu_count() - 2))
    print(f"[INFO] 启动多进程，使用 {num_workers} 个 CPU 核心...")

    success_count = 0
    process_func = partial(process_single_file, out_dir=out_dir)

    with mp.Pool(processes=num_workers) as pool:
        # 【核心修改点】：用 tqdm 包装 pool.imap_unordered
        # total=total_files 让 tqdm 知道总共有多少任务，从而计算百分比和剩余时间
        results = pool.imap_unordered(process_func, bin_files, chunksize=20)
        
        for success, msg in tqdm(results, total=total_files, desc="处理进度", unit="文件"):
            if success:
                success_count += 1
            else:
                # 遇到错误时，必须用 tqdm.write 打印，否则会把进度条的排版打乱
                tqdm.write(f"[ERROR] {msg}")

    print("-" * 40)
    print(f"[INFO] 处理完成! 成功: {success_count}/{total_files}")


if __name__ == "__main__":
    main()