import numpy as np


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


def estimate_ring_id_by_elevation(points_xyz, num_rings=64, num_iter=30):
    """
    根据垂直角估计 ring id。

    注意：
    这不是严格 laser_id。
    严格 laser_id 需要 HDL-64E 的 per-laser calibration。
    这里只是根据 elevation angle 聚类得到近似 ring。
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

    for _ in range(num_iter):
        # assign
        dist = np.abs(elev_valid[:, None] - centers[None, :])
        labels_valid = np.argmin(dist, axis=1)

        # update
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

    labels_all = np.zeros_like(elevation, dtype=np.int64)
    dist_all = np.abs(elevation[:, None] - centers[None, :])
    labels_all = np.argmin(dist_all, axis=1)
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


def main():
    in_bin = r"E:\mmdetection3d\data\kitti\training\velodyne_val\000004.bin"
    out_bin = r"E:\mmdetection3d\data\kitti\scripts\data\equal_chunk_interleave.bin"

    pts = load_kitti_bin(in_bin)
    out_bin = out_bin
    order = np.
    pts_reordered, order, ring_id, elev_centers = reorder_kitti_to_rotating_scan(
        pts,
        num_rings=64,
        start_angle_deg=0.0,
        direction="ccw",
    )
    r
    save_kitti_bin(out_bin, pts_reordered)

    print(f"[INFO] input points  = {pts.shape}")
    print(f"[INFO] output points = {pts_reordered.shape}")
    print(f"[INFO] saved to: {out_bin}")

    print("[INFO] estimated elevation centers in degree:")
    print(np.rad2deg(elev_centers))


if __name__ == "__main__":
    main()