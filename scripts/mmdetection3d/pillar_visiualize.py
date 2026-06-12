import torch
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# ----------------- 你的数据 -----------------
# pillar_tensor: [6752, 20, 5]，内容 [x, y, z, intensity, 0]
# num_points   : [6752]
# coors_batch  : [6752, 4] -> [batch_idx, z_id, y_id, x_id]

# 这里假设它们已经在变量里：
# pillar_tensor, num_points, coors_batch = ...

# ----------------- 配置（你给的） -----------------
voxel_size = [0.15, 0.15, 8.0]
point_cloud_range = [-54, -54, -5.0, 54, 54, 3.0]

vx, vy, _ = voxel_size
x_min, y_min, z_min, x_max, y_max, z_max = point_cloud_range
# 对 pillar，我们直接用整根高度
vz = z_max - z_min    # = 8.0

# =====================================================
# 1. 先可视化点云（xyz + intensity 伪彩色）
# =====================================================
xyz = voxels[:, :, :3].reshape(-1, 3).cpu().numpy()
intensity = voxels[:, :, 3].reshape(-1).cpu().numpy()

i_min, i_max = intensity.min(), intensity.max()
intensity_norm = (intensity - i_min) / (i_max - i_min + 1e-6)
colors = plt.cm.jet(intensity_norm)[:, :3]   # 伪彩色

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(colors)

# =====================================================
# 2. 根据 num_points + coors_batch 画 pillar 边界框
# =====================================================
mask = num_points > 0                               # 只画有点的 pillar
valid_coors = coors[mask].cpu().numpy()      # [N_valid, 4]

boxes = []
for b, z_id, y_id, x_id in valid_coors:
    # x, y 方向按栅格索引 * voxel_size 再加上起始点
    x0 = x_min + x_id * vx
    x1 = x0 + vx

    y0 = y_min + y_id * vy
    y1 = y0 + vy

    # 对 pillar：整根柱子高度覆盖整个 z_range
    z0 = z_min
    z1 = z_max

    aabb = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=[x0, y0, z0],
        max_bound=[x1, y1, z1]
    )
    aabb.color = (1.0, 0.0, 0.0)   # 红色边界框
    boxes.append(aabb)

print(f"可视化 {len(boxes)} 个非空 pillar")

# =====================================================
# 3. 一起显示：点云 + 所有 pillar 边界框
# =====================================================
o3d.visualization.draw_geometries([pcd] + boxes)
