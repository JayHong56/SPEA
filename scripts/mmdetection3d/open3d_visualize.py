import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def visualize_bin_by_order(bin_file_path, colormap_name='rainbow'):
    """
    读取 bin 文件并根据点云读入的先后顺序进行上色可视化
    
    参数:
        bin_file_path (str): .bin 文件的路径
        colormap_name (str): matplotlib 色谱名称，如 'rainbow', 'jet', 'viridis'
    """
    # 1. 读取点云数据
    # 通常的自动驾驶点云 bin 文件是 float32 格式，每行 4 个数值 (x, y, z, intensity)
    # 如果你的数据只有 x, y, z，请将 4 改为 3
    print(f"正在读取文件: {bin_file_path} ...")
    try:
        points = np.fromfile(bin_file_path, dtype=np.float32).reshape(-1, 4)
    except FileNotFoundError:
        print(f"错误：找不到文件 {bin_file_path}。请检查路径。")
        return

    # 提取空间坐标 (x, y, z)
    xyz = points[:, :3]
    num_points = xyz.shape[0]
    print(f"共读取到 {num_points} 个点。")

    # 2. 根据读入顺序生成颜色
    # 生成从 0 到 num_points - 1 的索引
    indices = np.arange(num_points)
    
    # 归一化到 [0.0, 1.0] 的区间，适配色谱函数
    normalized_indices = indices / (num_points - 1)

    # 获取 matplotlib 色谱
    cmap = plt.get_cmap(colormap_name)
    
    # cmap 传入归一化的数组后，会返回 RGBA 格式的颜色 (N, 4)
    # Open3D 只需要 RGB，所以我们截取前三列 [:, :3]
    colors = cmap(normalized_indices)[:, :3]

    # 3. 构建 Open3D 点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # 4. 可视化
    print("启动可视化窗口...")
    o3d.visualization.draw_geometries(
        [pcd],
        window_name=f"Point Cloud Visualizer - Colored by Order ({colormap_name})",
        width=1024,
        height=768,
        left=50,
        top=50,
        point_show_normal=False
    )

if __name__ == "__main__":
    # 替换为你实际的 .bin 文件路径
    # 例如："000000.bin"
    sample_bin_path = r"E:\mmdetection3d\data\kitti\scripts\data\equal_chunk_interleave.bin" 
    
    # 运行可视化 (推荐使用 'rainbow' 或 'jet' 看时间顺序最明显)
    visualize_bin_by_order(sample_bin_path, colormap_name='rainbow')