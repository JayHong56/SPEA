import numpy as np

def sort_bin_by_azimuth(input_bin, output_bin):
    # 读取原始点云
    points = np.fromfile(input_bin, dtype=np.float32).reshape(-1, 4)
    
    # 提取 x 和 y 坐标
    x = points[:, 0]
    y = points[:, 1]
    
    # 计算水平方位角 (Azimuth)，范围是 -pi 到 pi
    # 使用 arctan2 完美还原雷达的 360 度旋转角度
    azimuth_angles = np.arctan2(y, x)
    
    # 获取按方位角从小到大排序的索引
    sorted_indices = np.argsort(azimuth_angles)
    
    # 对点云进行重新排序
    sorted_points = points[sorted_indices]
    
    # 保存为新的 bin 文件
    sorted_points.tofile(output_bin)
    print(f"数据已按雷达旋转顺序重排，并保存至: {output_bin}")

# 使用示例
sort_bin_by_azimuth(r"E:\mmdetection3d\data\kitti\scripts\data\000003.bin", r"E:\mmdetection3d\data\kitti\scripts\data\time_ordered_data.bin")