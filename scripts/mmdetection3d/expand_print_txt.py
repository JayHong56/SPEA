import torch
import numpy as np

def save_features_to_txt(features, coors, output_path="voxel_features_log.txt"):
    """
    将 features 根据 coors 的坐标信息打印到 txt 中。
    
    参数:
        features: torch.Tensor, shape [N, 32, 9]
        coors: torch.Tensor, shape [N, 4] 格式为 [batch_idx, z, voxel_y, voxel_x]
        output_path: str, 保存的文件路径
    """
    # 1. 转移到 CPU 并转为 NumPy 数组以加速数据读取
    features_np = features.cpu().numpy()
    coors_np = coors.cpu().numpy()
    
    num_voxels = features_np.shape[0]
    
    with open(output_path, 'w', encoding='utf-8') as f:
        for i in range(num_voxels):
            # 2. 提取坐标信息
            # 索引 2 对应第三维 (voxel_y)，索引 3 对应第四维 (voxel_x)
            voxel_y = int(coors_np[i, 2])
            voxel_x = int(coors_np[i, 3])
            batch_idx = int(coors_np[i, 0]) # 顺便提取 batch 编号，方便查错
            
            # 写入体素头部信息
            f.write(f"=== Voxel Index: {i} | batch: {batch_idx} | voxel_x: {voxel_x} | voxel_y: {voxel_y} ===\n")
            
            # 3. 提取当前体素内部的 32 个点及其 9 维特征
            voxel_points = features_np[i]  # shape: (32, 9)
            
            for pt_idx, point in enumerate(voxel_points):
                # 将 9 维特征格式化为保留 4 位小数的字符串
                pt_str = ", ".join([f"{val:.4f}" for val in point])
                f.write(f"  Point {pt_idx:2d}: [{pt_str}]\n")
            
            # 每个体素区块后加一个空行，增强可读性
            f.write("\n")
            
    print(f"成功将 {num_voxels} 个 Voxel 的数据保存至 {output_path}")

# ================= 模拟测试 =================
if __name__ == "__main__":
    # 为了方便测试，我们生成一个较小的规模 N=5 (代替 14840)
    N = 5
    
    mock_features = torch.randn(N, 32, 9)
    mock_coors = torch.randint(0, 400, (N, 4))
    mock_coors[:, 0] = 0 # Batch ID 设为 0
    mock_coors[:, 1] = 0 # Z 轴坐标 设为 0
    
    save_features_to_txt(mock_features, mock_coors, "test_voxel_features.txt")