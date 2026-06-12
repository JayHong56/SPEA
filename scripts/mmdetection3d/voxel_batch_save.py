import torch
import numpy as np

def save_voxel_data_to_txt(voxel, num_points, coors, output_path="voxel_data.txt"):
    """
    将体素数据按批次(batch)和体素(voxel)为间隔保存到txt中
    """
    # 将数据转移到CPU并转为numpy数组，方便逐行处理和格式化
    voxel_np = voxel.cpu().numpy()
    num_points_np = num_points.cpu().numpy()
    coors_np = coors.cpu().numpy()

    # 假设 coors 的结构为 (batch_idx, z, y, x) 或类似结构，第一列通常为 batch_idx
    batch_indices = coors_np[:, 0]
    unique_batches = np.unique(batch_indices)

    with open(output_path, 'w', encoding='utf-8') as f:
        for batch_idx in unique_batches:
            f.write(f"==================== Batch {int(batch_idx)} ====================\n")
            
            # 获取属于当前 batch 的所有体素的索引
            batch_mask = (batch_indices == batch_idx)
            voxel_idx_in_batch = np.where(batch_mask)[0]

            for v_idx in voxel_idx_in_batch:
                current_coors = coors_np[v_idx]
                n_pts = int(num_points_np[v_idx])
                
                # 写入体素分隔头信息
                f.write(f"  --- Voxel Index: {v_idx} | Coordinates: {current_coors.tolist()} | Valid Points: {n_pts} ---\n")
                
                # 根据 num_points 截取真实有效的点，丢弃因为凑齐32而padding进去的无效点
                valid_points = voxel_np[v_idx, :n_pts, :]
                
                # 写入该体素内的每一个点
                for pt_idx, point in enumerate(valid_points):
                    # 保留4位小数格式化点云特征(例如: x, y, z, intensity)
                    point_str = ", ".join([f"{val:.4f}" for val in point])
                    f.write(f"    Point {pt_idx:2d}: [{point_str}]\n")
            
            # 每个batch结束后空一行
            f.write("\n")
            
    print(f"数据已成功保存至 {output_path}")

# ================= 模拟测试用例 =================
if __name__ == "__main__":
    # 生成一些模拟数据来测试代码
    num_voxels = 10 # 测试时只用10个voxel代替16534以方便查看
    
    mock_voxel = torch.randn((num_voxels, 32, 4))
    # 随机生成每个voxel的有效点数(1到32之间)
    mock_num_points = torch.randint(1, 33, (num_voxels,))
    # 模拟坐标：第一维是batch(0或1)，后三维是空间坐标
    mock_coors = torch.randint(0, 50, (num_voxels, 4))
    mock_coors[:, 0] = torch.randint(0, 2, (num_voxels,)) # Batch ID 为 0 或 1

    # 运行函数
    save_voxel_data_to_txt(mock_voxel, mock_num_points, mock_coors, "sample_output.txt")