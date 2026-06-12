import os
import numpy as np

# ==========================================
# 1. 基础解析工具 (仅用于读取 .mem 文件并转为浮点矩阵)
# ==========================================
def sign_extend(val: int, width: int) -> int:
    """提取补码的真实带符号整数值"""
    val = val & ((1 << width) - 1)
    if val & (1 << (width - 1)):
        return val - (1 << width)
    return val

def load_bias_to_numpy(filepath: str, num_elements: int = 48) -> np.ndarray:
    """解析 32-bit Bias 文件，返回形状为 (48,) 的 NumPy 数组"""
    data = np.zeros(num_elements, dtype=np.float32)
    if not os.path.exists(filepath):
        print(f"⚠️ 警告: 未找到 {filepath}，Bias 初始化为 0。")
        return data
    
    with open(filepath, 'r') as f:
        idx = 0
        for line in f:
            line = line.split('//')[0].strip().replace('_', '')
            if line:
                data[idx] = sign_extend(int(line, 16), 32)
                idx += 1
                if idx >= num_elements: break
    return data

def load_weight_to_numpy(filepath: str, rows: int = 48, cols: int = 11) -> np.ndarray:
    """
    解析 176-bit Weight 文件，返回形状为 (48, 11) 的 NumPy 数组。
    注意：为了对齐你的硬件 LSB-first 逻辑，在返回前将矩阵列翻转。
    """
    data = np.zeros((rows, cols), dtype=np.float32)
    if not os.path.exists(filepath):
        print(f"⚠️ 警告: 未找到 {filepath}，Weight 初始化为 0。")
        return data

    chars_per_col = 16 // 4  # 16-bit = 4 hex chars

    with open(filepath, 'r') as f:
        row_idx = 0
        for line in f:
            line = line.split('//')[0].strip().replace('_', '')
            if line:
                expected_len = cols * chars_per_col 
                line = line.zfill(expected_len)
                
                # 按照硬件存放顺序截取 (最右边为 k=0，最左边为 k=10)
                for k in range(cols):
                    right_idx = expected_len - k * chars_per_col
                    left_idx = right_idx - chars_per_col
                    val_hex = line[left_idx : right_idx]
                    data[row_idx, k] = sign_extend(int(val_hex, 16), 16)
                
                row_idx += 1
                if row_idx >= rows: break
                
    # 🌟 核心操作：矩阵列反转
    # 硬件里 x 位于最高位 (k=10)，翻转后 w[:, 0] 就会对应 x，无需再翻转输入点云！
    return np.fliplr(data)

# ==========================================
# 2. PFN 纯软件算法层 (高度向量化)
# ==========================================
class PfnLayerSoftware:
    def __init__(self, weight_file="NOTHING", bias_file="NOTHING"):
        # 1. 载入权重和偏置矩阵
        self.W = load_weight_to_numpy(weight_file, rows=48, cols=11)
        self.B = load_bias_to_numpy(bias_file, num_elements=48)
        
        # 2. 计算纯浮点数学等价的全局 Scale
        # 硬件推导: Q8.8_Out = ((Q8.8_In * W) + B * 256) * 994 / (2^20)
        # 数学等价 Float_Out = (Float_In * W + B) * (994 / 2^20)
        self.scale = 994.0 / 1048576.0 

    def process_voxel(self, points_list: list) -> np.ndarray:
        """
        核心运算：一次性处理整个 Voxel 的点云，抛弃循环，全矩阵计算
        points_list: 形状 (N, 11) 的 Python 列表
        返回: 形状 (48,) 的特征向量
        """
        if not points_list:
            return np.zeros(48, dtype=np.float32)

        # X shape: (N, 11)
        X = np.array(points_list, dtype=np.float32)

        # 1. 矩阵乘法 + 偏置 (Linear Layer):  X @ W^T + B
        # 结果 shape: (N, 48)
        mac_out = X @ self.W.T + self.B

        # 2. 激活函数与反量化 (ReLU & Dequantization)
        # np.maximum(0, x) 即为 ReLU
        relu_out = np.maximum(0, mac_out * self.scale)

        # 3. 最大池化 (Max Pooling)
        # 沿着 N 维度 (axis=0) 聚合并取最大值 -> 结果 shape: (48,)
        final_features = np.max(relu_out, axis=0)

        return final_features

# ==========================================
# 3. 运行剧本：演示与打印输出
# ==========================================
if __name__ == "__main__":
    print("="*60)
    print("🚀 PFN 纯软件科学计算仿真器 (NumPy 向量化)")
    print("="*60)

    # 1. 实例化仿真模型 (填入你的真实文件路径)
    sim = PfnLayerSoftware(
        weight_file=r"E:/mmdetection3d/my_output_parameters/weight.mem", 
        bias_file=r"E:/mmdetection3d/my_output_parameters/bias.mem"
    )

    # 2. 上游传来的点云集合 (你的测试数据)
    points_in_voxel = [
        [-14.019531, 21.699219, 1.187500, 21.000000, 0.000000, 0.000000, -0.003906, -0.304688, 0.007813, 0.027344, 2.187500],
        [-14.019531, 21.707031, 1.796875, 19.000000, 0.000000, 0.000000,  0.003906,  0.304688, 0.007813, 0.035156, 2.796875]
    ]
    vox_x, vox_y = 266, 504

    # 3. 一键矩阵计算得出 Voxel 特征
    print(f"📡 送入 Voxel ({vox_x}, {vox_y})，包含 {len(points_in_voxel)} 个浮点坐标点...")
    
    out_features_float = sim.process_voxel(points_in_voxel)
    
    # 4. 打印输出
    print("\n✅ PFN Layer 运算完毕，触发输出：")
    print(f"  📍 坐标: X={vox_x}, Y={vox_y} | 总点数: {len(points_in_voxel)}")
    print(f"  📊 48维特征输出 (前 10 维):")
    for c in range(10):
        # 方便你与硬件数据比对，这里提供转换回 Q8.8 整数格式的代码
        q88_val = int(out_features_float[c] * 256)
        print(f"      Ch[{c:02d}]: 真实浮点 = {out_features_float[c]:.4f}  ->  理论 Q8.8值 = {q88_val:>5d} (0x{q88_val&0xFFFF:04X})")