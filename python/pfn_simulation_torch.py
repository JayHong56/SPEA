import os
import numpy as np

# ==========================================
# 1. TXT 权重与偏置加载工具
# ==========================================
def load_float_txt(weight_file: str, bias_file: str, out_dim=64, in_dim=9):
    """
    从 .txt 文件中读取浮点权重和偏置
    如果 np.savetxt 导出时是空格或换行分隔，loadtxt 会自动处理
    """
    if not os.path.exists(weight_file):
        print(f"⚠️ 警告: 找不到权重文件 {weight_file}，将使用随机初始化！")
        W = np.random.randn(out_dim, in_dim).astype(np.float16)
    else:
        # 直接读取 txt 并强转为 float16，最后 reshape 确保形状为 (48, 11)
        W = np.loadtxt(weight_file, dtype=np.float16).reshape(out_dim, in_dim)

    if not os.path.exists(bias_file):
        print(f"⚠️ 警告: 找不到偏置文件 {bias_file}，将使用随机初始化！")
        B = np.random.randn(out_dim).astype(np.float32)
    else:
        # 直接读取 txt 并强转为 float32，确保形状为 (48,)
        B = np.loadtxt(bias_file, dtype=np.float32).reshape(out_dim,)

    return W, B

# ==========================================
# 2. PFN 纯浮点算法层
# ==========================================
class PfnLayerFloatSoftware:
    def __init__(self, W_float16: np.ndarray, B_float32: np.ndarray):
        # 接收准备好的 numpy 矩阵
        self.W = W_float16
        self.B = B_float32

    def process_voxel(self, points_list: list) -> np.ndarray:
        if not points_list:
            return np.zeros(64, dtype=np.float32)

        # 1. 转换为输入矩阵 X，形状 (N, 11)，使用 float32 防止溢出
        X = np.array(points_list, dtype=np.float32)

        # 2. 矩阵乘法 + 偏置 (Linear Layer): X @ W^T + B
        linear_out = X @ self.W.T + self.B
        
        # 3. ReLU 激活函数 (小于 0 的变为 0)
        relu_out = np.maximum(0, linear_out)
        
        # 4. 最大池化 (Max Pooling)
        final_features = np.max(relu_out, axis=0)

        return final_features

# ==========================================
# 3. 测试与运行
# ==========================================
if __name__ == "__main__":
    print("="*60)
    print("🚀 PFN 纯软件算法 (TXT 文件读取版)")
    print("="*60)

    # 1. 从 txt 文件加载权重 (请替换为你的真实路径)
    # 如果你的 txt 是逗号分隔的，可以在 loadtxt 加上参数：delimiter=','
    # weight_path = r"E:\verilog\pillarnest\rtl\.parameter\float\W_fused_fp32.txt"
    # bias_path   = r"E:\verilog\pillarnest\rtl\.parameter\float\b_fused_fp32.txt"
    weight_path = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\W_fused_fp32.txt"
    bias_path   = "E:\\mmdetection3d\\my_output_parameters\\pfn_layer_fused_int_kitti\\b_fused_fp32.txt"

    W_fp16, B_fp32 = load_float_txt(weight_path, bias_path)
    print(f"✅ 成功加载权重矩阵 W 形状: {W_fp16.shape}，数据类型: {W_fp16.dtype}")
    print(f"✅ 成功加载偏置向量 B 形状: {B_fp32.shape}，数据类型: {B_fp32.dtype}")

    # 2. 实例化模型
    pfn_model = PfnLayerFloatSoftware(W_fp16, B_fp32)

    # 3. 模拟测试数据 (上游传来的点云集合)
    points_in_voxel = [
        [0.000000,-9.460938,0.528809,0.299988,-0.011719,0.050781,-0.001953,-0.078125,0.058594],
        [0.027344,-9.562500,0.532959,0.500000,0.015625,-0.050781,0.002197,-0.050781,-0.042969]
    ]

    print(f"\n📡 正在执行矩阵运算...")
    output_features = pfn_model.process_voxel(points_in_voxel)

    # 4. 打印输出
    print("\n✅ Voxel 特征提取完毕！(48维向量)")
    for c in range(64):
        print(f"      Ch[{c:02d}] 真实浮点特征 = {output_features[c]:.6f}")
    
    print("      ... (后 38 维省略)")