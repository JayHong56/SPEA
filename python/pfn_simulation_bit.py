import os

# ==========================================
# 1. 硬件位阶与符号扩展工具 (物理级复刻)
# ==========================================
def mask_u(val: int, width: int) -> int:
    """无符号截断，保留低 width 位"""
    return val & ((1 << width) - 1)

def sign_extend(val: int, width: int) -> int:
    """符号位扩展并限制位宽 (模拟 FPGA 寄存器的自然溢出与截断)"""
    val = mask_u(val, width)
    sign_bit = 1 << (width - 1)
    if val & sign_bit:
        return val - (1 << width)
    return val

# ==========================================
# 2. .mem 文件解析器 (完美适配紧凑的 Hex)
# ==========================================
def readmemh_bias(filepath: str, num_elements: int, width: int):
    """解析 32-bit Bias 文件"""
    data = [0] * num_elements
    if not os.path.exists(filepath):
        print(f"⚠️ 警告: 未找到 {filepath}，Bias 默认初始化为 0。")
        return data
    
    with open(filepath, 'r') as f:
        row_idx = 0
        for line in f:
            line = line.split('//')[0].strip().replace('_', '')
            if not line: continue
            data[row_idx] = sign_extend(int(line, 16), width)
            row_idx += 1
            if row_idx >= num_elements: break
    return data

def readmemh_weight(filepath: str, rows: int, cols: int, width: int):
    """
    解析 176-bit Weight 文件 (每行 44 个 Hex 字符)
    对齐硬件：LSB-first (最右侧为 k=0，最左侧为 k=10)
    """
    data = [[0]*cols for _ in range(rows)]
    if not os.path.exists(filepath):
        print(f"⚠️ 警告: 未找到 {filepath}，Weight 默认初始化为 0。")
        return data

    chars_per_col = width // 4  # 16-bit = 4个Hex字符

    with open(filepath, 'r') as f:
        row_idx = 0
        for line in f:
            line = line.split('//')[0].strip().replace('_', '')
            if not line: continue
            
            # 补齐 44 个字符，防前导 0 被吞
            expected_len = cols * chars_per_col 
            line = line.zfill(expected_len)
            
            # 从右向左切片 (完美对齐 Verilog 的 [k*16+:16])
            for k in range(cols):
                right_idx = expected_len - k * chars_per_col
                left_idx = right_idx - chars_per_col
                val_hex = line[left_idx : right_idx]
                data[row_idx][k] = sign_extend(int(val_hex, 16), width)
            
            row_idx += 1
            if row_idx >= rows: break
    return data

# ==========================================
# 3. PFN 硬件行为级仿真核心
# ==========================================
class PfnLayerBitTrue:
    def __init__(self, weight_file="NOTHING", bias_file="NOTHING"):
        # 硬件参数对齐
        self.EXPAND_PT_DIM = 11
        self.OUT_PT_DIM = 48
        self.PT_WIDTH = 16
        self.WEIGHT_WIDTH = 16
        self.ACC_WIDTH = 32

        # 载入真实的定点权重
        self.weights = readmemh_weight(weight_file, self.OUT_PT_DIM, self.EXPAND_PT_DIM, self.WEIGHT_WIDTH)
        self.biases = readmemh_bias(bias_file, self.OUT_PT_DIM, self.ACC_WIDTH)

        # 硬件状态机寄存器
        self.max_pool_regs = [0] * self.OUT_PT_DIM
        self.pt_cnt = 0

    def process_point(self, pt_data_float: list, is_last: bool, voxel_x: int = 0, voxel_y: int = 0):
        """
        核心运算：浮点入，Q8.8出。单点输入，Voxel结束时吐出特征。
        """
        assert len(pt_data_float) == self.EXPAND_PT_DIM, f"输入必须是 {self.EXPAND_PT_DIM} 维"
        
        # ---------------------------------------------------------
        # [模拟 PFE] Float -> Q8.8 定点数转换
        # ---------------------------------------------------------
        q88_pt_data = []
        for val in pt_data_float:
            fixed_val = int(round(val * 256.0))
            q88_pt_data.append(sign_extend(fixed_val, self.PT_WIDTH))


        q88_pt_data.reverse()
        self.pt_cnt += 1

        # ---------------------------------------------------------
        # [模拟 PFN] MAC 累加树与 ReLU
        # ---------------------------------------------------------
        for ch in range(self.OUT_PT_DIM):
            # 1. Bias 对齐拼接 (对应 mac_comb[j] = bias_rom <<< 8)
            bias_val = sign_extend(self.biases[ch], self.ACC_WIDTH)
            mac = sign_extend(bias_val << 8, self.ACC_WIDTH)

            # 2. 11路乘累加 (对应 $signed(current_data) * $signed(weight))
            for k in range(self.EXPAND_PT_DIM):
                w = sign_extend(self.weights[ch][k], self.WEIGHT_WIDTH)
                d = q88_pt_data[k]
                mult_res = sign_extend(w * d, self.ACC_WIDTH)      
                mac = sign_extend(mac + mult_res, self.ACC_WIDTH)  

            # 3. 反量化倍数 (dequant_temp = mac_comb[j] * 994)
            dequant_temp = sign_extend(mac * 994, 64)

            # 4. ReLU 与截断 (严格对齐你的 Verilog: dequant_temp[35:20])
            if mac < 0:
                relu_val = 0
            else:
                shifted_val = dequant_temp >> 20
                # 注意：这里采用了直接位截断，复刻你的代码行为
                relu_val = mask_u(shifted_val, self.PT_WIDTH)

            # 5. 更新 Max Pool 寄存器
            self.max_pool_regs[ch] = max(self.max_pool_regs[ch], relu_val)

        # ---------------------------------------------------------
        # [模拟 PFN] Voxel 数据吐出
        # ---------------------------------------------------------
        if is_last:
            out_features = list(self.max_pool_regs)
            result = {
                "voxel_x": voxel_x,
                "voxel_y": voxel_y,
                "pt_cnt": self.pt_cnt,
                "data_q88": out_features,                                  # 定点 HEX 值
                "data_float": [val / 256.0 for val in out_features]        # 还原回真实物理量
            }
            # 清空状态机
            self.max_pool_regs = [0] * self.OUT_PT_DIM
            self.pt_cnt = 0
            return result
        else:
            return None

# ==========================================
# 4. 运行剧本：演示与打印输出
# ==========================================
if __name__ == "__main__":
    print("="*60)
    print("🚀 PFN 硬件级 Python 仿真器 (Float In -> Q8.8 Out)")
    print("="*60)

    # 1. 实例化仿真模型 (填入你的 weight.mem 和 bias.mem 路径)
    sim = PfnLayerBitTrue(weight_file=r"E:\mmdetection3d\my_output_parameters\pfn_layer_fused_int\mem\pfn_weight.mem", bias_file=r"E:/mmdetection3d/my_output_parameters/bias.mem")

    # # [演示用] 灌入一点测试权重，不然都是 0 没法看
    # for c in range(48):
    #     sim.biases[c] = 5
    #     for k in range(11):
    #         sim.weights[c][k] = 2

    # 2. 模拟上游传来的 3 个点 (浮点)
    points_in_voxel = [
        [-14.019531,21.699219,1.187500,21.000000,0.000000,0.000000,-0.003906,-0.304688,0.007813,0.027344,2.187500],
        [-14.019531,21.707031,1.796875,19.000000,0.000000,0.000000,0.003906,0.304688,0.007813,0.035156,2.796875]
    ]
    vox_x, vox_y = 266, 504

    # 3. 逐点输入并校验
    print(f"📡 送入 Voxel ({vox_x}, {vox_y})，包含 {len(points_in_voxel)} 个浮点坐标点...")
    for i, pt_float in enumerate(points_in_voxel):
        is_last_point = (i == len(points_in_voxel) - 1)
        
        out = sim.process_point(pt_float, is_last=is_last_point, voxel_x=vox_x, voxel_y=vox_y)
        
        if out is not None:
            print("\n✅ PFN Layer 运算完毕，触发输出：")
            print(f"  📍 坐标: X={out['voxel_x']}, Y={out['voxel_y']} | 总点数: {out['pt_cnt']}")
            print(f"  📊 48维特征输出 (前 4 维):")
            for c in range(48):
                print(f"      Ch[{c:02d}]: Q8.8定点值 = {out['data_q88'][c]:>5d} (0x{out['data_q88'][c]&0xFFFF:04X})  ->  还原浮点 = {out['data_float'][c]:.4f}")

