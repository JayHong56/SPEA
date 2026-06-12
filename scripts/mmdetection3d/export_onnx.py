import torch
from mmdet3d.apis import init_model
import onnx
import onnxruntime as ort
import numpy as np
 
def export_model_to_onnx(config_path, checkpoint_path, output_path, input_shape=(1, 4, 16384)):
    # 初始化模型
    model = init_model(config_path, checkpoint_path, device='cuda:0')
    model.eval()
 
    # 创建输入张量
    input_tensor = torch.randn(input_shape)
 
    # 导出ONNX模型
    torch.onnx.export(
        model,
        input_tensor,
        output_path,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}},
        opset_version=11
    )
 
    print(f"ONNX模型已导出至 {output_path}")
 
def verify_onnx_model(onnx_path, input_shape=(1, 4, 16384)):
    # 加载ONNX模型
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)
 
    # 创建ONNX Runtime会话
    ort_session = ort.InferenceSession(onnx_path)
 
    # 生成随机输入
    input_data = np.random.randn(*input_shape).astype(np.float32)
 
    # 推理
    inputs = {ort_session.get_inputs()[0].name: input_data}
    outputs = ort_session.run(None, inputs)
 
    print(f"ONNX模型验证成功，输出形状: {outputs[0].shape}")
 
if __name__ == "__main__":
    config_path = "configs/pillarnest/pillarnest_small.py"
    checkpoint_path = "checkpoints/pillarnest_small.pth"
    output_path = "pillarnest.onnx"
 
    export_model_to_onnx(config_path, checkpoint_path, output_path)
    verify_onnx_model(output_path)