import os
import torch
import numpy as np
from PIL import Image

def save_feature_map_as_images(feature_map, save_dir="feature_vis", num_channels=48):
    """
    将形状为 [1, C, H, W] 的特征图保存为若干张灰度图
    
    参数:
        feature_map: torch.Tensor, shape = [1, C, H, W]
        save_dir: 保存图片的文件夹
        num_channels: 要保存的通道数
    """
    assert isinstance(feature_map, torch.Tensor), "feature_map 必须是 torch.Tensor"
    assert feature_map.dim() == 4, "feature_map 维度必须是 [1, C, H, W]"
    assert feature_map.shape[0] == 1, "batch size 必须为 1"
    
    _, C, H, W = feature_map.shape
    num_channels = min(num_channels, C)

    os.makedirs(save_dir, exist_ok=True)

    # 去掉 batch 维度 => [C, H, W]
    feature_map = feature_map[0].detach().cpu()

    for i in range(num_channels):
        channel_data = feature_map[i]  # [H, W]

        # 转成 numpy
        img = channel_data.numpy()

        # 归一化到 0~255，便于可视化
        img_min = img.min()
        img_max = img.max()

        if img_max > img_min:
            img = (img - img_min) / (img_max - img_min)
        else:
            img = np.zeros_like(img)

        img = (img * 255).astype(np.uint8)

        # 保存为灰度图
        img_pil = Image.fromarray(img, mode='L')
        img_pil.save(os.path.join(save_dir, f"channel_{i:02d}.png"))

    print(f"已保存 {num_channels} 张图片到: {save_dir}")


# =========================
# 使用示例
# =========================
if __name__ == "__main__":
    # 假设你的特征图是这个
    feature = torch.randn(1, 48, 720, 720)

    # 保存前48个通道
    save_feature_map_as_images(feature, save_dir="my_script/feature_vis_nuscenes", num_channels=48)