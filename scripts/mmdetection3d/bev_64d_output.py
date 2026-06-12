import numpy as np
import torch

def save_bev64_to_txt(bev_tensor, txt_path, batch_idx=0, only_nonzero=False):
    """
    bev_tensor: [B, 48, H, W]，例如 [B, 48, 720, 720]
    txt_path: 输出 txt 路径
    batch_idx: 保存第几个 batch
    only_nonzero: True 时只保存特征非零的点，False 时保存全部 H*W 行
    """
    bev = bev_tensor[batch_idx].detach().to(torch.float32).cpu()  # [48, H, W]
    c, h, w = bev.shape
    if c != 64:
        raise ValueError(f"期望通道数为64，实际为 {c}")

    # [H, W, 48] -> [H*W, 48]
    feat = bev.permute(1, 2, 0).reshape(-1, 64)

    # 坐标网格，注意前两列要求是 x, y
    ys = torch.arange(h, dtype=torch.float32)
    xs = torch.arange(w, dtype=torch.float32)
    grid_y, grid_x = torch.meshgrid(ys, xs)  # 默认就是 ij

    coords = torch.stack(
        [grid_x.reshape(-1), grid_y.reshape(-1)], dim=1
    )  # [H*W, 2], 顺序是 x,y

    data = torch.cat([coords, feat], dim=1)  # [H*W, 50]

    if only_nonzero:
        mask = (feat.abs().sum(dim=1) > 0)
        data = data[mask]

    # 转成 float32 再保存
    data_np = data.numpy().astype(np.float32)

    # 每行 50 列：x y f0 ... f47
    np.savetxt(txt_path, data_np, fmt="%.6f")
    print(f"已保存: {txt_path}, 行数: {data_np.shape[0]}, 列数: {data_np.shape[1]}")






    save_bev64_to_txt(x, "bev_64d_nonzero.txt", batch_idx=0, only_nonzero=True)