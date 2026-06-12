import torch
from mmdet3d.apis import init_model, inference_detector, show_result_meshlab

from torchsummary import summary
from mmcv import Config
from mmdet.datasets import replace_ImageToTensor
from mmdet3d.datasets import build_dataloader, build_dataset
from mmdet3d.models import build_model
from mmcv.runner import (get_dist_info, init_dist, load_checkpoint,
                         wrap_fp16_model)
import sys
from parse_args import parse_args
import warnings
from mmdet.apis import set_random_seed
from mmcv.parallel import MMDataParallel
from mmcv.cnn import fuse_conv_bn
import numpy as np
import open3d as o3d

def tensor_to_numpy(x):
    if hasattr(x, 'tensor'):
        x = x.tensor
    if isinstance(x, torch.Tensor):
        x = x.detach().cpu().numpy()
    return x
import torch
import numpy as np
from mmcv.parallel import DataContainer


def extract_points_from_batch(each_data):
    """
    从 MMDetection3D dataloader 的 each_data 中取出当前帧点云。
    返回 numpy: [N, 3]
    """

    pts = each_data['points']

    while True:
        # 1. 一定要先判断 torch.Tensor
        # 因为 Tensor 也有 .data 属性，否则会死循环
        if isinstance(pts, torch.Tensor):
            pts = pts.detach().cpu().numpy()
            break

        # 2. numpy
        if isinstance(pts, np.ndarray):
            break

        # 3. MMDetection / MMCV 的 DataContainer
        if isinstance(pts, DataContainer):
            pts = pts.data
            continue

        # 4. list / tuple
        if isinstance(pts, (list, tuple)):
            pts = pts[0]
            continue

        raise TypeError(f"Unsupported points type: {type(pts)}")

    return pts[:, :3]


def make_open3d_point_cloud(points_xyz):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    return pcd


def make_box_lineset(corners, color=(1, 0, 0)):
    """
    corners: [8, 3]
    color:
        GT 建议红色:   (1, 0, 0)
        Pred 建议绿色: (0, 1, 0)
    """
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color for _ in lines])
    return line_set


def visualize_gt_boxes_open3d(points_xyz, gt_bboxes_3d, gt_labels_3d=None):
    """
    可视化点云 + GT 3D box
    """
    geometries = []

    pcd = make_open3d_point_cloud(points_xyz)
    geometries.append(pcd)

    # gt_bboxes_3d 是 LiDARInstance3DBoxes
    corners = gt_bboxes_3d.corners
    corners = tensor_to_numpy(corners)  # [N, 8, 3]

    for box_corners in corners:
        box = make_box_lineset(box_corners, color=(1, 0, 0))  # 红色 GT
        geometries.append(box)

    o3d.visualization.draw_geometries(geometries)

def visualize_gt_and_pred_open3d(points_xyz, gt_bboxes_3d, result, score_thr=0.3):
    geometries = []

    pcd = make_open3d_point_cloud(points_xyz)
    geometries.append(pcd)

    # -------------------------
    # GT boxes: 红色
    # -------------------------
    gt_corners = tensor_to_numpy(gt_bboxes_3d.corners)

    for box_corners in gt_corners:
        geometries.append(make_box_lineset(box_corners, color=(1, 0, 0)))

    # -------------------------
    # Pred boxes: 绿色
    # -------------------------
    pred = result[0]
    if 'pts_bbox' in pred:
        pred = pred['pts_bbox']

    pred_boxes = pred['boxes_3d']
    pred_scores = pred['scores_3d']

    keep = pred_scores > score_thr

    pred_corners = pred_boxes[keep].corners
    pred_corners = tensor_to_numpy(pred_corners)

    for box_corners in pred_corners:
        geometries.append(make_box_lineset(box_corners, color=(0, 1, 0)))

    print("Pred num after score_thr:", len(pred_corners))

    o3d.visualization.draw_geometries(geometries)

def get_bin_path_from_each_data(each_data):
    """
    从 MMDetection3D dataloader 返回的 each_data 中提取当前帧 .bin 文件路径
    兼容结构：
        each_data['img_metas'] = [DataContainer([[{...}]])]
    """

    img_metas = each_data['img_metas']

    # 通常是 list: [DataContainer(...)]
    if isinstance(img_metas, (list, tuple)):
        img_metas = img_metas[0]

    # DataContainer
    if hasattr(img_metas, 'data'):
        img_metas = img_metas.data

    # 通常是 [[dict]]
    while isinstance(img_metas, (list, tuple)):
        img_metas = img_metas[0]

    # 此时应该是 dict
    if not isinstance(img_metas, dict):
        raise TypeError(f"Unsupported img_metas type: {type(img_metas)}")

    return img_metas.get('pts_filename', None), img_metas.get('sample_idx', None)


import open3d
def main():

    open3d.utility.set_verbosity_level(open3d.utility.VerbosityLevel.Error)

    config_file = 'configs\pointpillars\hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class.py'
    checkpoint_file = 'checkpoints\hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20200620_230421-aa0f3adb.pth'
    args = parse_args()

    cfg = Config.fromfile(config_file)
    if args.cfg_options is not None:
        cfg.merge_from_dict(args.cfg_options)
    # set cudnn_benchmark
    if cfg.get('cudnn_benchmark', False):
        torch.backends.cudnn.benchmark = True

    cfg.model.pretrained = None
    # in case the test dataset is concatenated
    samples_per_gpu = 1
    if isinstance(cfg.data.test, dict):
        cfg.data.test.test_mode = True
        samples_per_gpu = cfg.data.test.pop('samples_per_gpu', 1)
        if samples_per_gpu > 1:
            # Replace 'ImageToTensor' to 'DefaultFormatBundle'
            cfg.data.test.pipeline = replace_ImageToTensor(
                cfg.data.test.pipeline)
    elif isinstance(cfg.data.test, list):
        for ds_cfg in cfg.data.test:
            ds_cfg.test_mode = True
        samples_per_gpu = max(
            [ds_cfg.pop('samples_per_gpu', 1) for ds_cfg in cfg.data.test])
        if samples_per_gpu > 1:
            for ds_cfg in cfg.data.test:
                ds_cfg.pipeline = replace_ImageToTensor(ds_cfg.pipeline)

    if args.gpu_ids is not None:
        cfg.gpu_ids = args.gpu_ids[0:1]
        warnings.warn('`--gpu-ids` is deprecated, please use `--gpu-id`. '
                      'Because we only support single GPU mode in '
                      'non-distributed testing. Use the first GPU '
                      'in `gpu_ids` now.')
    else:
        cfg.gpu_ids = [args.gpu_id]
    # init distributed env first, since logger depends on the dist info.
    if args.launcher == 'none':
        distributed = False
    else:
        distributed = True
        init_dist(args.launcher, **cfg.dist_params)
    # set random seeds
    if args.seed is not None:
        set_random_seed(args.seed, deterministic=args.deterministic)

    cfg.data.workers_per_gpu = 0
    # build the dataloader
    dataset = build_dataset(cfg.data.test)
    data_loader = build_dataloader(
        dataset,
        samples_per_gpu=samples_per_gpu,
        workers_per_gpu=cfg.data.workers_per_gpu,
        dist=distributed,
        shuffle=False)
    
    # build the model and load checkpoint
    cfg.model.train_cfg = None
    model = build_model(cfg.model, test_cfg=cfg.get('test_cfg'))
    fp16_cfg = cfg.get('fp16', None)
    if fp16_cfg is not None:
        wrap_fp16_model(model)
    checkpoint = load_checkpoint(model, checkpoint_file, map_location='cpu')
    if args.fuse_conv_bn:
        model = fuse_conv_bn(model)
    # old versions did not save class info in checkpoints, this walkaround is
    # for backward compatibility
    if 'CLASSES' in checkpoint.get('meta', {}):
        model.CLASSES = checkpoint['meta']['CLASSES'] 
    else:
        model.CLASSES = dataset.CLASSES
    # palette for visualization in segmentation tasks
    if 'PALETTE' in checkpoint.get('meta', {}):
        model.PALETTE = checkpoint['meta']['PALETTE']
    elif hasattr(dataset, 'PALETTE'):
        # segmentation dataset has `PALETTE` attribute
        model.PALETTE = dataset.PALETTE


    if not distributed:
        model = MMDataParallel(model, device_ids=[0])
        model.eval()
        dataset = data_loader.dataset

        # model = init_model(config_file, checkpoint_file, device='cuda:0')
        # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # t = model().to(device)
        # with open('model_print.txt', 'w') as f:
        #     print(model, file=f)
        # pcd = 'demo/data/kitti/kitti_000008.bin'
        # pcd = 'demo/data/nuscenes/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151604048025.pcd.bin'

        outputs = []
        each_count = 1
        for i, each_data in enumerate(data_loader):
            each_count += 1
            # if each_count == 4:
            bin_path, sample_idx = get_bin_path_from_each_data(each_data)
            print(f"[iter={i:06d}] sample_idx={sample_idx}, bin_path={bin_path}")

            # result, data = inference_detector(model, each_data)

            # if each_count == 28:
            #     breakpoint()

            with torch.no_grad():
                result = model(return_loss=False, rescale=True, **each_data)

            out_dir = './output/output_test_kitti'
            each_data['img_metas'] = each_data['img_metas'][0].data
            each_data['points'] = each_data['points'][0].data

            show_result_meshlab(
                each_data,
                result,
                out_dir,
                show=True
            )
        # print(dataset.evaluate(outputs, metric='bbox'))
if __name__ == '__main__':
    # torch.multiprocessing.set_start_method('spawn', force=True)
    sys.argv = ['2-kitti_pointpillar.py', 
                r'configs\pointpillars\hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class.py', 
                r'checkpoints\hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20200620_230421-aa0f3adb.pth', 
                '--show',
                '--show-dir', r'results']
    main()