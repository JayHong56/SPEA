from mmdeploy.apis import torch2onnx
# from mmdeploy.backend.sdk.export_info import export2SDK

work_dir = 'mmdeploy_models/mmdet/onnx'
save_file = 'end2end.onnx'
deploy_cfg = '../mmdeploy/configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py'
model_cfg = r'configs/faster_rcnn/faster-rcnn_r50_fpn_1x_coco.py'
model_checkpoint = r'checkpoints/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20200620_230405-2fa62f3d.pth'
device = 'cuda'

# 1. convert model to onnx
torch2onnx( work_dir=work_dir, save_file=save_file, deploy_cfg=deploy_cfg, model_cfg=model_cfg,
           model_checkpoint=model_checkpoint, device=device)

# # 2. extract pipeline info for inference by MMDeploy SDK
# export2SDK(deploy_cfg, model_cfg, work_dir, pth=model_checkpoint,
#            device=device)