# build_model_3d.py

from mmdet3d.apis import init_model

config_file = r'configs\pillarnest\pillarnest_small.py'
checkpoint_file = r'checkpoints\pillarnest_small.pth'

def main():
    model = init_model(config_file, checkpoint_file, device='cuda:0')
    print('Model built:', type(model))

if __name__ == '__main__':
    main()
