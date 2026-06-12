# trace_init_model.py
import os
import trace

# 1. 这里填你的 mmdet3d 仓库根目录路径
#    比如 C:\Users\86170\mmdet3d 之类
MMDET3D_ROOT = r'C:\Users\86170\mmdetection3d\mmdet3d'  # TODO: 改成你的实际路径
MMDETECTION3D_ROOT = r'C:\Users\86170\mmdetection3d'  # TODO: 改成你的实际路径


# 2. 你的配置和权重路径（照你原来的推理代码）
CONFIG_FILE = r'C:\Users\86170\mmdetection3d\configs\pillarnest\pillarnest_small.py'
CHECKPOINT_FILE = r'C:\Users\86170\mmdetection3d\checkpoints\pillarnest_small.pth'

def run_init_model():
    """只做模型初始化，触发所有相关代码执行。"""
    from mmdet3d.apis import init_model

    print(">>> call init_model")
    model = init_model(CONFIG_FILE, CHECKPOINT_FILE, device="cuda:0")
    print("Model built:", type(model))


def main():
    tracer = trace.Trace(count=1, trace=0)

    # 跑一次 init_model，trace 会记录所有执行过的 (filename, lineno)
    tracer.runfunc(run_init_model)

    results = tracer.results()

    root = os.path.abspath(MMDETECTION3D_ROOT)
    visited_files = set()

    for key, count in results.counts.items():
        # key 应该是一个 2 元组
        if not isinstance(key, tuple) or len(key) != 2:
            continue

        filename, lineno = key

        # filename 必须是字符串
        if not isinstance(filename, str):
            continue
        if not filename:
            continue

        abs_path = os.path.abspath(filename)

        # 只保留 .py 文件
        if not abs_path.endswith(".py"):
            continue

        # 只保留 mmdet3d 工程里的文件
        if not abs_path.startswith(root):
            continue

        visited_files.add(abs_path)

    out_file = os.path.join(root, "visited_files_init_model.txt")
    with open(out_file, "w", encoding="utf-8") as f:
        for p in sorted(visited_files):
            f.write(p + "\n")

    print(f"共记录到 {len(visited_files)} 个执行过的 .py 文件")
    print(f"结果已写入: {out_file}")


if __name__ == "__main__":
    main()