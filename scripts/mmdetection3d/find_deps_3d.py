# find_deps_3d.py
import os
import modulefinder

# 1. mmdet3d 仓库根目录：为了过滤出“自己仓库里的 .py”
#   如果这个脚本就放在仓库根目录，可以直接用当前文件的上级目录
MMDET3D_ROOT = os.path.dirname(os.path.abspath(__file__))

# 2. 要分析的脚本，就是刚才的 build_model_3d.py
TARGET_SCRIPT = os.path.join(MMDET3D_ROOT, 'build_model_3d.py')

def main():
    finder = modulefinder.ModuleFinder()

    print('Running script for dependency analysis:', TARGET_SCRIPT)
    # 运行脚本：在这期间所有 import 的模块都会被记录
    finder.run_script(TARGET_SCRIPT)

    needed_files = set()

    for name, mod in finder.modules.items():
        f = getattr(mod, '__file__', None)
        if not f:
            continue
        f = os.path.abspath(f)

        # 只要 .py 文件
        if not f.endswith('.py'):
            continue

        # 只统计 mmdet3d 仓库里的文件
        if f.startswith(MMDET3D_ROOT):
            needed_files.add(f)

    out_path = os.path.join(MMDET3D_ROOT, 'needed_files_mmdet3d_init_model.txt')
    with open(out_path, 'w', encoding='utf-8') as fp:
        for path in sorted(needed_files):
            fp.write(path + '\n')

    print(f'共发现 {len(needed_files)} 个 .py 文件')
    print(f'已写入: {out_path}')

if __name__ == '__main__':
    main()
