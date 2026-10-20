"""python 运行自整定测试.py --compiler C:/path/to/tcc.exe"""
from pathlib import Path
import argparse, shutil, subprocess, sys
p=argparse.ArgumentParser()
p.add_argument('--compiler',default=shutil.which('tcc'))
a=p.parse_args()
if not a.compiler:
    sys.exit('请用 --compiler 指定便携 TinyCC 的 tcc.exe 路径；本脚本不自动下载/安装。')
base=Path(__file__).resolve().parents[1]
cases=[('PORTABLE_TUNE_IMPLEMENTATION','通用自整定.h','自整定运行测试.c'),
       ('PORTABLE_TUNE_VALIDATION_IMPLEMENTATION','通用自整定验收.h','验收运行测试.c')]
for macro,header,case in cases:
    subprocess.run([a.compiler,'-std=c89','-Wall','-Werror','-D'+macro,
        '-include',str(base/header),'-run',str(base/'验证测试'/case)],check=True)
subprocess.run([a.compiler,'-std=c89','-Wall','-Werror','-include',
    str(base/'调用示例'/'自整定与原控制器接入.c'),'-run',
    str(base/'验证测试'/'接入运行测试.c')],check=True)
print('全部真实C运行测试通过。')
