"""Install the Laya inference runtime separately from ROS and YOLO."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--skip-torch', action='store_true',
                        help='Keep the PyTorch build already installed in .venv-laya')
    args = parser.parse_args()
    if sys.maxsize < 2**32:
        raise SystemExit('Laya runtime requires a 64-bit Python installation')
    root = args.workspace.resolve()
    cache = root / '.cache/laya-install'
    cache.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update({'PIP_CACHE_DIR': str(cache / 'pip'), 'TMPDIR': str(cache),
                        'PYTHONNOUSERSITE': '1', 'PYTHONDONTWRITEBYTECODE': '1'})
    venv = root / '.venv-laya'
    python = venv / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python3')
    if not python.exists():
        subprocess.run([sys.executable, '-m', 'venv', '--without-pip', str(venv)],
                       check=True, env=environment)
    has_pip = subprocess.run([str(python), '-m', 'pip', '--version'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if not has_pip:
        with urllib.request.urlopen('https://pypi.org/pypi/pip/25.2/json', timeout=30) as response:
            info = next(x for x in json.load(response)['urls'] if x['filename'].endswith('.whl'))
        wheel = cache / info['filename']
        with urllib.request.urlopen(info['url'], timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != info['digests']['sha256']:
            raise RuntimeError('pip wheel checksum mismatch')
        wheel.write_bytes(data)
        bootstrap = environment.copy()
        bootstrap['PYTHONPATH'] = str(wheel)
        subprocess.run([str(python), '-m', 'pip', 'install', '--no-compile', '--no-deps',
                        str(wheel)], check=True, env=bootstrap)
    if not args.skip_torch:
        command = [str(python), '-m', 'pip', 'install', '--no-compile', 'torch==2.5.1']
        if platform.machine().lower() not in ('aarch64', 'arm64'):
            command += ['--index-url', 'https://download.pytorch.org/whl/cpu']
        subprocess.run(command, check=True, env=environment)
    subprocess.run([str(python), '-m', 'pip', 'install', '--no-compile',
                    'laya==0.4.8', 'transformers==4.57.6', 'huggingface-hub==0.36.0',
                    'numpy==1.26.4'], check=True, env=environment)
    print('Laya runtime ready:', python)
    print('Start: .venv-laya/bin/python3 scripts/laya_server.py')


if __name__ == '__main__':
    main()
