"""Install CPU YOLO in this workspace and fetch the demonstration assets."""

import argparse
import json
import os
import platform
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request


def download(url, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.download')
    with urllib.request.urlopen(url, timeout=90) as source:
        with temporary.open('wb') as target:
            shutil.copyfileobj(source, target)
    temporary.replace(destination)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    package = Path(__file__).resolve().parents[1]
    cache = workspace / '.cache'
    (cache / 'tmp').mkdir(parents=True, exist_ok=True)
    (workspace / '.yolo-config').mkdir(parents=True, exist_ok=True)
    (cache / 'matplotlib').mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update({
        'PYTHONNOUSERSITE': '1', 'PYTHONDONTWRITEBYTECODE': '1',
        'PIP_CACHE_DIR': str(cache / 'pip'), 'TMPDIR': str(cache / 'tmp'),
        'YOLO_CONFIG_DIR': str(workspace / '.yolo-config'),
        'MPLCONFIGDIR': str(cache / 'matplotlib'),
    })
    venv = workspace / '.venv-yolo'
    python = venv / 'bin/python3'
    print('Creating YOLO environment:', venv, flush=True)
    subprocess.run(
        [sys.executable, '-m', 'venv', '--without-pip', str(venv)],
        check=True, env=environment)
    with urllib.request.urlopen('https://pypi.org/pypi/pip/25.2/json', timeout=30) as source:
        files = json.load(source)['urls']
    wheel_info = next(item for item in files if item['filename'].endswith('.whl'))
    wheel = cache / wheel_info['filename']
    download(wheel_info['url'], wheel)
    bootstrap = environment.copy()
    bootstrap['PYTHONPATH'] = str(wheel)
    subprocess.run(
        [str(python), '-m', 'pip', 'install', '--quiet', '--no-compile', '--ignore-installed',
         '--no-deps', str(wheel)], check=True, env=bootstrap)
    print('Installing CPU PyTorch...', flush=True)
    torch_command = [str(python), '-m', 'pip', 'install', '--quiet', '--no-compile',
                     'torch==2.5.1', 'torchvision==0.20.1']
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        torch_command.extend(['--index-url', 'https://download.pytorch.org/whl/cpu'])
    subprocess.run(torch_command, check=True, env=environment)
    print('Installing YOLO and NumPy 1.x for ROS cv_bridge...', flush=True)
    subprocess.run(
        [str(python), '-m', 'pip', 'install', '--quiet', '--no-compile',
         'numpy==1.26.4', 'opencv-python==4.10.0.84', 'ultralytics==8.4.177'],
        check=True, env=environment)
    weights = package / 'weights/yolo11n.pt'
    if not weights.is_file():
        print('Downloading YOLO11n weights...', flush=True)
        download(
            'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt',
            weights)
    sign = package / 'models/stop_sign'
    if not (sign / 'meshes/stop_sign.dae').is_file():
        print('Downloading the 3D stop-sign model...', flush=True)
        with urllib.request.urlopen(
                'https://api.github.com/repos/osrf/gazebo_models/commits/master',
                timeout=30) as source:
            commit = json.load(source)['sha']
        base = f'https://raw.githubusercontent.com/osrf/gazebo_models/{commit}/'
        files = [
            'model.sdf', 'model.config', 'meshes/stop_sign.dae',
            'materials/textures/StopSign_Diffuse.png',
            'materials/textures/StopSign_Spec.png',
        ]
        with urllib.request.urlopen(
                'https://api.github.com/repos/osrf/gazebo_models/contents/'
                'stop_sign/materials/scripts', timeout=30) as source:
            scripts = json.load(source)
        files.extend(item['path'].removeprefix('stop_sign/')
                     for item in scripts if item['type'] == 'file')
        for name in files:
            download(base + 'stop_sign/' + name, sign / name)
        download(base + 'LICENSE', sign / 'LICENSE')
        (sign / 'SOURCE.txt').write_text(
            'Stop Sign by Cole Biesemeyer and Nate Koenig.\n'
            f'Source: https://github.com/osrf/gazebo_models/tree/{commit}/stop_sign\n'
            'License: Creative Commons Attribution 3.0 Unported; see LICENSE.\n')
    print('YOLO environment and assets are ready. Build my_bot with colcon.', flush=True)


if __name__ == '__main__':
    main()
