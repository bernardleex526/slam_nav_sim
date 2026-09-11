#!/usr/bin/env python3
"""Package source and measured results without build caches or old run artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    args = parser.parse_args()
    target = Path(args.directory).resolve()
    target.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / 'DELIVERY_SCOPE.md', target / '运行与范围说明.md')
    source_zip = target / 'sim_ws_source.zip'
    hashes = {}
    with zipfile.ZipFile(source_zip, 'w', zipfile.ZIP_DEFLATED) as archive:
        for directory in ('src', 'tools'):
            for path in sorted((ROOT / directory).rglob('*')):
                if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc':
                    continue
                relative = path.relative_to(ROOT).as_posix()
                archive.write(path, 'sim_ws/'+relative)
                hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
        archive.write(ROOT / 'DELIVERY_SCOPE.md', 'sim_ws/DELIVERY_SCOPE.md')
    (target / 'source_sha256.json').write_text(json.dumps(hashes, indent=2))
    checks = target / 'checks'
    checks.mkdir(exist_ok=True)
    paths = {
        'contact.json': Path.home()/'.ros/sim_delivery_checks/contact/result.json',
        'model_navigation.json': Path.home()/'.ros/sim_delivery_checks/navigation/model_acceptance.json',
        'dynamic_obstacles.json': Path.home()/'.ros/sim_review_20260911/dynamic_obstacles/result.json',
    }
    for robot in ('wheeled_quadruped', 'quadruped', 'go2w', 'agv'):
        paths['planar_'+robot+'.json'] = Path.home()/f'.ros/sim_review_20260911/planar_{robot}/result.json'
    for name, path in paths.items():
        if path.is_file():
            shutil.copy2(path, checks / name)
    versions = subprocess.run(['dpkg-query', '-W', 'ros-humble-navigation2', 'ros-humble-nav2-bringup',
        'ros-humble-slam-toolbox', 'ros-humble-rmw-cyclonedds-cpp', 'gazebo'], capture_output=True, text=True)
    (target / 'runtime_versions.txt').write_text(versions.stdout+versions.stderr)
    print(source_zip)


if __name__ == '__main__':
    main()
