#!/usr/bin/env bash
set -euo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
python_bin="${ROBOT_PYTHON:-python3}"
"$python_bin" -m venv "$plugin_root/.venv"
"$plugin_root/.venv/bin/python" -m pip install 'setuptools>=68' wheel
"$plugin_root/.venv/bin/python" -m pip install --no-build-isolation "$plugin_root/third_party/breezyslam/python"
"$plugin_root/.venv/bin/python" -m pip install -e "$plugin_root"
echo "安装完成。命令：$plugin_root/.venv/bin/robot-validate"
