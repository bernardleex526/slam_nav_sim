#!/usr/bin/env bash
set -euo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PATH="/opt/validation-venv/bin:$PATH"
export ROBOT_VALIDATE_ROOT="$plugin_root"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$plugin_root/build/contact_monitor${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export LIBGL_ALWAYS_SOFTWARE=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export PYTHONUNBUFFERED=1
exec robot-validate "$@"
