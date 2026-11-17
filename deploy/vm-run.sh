#!/usr/bin/env bash
# Reuse the Lima VM prepared for this workspace; no cloud service or host ROS install.
set -euo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
runtime_root="${ROBOT_VM_RUNTIME:-$(cd "$plugin_root/../.." && pwd)/work/ros2-runtime}"
if [[ ! -x "$runtime_root/lima/bin/limactl" ]]; then
  echo "Set ROBOT_VM_RUNTIME to the prepared Lima runtime directory." >&2
  exit 3
fi
export LIMA_HOME="$runtime_root/vm"
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  exec "$runtime_root/lima/bin/limactl" shell --start --workdir=/mnt/robot-validation rv \
    env "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" bash deploy/native-run.sh "$@"
fi
exec "$runtime_root/lima/bin/limactl" shell --start --workdir=/mnt/robot-validation rv \
  bash deploy/native-run.sh "$@"
