#!/usr/bin/env bash
set -euo pipefail
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
results_dir="${ROBOT_RESULTS_DIR:-$plugin_root/results/docker}"
mkdir -p "$results_dir"
if ! command -v docker >/dev/null 2>&1; then
  echo "Docker 不可用。请在已有 Docker 的 Ubuntu 24.04 或 Linux VM 中执行。" >&2
  exit 3
fi
docker build -f "$plugin_root/deploy/Dockerfile" -t robot-validation:jazzy "$plugin_root"
if [ "$#" -eq 0 ]; then
  set -- sim --scenario demo --output "/results/demo-$(date +%Y%m%d-%H%M%S)"
fi
exec docker run --rm --init --shm-size=1g \
  --mount "type=bind,src=$results_dir,dst=/results" \
  robot-validation:jazzy "$@"
