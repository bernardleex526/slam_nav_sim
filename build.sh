#!/usr/bin/env bash
# =============================================================================
# 一键构建
# -----------------------------------------------------------------------------
#   bash build.sh              # 全部包
#   bash build.sh robots       # 只构建机器人与仿真相关包
#   bash build.sh --clean      # 先清空 build/install/log 再构建
# =============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "找不到 /opt/ros/humble/setup.bash，先跑 install_deps.sh"
  exit 1
fi

if [[ "${1:-}" == "--clean" ]]; then
  echo "==> 清理 build/install/log"
  rm -rf build install log
  shift
fi

PKGS=()
case "${1:-all}" in
  robots) PKGS=(--packages-up-to robot_descriptions sim_worlds) ;;
  map)    PKGS=(--packages-select sim_worlds) ;;
  nav)    PKGS=(--packages-up-to nav2_config) ;;
  all)    PKGS=() ;;
  *)      echo "未知目标：$1（可选 all/robots/map/nav）"; exit 2 ;;
esac

echo "==> colcon build ${PKGS[*]:-（全部包）}"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "${PKGS[@]}"

echo
echo "构建完成。先 source 再启动："
echo "  source $WS/install/setup.bash"
echo "  ros2 launch sim_bringup sim.launch.py world:=simple check:=true"
