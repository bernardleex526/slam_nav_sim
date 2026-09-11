#!/usr/bin/env bash
# =============================================================================
# verify_slam_alive.sh —— 用 ~2 分钟确认 slam_toolbox 在当前参数下不会 FATAL
# =============================================================================
#
# 为什么需要它：
#   2026-09-11 把 correlation_search_space_dimension 从 0.5 收到 0.25 后，
#   slam_toolbox 在第一帧 scan 后约 75 秒崩溃：
#       Mapper FATAL ERROR - unable to get pointer in probability search!
#       [ERROR] process has died [exit code -6]
#   但它【死得很安静】：探索照常跑满 40 分钟，日志里没有任何 slam 异常，
#   只在最后存图时才暴露（/map 没有发布者）—— medium + complex 两轮
#   86 分钟全部作废。
#
#   所以任何改动 slam_toolbox 参数的动作，都必须先跑本脚本（2 分钟），
#   确认「没 FATAL + 进程活着 + /map 有发布者」，再上 50 分钟的长实验。
#
# 用法：
#   bash tools/verify_slam_alive.sh medium [等待秒数]
# =============================================================================
set -uo pipefail

WORLD="${1:-medium}"
DUR="${2:-110}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="/tmp/verify_slam_${WORLD}_$(date +%H%M%S).log"

cd "$ROOT" || exit 1

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
set -u

echo "=========================================================="
echo "[verify] world=$WORLD  等待=${DUR}s"
echo "[verify] 日志=$LOG"
echo "=========================================================="

echo "[verify] 清场..."
bash tools/clean_sim.sh 2>&1 | tail -1

setsid nohup ros2 launch sim_bringup sim.launch.py \
    robot:=wheeled_quadruped world:="$WORLD" mapping:=slam_toolbox \
    navigation:=none odom_source:=gt use_gimbal:=false headless:=true \
    run_name:="verify_${WORLD}" > "$LOG" 2>&1 < /dev/null &

echo "[verify] 仿真已启动，等待 ${DUR}s ..."
sleep "$DUR"

echo
echo "=== 1) FATAL 次数（必须为 0）==="
grep -c 'FATAL ERROR' "$LOG" || echo 0

echo "=== 2) slam 进程数（必须 >= 1）==="
ps -eo cmd | grep -c '[s]ync_slam_toolbox_node' || echo 0

echo "=== 3) slam 日志尾部 ==="
grep 'sync_slam_toolbox_node' "$LOG" | tail -4

echo "=== 4) /map 体检 ==="
timeout 20 ros2 topic info -v /map 2>&1 | grep -i 'publisher count' \
    || echo "（无法判定：CLI discovery 未收敛）"

echo
echo "=== 判读 ==="
FATAL=$(grep -c 'FATAL ERROR' "$LOG" || true)
ALIVE=$(ps -eo cmd | grep -c '[s]ync_slam_toolbox_node' || true)
if [ "${FATAL:-0}" -eq 0 ] && [ "${ALIVE:-0}" -ge 1 ]; then
    echo "✅ slam 存活且无 FATAL —— 可以上长实验"
    RC=0
else
    echo "❌ slam 未存活或发生 FATAL —— 禁止上长实验"
    RC=1
fi

echo "[verify] 收尾清场..."
bash tools/clean_sim.sh 2>&1 | tail -1
exit "$RC"
