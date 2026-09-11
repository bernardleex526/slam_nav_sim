#!/usr/bin/env bash
# =============================================================================
# run_stage_a.sh —— 跑阶段A（割草机式建图）
# -----------------------------------------------------------------------------
# 用法：
#   bash tools/run_stage_a.sh simple            # 只跑 simple 环境
#   bash tools/run_stage_a.sh medium
#   bash tools/run_stage_a.sh complex
#   bash tools/run_stage_a.sh all               # 全部矩阵
#
# 说明：
#   · 先 clean_sim.sh 清场（避免环境串扰 —— 这是 2026-09-10 幽灵墙的成因之一）
#   · 日志写在 /tmp/stage_a_<env>_<时间戳>.log，同时又打屏幕
#   · simple 约 25min、medium 约 50min、complex 约 35min（按 0.5m/s 估）
# =============================================================================
set -u

ENVIRON="${1:-simple}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="/tmp/stage_a_${ENVIRON}_$(date +%H%M%S).log"

cd "$WS" || exit 1

echo "=========================================================="
echo "[stage_a] 环境=$ENVIRON"
echo "[stage_a] 日志=$LOG"
echo "=========================================================="

# ⚠️ source ROS 环境必须临时关 set -u（setup.bash 引用未定义变量）
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
set -u

exec 9>/tmp/sim_ws_stage_a.lock
flock -n 9 || { echo "[stage_a] Another Stage A runner owns the lock"; exit 1; }
if pgrep -x gzserver >/dev/null; then
    echo "[stage_a] Gazebo is already running; refusing to kill another experiment."
    exit 1
fi

ARGS=(--config src/sim_benchmark/config/stage_a_matrix.yaml)
if [ "$ENVIRON" != "all" ]; then
    ARGS+=(--only "$ENVIRON")
fi

echo "[stage_a] 开始：python3 src/sim_benchmark/scripts/stage_a_runner.py ${ARGS[*]}"
echo "[stage_a] （可能要跑 25~50 分钟，耐心）"

# stdbuf 让 print 不缓冲，配合 tee 才能实时看到进度；
# 日志同时写文件，便于事后回看
stdbuf -oL -eL python3 \
    src/sim_benchmark/scripts/stage_a_runner.py "${ARGS[@]}" 2>&1 \
    | tee "$LOG"
RC=${PIPESTATUS[0]}

echo ""
echo "=========================================================="
echo "[stage_a] 退出码=$RC   日志=$LOG"
echo "=========================================================="
exit "$RC"
