#!/usr/bin/env bash
# =============================================================================
# run_stamp_probe.sh —— 启动仿真 + 运行时戳错位探针，一体化
# -----------------------------------------------------------------------------
# 目的：测量 /scan 时戳与 TF(odom->lidar_stabilized) 时戳的覆盖关系，
#       定位 slam_toolbox MessageFilter 持续丢帧（每 2.5s 一帧）的原因。
#
# 输出：/tmp/stamp_probe_out.txt
# 用法：bash tools/run_stamp_probe.sh [world] [probe_sec]
# =============================================================================
set -u

WORLD="${1:-simple}"
PROBE_SEC="${2:-30}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTLOG="/tmp/stamp_probe_out.txt"

cd "$WS" || exit 1

# ⚠️ source ROS 环境必须临时关 set -u（setup.bash 引用未定义变量）
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
set -u

bash tools/clean_sim.sh >/dev/null 2>&1

nohup ros2 launch sim_bringup sim.launch.py \
    world:="$WORLD" navigation:=none mapping:=slam_toolbox \
    headless:=true > /tmp/probe_sim.log 2>&1 &
LAUNCH_PID=$!
echo "[probe] launch PID=$LAUNCH_PID，等 gimbal 发布 TF ..."

# 等到 gimbal 首帧 TF（最多 40s）
READY=0
for i in $(seq 1 40); do
    sleep 1
    if grep -q "已开始发布 TF" /tmp/probe_sim.log 2>/dev/null; then
        READY=1
        echo "[probe] TF 已发布（${i}s），再等 6s 让 slam 起来 ..."
        break
    fi
done
[ "$READY" = "0" ] && echo "[probe] ⚠️ 40s 内未见 TF 发布日志，继续探测"
sleep 6

echo "[probe] 运行时戳探针 ${PROBE_SEC}s ..."
python3 tools/tf_stamp_probe.py "$PROBE_SEC" > "$OUTLOG" 2>&1
RC=$?

echo "[probe] 探针退出码=$RC，清理 ..."
pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f sim_ws/install 2>/dev/null
sleep 1
echo "[probe] 完成。结果在 $OUTLOG，仿真日志在 /tmp/probe_sim.log"
exit $RC
