#!/usr/bin/env bash
# =============================================================================
# check_gt_odom.sh —— 起一次最小仿真并校验 /odom 与 /ground_truth/odom
# -----------------------------------------------------------------------------
# 用途：验证"真值里程计(p3d)是否可信"这个前提。
#   阶段A 的全部评测指标（ATE/RPE/地图裁剪范围）都建立在这个话题上。
#   若它本身发错位姿，后面所有结论都不成立。
#
# 做法：起 launch（navigation=none, mapping=none），机器人静止不动，
#       记录 40s 的两路里程计，逐点比对。
#
# 期望结果（运动学模式，机器人不动）：
#   /odom                x≈0  y≈0  z=0
#   /ground_truth/odom   x≈0  y≈0  z=0     ← 两张表应当基本重合
#
# 排查记录：
#   · 不要用 grep -oP（PCRE 未必编译进去，精简版 grep 会直接报错）
#   · 慎用 set -u（未定义变量会静默终止脚本、不打印任何信息，极难排查）
# =============================================================================
trap 'echo "[ERROR] 第 $LINENO 行失败，退出码 $?"' ERR

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1

OUT=/tmp/gt_odom_check
rm -rf "$OUT"; mkdir -p "$OUT"
LOG="$OUT/sim.log"

echo "[1/3] 启动仿真（无建图、无导航、无探索）..."
ros2 launch sim_bringup sim.launch.py \
    robot:=wheeled_quadruped world:=simple \
    mapping:=none navigation:=none \
    odom_source:=gt use_gimbal:=true \
    headless:=true viz:=false metrics:=false \
    run_name:=gt_odom_check > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "      launch pid=$LAUNCH_PID"

cleanup() {
    echo "[清理] 关闭仿真 ..."
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 5
    kill -KILL "$LAUNCH_PID" 2>/dev/null
    pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
    pkill -9 -f gzserver 2>/dev/null
    sleep 2
    echo "[清理] 完成。仿真日志：$LOG"
}
trap cleanup EXIT

echo "[2/3] 等待仿真就绪（30s）..."
sleep 30

for t in /odom /ground_truth/odom /scan; do
    n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/^Publisher count: //p' | head -1)
    echo "      $t  发布者=${n:-0}"
done

echo "[3/3] 记录 40s 里程计 ..."
python3 tools/verify_odom_consistency.py --duration 40 --out "$OUT/data"
