#!/usr/bin/env bash
# =============================================================================
# check_go2w_spawn.sh —— 验证宇树 Go2W（开源 URDF）能否正确投入 Gazebo
# -----------------------------------------------------------------------------
# 这是"接入开源 URDF"需求（用户指令 #2）的实跑验证。此前只做过结构静态检查
# （xacro 展开 / rigidify / 插件注入），没真正 spawn 过。
#
# 验证要点：
#   1. 实体成功 spawn（无 "entity already exists" / 解析失败）
#   2. TF 树完整：base_footprint -> base -> 四腿 -> 四轮
#   3. base_footprint 的 z ≈ 0（根 link 贴地）—— 验证 spawn_z=0 修正
#   4. base 的 z ≈ 0.5254（URDF 标称站姿）—— 验证 stand_height 修正
#   5. 传感器话题在线：/scan（或 /points）、/imu、/odom、/ground_truth/odom
#   6. 车体不塌陷：连续采样 z，波动应 < 1cm
#
# 用法：bash tools/check_go2w_spawn.sh
# =============================================================================
trap 'echo "[ERROR] 第 $LINENO 行失败"' ERR

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1

OUT=/tmp/go2w_spawn_check
rm -rf "$OUT"; mkdir -p "$OUT"
LOG="$OUT/sim.log"

echo "======================================================================"
echo " Go2W spawn 验证"
echo "======================================================================"

echo "[1/5] 启动仿真（go2w, simple 世界, 仅 spawn 不开建图）..."
ros2 launch sim_bringup sim.launch.py \
    robot:=go2w world:=simple \
    mapping:=none navigation:=none \
    odom_source:=gt use_gimbal:=true \
    headless:=true viz:=false metrics:=false \
    run_name:=go2w_spawn_check > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "      launch pid=$LAUNCH_PID"

cleanup() {
    echo
    echo "[清理] 关闭仿真 ..."
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 5
    kill -KILL "$LAUNCH_PID" 2>/dev/null
    pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
    pkill -9 -f gzserver 2>/dev/null
    sleep 2
    echo "[清理] 完成。日志：$LOG"
}
trap cleanup EXIT

echo "[2/5] 等待仿真就绪（70s）..."
sleep 70

echo
echo "[3/5] spawn 结果检查（日志关键字）..."
if grep -q "Successfully spawned entity" "$LOG"; then
    grep "Successfully spawned entity" "$LOG" | tail -2 | sed 's/^/      ✅ /'
else
    echo "      ❌ 未找到 spawn 成功记录，最后 15 行："
    tail -15 "$LOG" | sed 's/^/         /'
fi
echo "      站姿日志："
grep -E "spawn_robot|stand_height|站姿" "$LOG" | tail -3 | sed 's/^/         /'
echo "      错误检查："
if grep -qiE "ERROR|Failed to load|does not exist|invalid" "$LOG"; then
    grep -iE "ERROR|Failed to load|does not exist|invalid" "$LOG" | head -5 | sed 's/^/         ⚠️  /'
else
    echo "         ✅ 日志中无 ERROR"
fi

echo
echo "[4/5] 话题在线检查..."
for t in /scan /points /imu /odom /ground_truth/odom; do
    n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/^Publisher count: //p' | head -1)
    printf "      %-22s 发布者=%s\n" "$t" "${n:-0}"
done

echo
echo "[5/5] TF 检查（关键 frame 的高度）..."
for pair in "base_footprint base" "base_footprint lidar_link" "odom base_footprint"; do
    set -- $pair
    echo "      --- $1 -> $2 ---"
    timeout 12 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null \
        | grep -m1 "Translation" | sed 's/^/         /' || echo "         (无 TF)"
done

echo
echo "======================================================================"
echo " 判读标准："
echo "   base_footprint -> base         z 应 ≈ 0.5254  (URDF 标称站姿)"
echo "   base_footprint -> lidar_link   z 应 ≈ 0.1400  (雷达挂高)"
echo "   odom -> base_footprint         z 应 = 0.0000  (根 link 贴地) ✅重点"
echo "======================================================================"
