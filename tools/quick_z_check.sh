#!/usr/bin/env bash
# 快速验证：机器人 spawn 后 z 是否站得住（不被刚性化修复回归）
# 注意：不要开 set -u —— ROS 的 setup.bash 里有未绑定变量，会直接炸。
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

TAG=zhcheck_$$
ros2 launch sim_bringup sim.launch.py \
  robot:=wheeled_quadruped world:=simple mapping:=none \
  navigation:=none odom_source:=gt use_gimbal:=true \
  headless:=true metrics:=false run_name:=$TAG viz:=false \
  > /tmp/$TAG.log 2>&1 &
LPID=$!

echo "[wait] 仿真启动中（最多 90s）..."
for i in $(seq 1 45); do
  sleep 2
  if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    echo "[ok] /odom 已出现（${i}0s 内）"
    break
  fi
done

# 等 spawn + 落地稳定
sleep 12
echo "=== /odom 采样（每 3s 一次，看 z 是否稳定在 ~0.43） ==="
for k in 1 2 3 4; do
  OUT=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A4 "position:" | head -5 | tr '\n' ' ')
  echo "  t=${k}: $OUT"
  sleep 3
done

echo "=== 真值 odom（world 系） ==="
timeout 6 ros2 topic echo /ground_truth/odom --once 2>/dev/null | grep -A4 "position:" | head -5

echo "=== 腿关节是否还在（应为空，全被锁成 fixed） ==="
timeout 6 ros2 topic echo /joint_states --once 2>/dev/null | grep -o "name: [A-Za-z_0-9]*" | head -20

kill -INT $LPID 2>/dev/null
sleep 4
pkill -9 -f "[g]zserver" 2>/dev/null
pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
echo "[done] 日志：/tmp/$TAG.log"
