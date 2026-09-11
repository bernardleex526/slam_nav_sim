#!/usr/bin/env bash
# 验证平面运动：发一段 cmd_vel，看 /odom 是否真的移动、且不卡住。
# 这是阶段A"卡在 TURN"问题的直接判据。
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

TAG=movecheck_$$
ros2 launch sim_bringup sim.launch.py \
  robot:=wheeled_quadruped world:=simple mapping:=none \
  navigation:=none odom_source:=gt use_gimbal:=true \
  headless:=true metrics:=false run_name:=$TAG viz:=false \
  > /tmp/$TAG.log 2>&1 &
LPID=$!

for i in $(seq 1 45); do
  sleep 2
  ros2 topic list 2>/dev/null | grep -q "^/odom$" && break
done
echo "[ok] 仿真就绪，等 10s 落地稳定"
sleep 10

echo "=== 阶段1：直行 (linear.x=0.4, 3s) ==="
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}" > /dev/null 2>&1
sleep 2
echo "  直行后 odom:"
timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A4 "position:" | head -5

echo "=== 阶段2：原地左转 (angular.z=0.8, 3s) ==="
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.8}}" > /dev/null 2>&1
sleep 2
echo "  转向后 odom:"
timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A4 "position:" | head -5

echo "=== 阶段3：再直行 (linear.x=0.4, 3s) —— 验证转向后仍能前进 ==="
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}" > /dev/null 2>&1
sleep 2
echo "  二次直行后 odom:"
timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A4 "position:" | head -5

kill -INT $LPID 2>/dev/null
sleep 3
pkill -9 -f "[g]zserver" 2>/dev/null
echo "[done] 日志：/tmp/$TAG.log"
