#!/usr/bin/env bash
# 对比 use_gimbal=true/false 时的 TF 与 scan frame
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

for G in true false; do
  echo ""
  echo "################ use_gimbal=$G ################"
  TAG=gchk_${G}_$$
  ros2 launch sim_bringup sim.launch.py \
    robot:=wheeled_quadruped world:=simple mapping:=slam_toolbox \
    navigation:=none odom_source:=gt use_gimbal:=$G \
    headless:=true metrics:=false run_name:=$TAG viz:=false \
    > /tmp/$TAG.log 2>&1 &
  LPID=$!

  for i in $(seq 1 40); do
    sleep 2
    ros2 topic list 2>/dev/null | grep -q "^/scan$" && break
  done
  sleep 8

  echo "--- /scan header.frame_id ---"
  timeout 6 ros2 topic echo /scan --once --field header 2>/dev/null | head -5

  echo "--- TF: 所有到 lidar 的链路 ---"
  timeout 6 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child_frame_id" | head -20

  echo "--- slam_toolbox 警告 ---"
  grep -i "dropping message\|queue is full\|lidar_stabilized" /tmp/$TAG.log 2>/dev/null | head -3

  kill -INT $LPID 2>/dev/null
  sleep 4
  pkill -9 -f "[g]zserver" 2>/dev/null
  sleep 2
done
echo ""
echo "[done]"
