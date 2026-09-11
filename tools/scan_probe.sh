#!/usr/bin/env bash
# 起仿真，采集 /scan，统计有效读数与距离分布，用于定位"墙为什么没建出来"
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

TAG=scanchk_$$
ros2 launch sim_bringup sim.launch.py \
  robot:=wheeled_quadruped world:=simple mapping:=slam_toolbox \
  navigation:=none odom_source:=gt use_gimbal:=true \
  headless:=true metrics:=false run_name:=$TAG viz:=false \
  > /tmp/$TAG.log 2>&1 &
LPID=$!

for i in $(seq 1 45); do
  sleep 2
  ros2 topic list 2>/dev/null | grep -q "^/scan$" && break
done
echo "[ok] /scan 就绪，等 12s 让 slam 稳定"
sleep 12

# 机器人朝一个方向走一段，产生多样化的观测
timeout 12 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}" > /dev/null 2>&1 &
sleep 14
pkill -f "[r]os2 topic pub" 2>/dev/null
sleep 1

echo "=== /scan 统计 ==="
python3 tools/scan_stats.py --topic /scan --n 40 2>&1 | tail -30

echo ""
echo "=== /points 点数 ==="
timeout 8 ros2 topic hz /points 2>&1 | head -4

kill -INT $LPID 2>/dev/null
sleep 3
pkill -9 -f "[g]zserver" 2>/dev/null
echo "[done] /tmp/$TAG.log"
