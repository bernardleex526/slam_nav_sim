#!/usr/bin/env bash
# 最终验收测试
# 用法：bash accept.sh <world> <profile> <gimbal> <期望的 lidar frame>
OUT="/tmp/slam_nav_sim_accept.txt"
WORLD="${1:-complex}"; PROF="${2:-lidar3d_imu_cam}"; GIM="${3:-true}"; EXPECT="${4:-lidar_stabilized}"
: > "$OUT"
exec >> "$OUT" 2>&1

source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"; source install/setup.bash

timeout -k 3 20 ros2 daemon stop  >/dev/null 2>&1; sleep 2
timeout -k 3 20 ros2 daemon start >/dev/null 2>&1; sleep 2
rm -f /dev/shm/fastrtps_* 2>/dev/null
pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[r]obot_state_publisher' 2>/dev/null; sleep 2

LOG=/tmp/accept_${WORLD}_${GIM}.log
echo "########## world=$WORLD profile=$PROF gimbal=$GIM ##########"
setsid ros2 launch sim_bringup sim.launch.py \
  world:=$WORLD headless:=true mapping:=none navigation:=none \
  viz:=false metrics:=false check:=false \
  sensor_profile:=$PROF use_gimbal:=$GIM > "$LOG" 2>&1 &
LPID=$!
cleanup() {
  kill -9 -- -"$LPID" 2>/dev/null
  pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[r]obot_state_publisher' 2>/dev/null
  pkill -9 -f '[s]pawn_entity' 2>/dev/null; pkill -9 -f '[g]imbal_stabilizer' 2>/dev/null
  pkill -9 -f '[s]im_joint_state_bridge' 2>/dev/null
  sleep 2
}
trap cleanup EXIT

for i in $(seq 1 6); do
  timeout -k 3 30 ros2 topic echo /clock --once >/dev/null 2>&1 && break
done
sleep 22

echo
echo "--- 核心话题 ---"
for t in /points /imu /odom /ground_truth/odom /joint_states /tf /camera/points; do
  if timeout -k 3 12 ros2 topic echo "$t" --once >/dev/null 2>&1; then echo "  OK   $t"; else echo "  FAIL $t"; fi
done

echo
echo "--- /points 关键指标 ---"
echo -n "  frame_id  : "; timeout -k 3 12 ros2 topic echo /points --once --field header.frame_id 2>/dev/null
echo -n "  点数      : "; timeout -k 3 12 ros2 topic echo /points --once --field width 2>/dev/null
echo -n "  频率      : "; timeout -k 3 12 ros2 topic hz /points 2>/dev/null | head -1
echo "  期望 frame: $EXPECT"

echo
echo "--- IMU 三类协方差（LIO 真正消费的是陀螺与加速度）---"
echo "  [orientation]"; timeout -k 3 12 ros2 topic echo /imu --once --field orientation_covariance 2>/dev/null | sed 's/^/    /'
echo "  [angular_velocity]"; timeout -k 3 12 ros2 topic echo /imu --once --field angular_velocity_covariance 2>/dev/null | sed 's/^/    /'
echo "  [linear_acceleration]"; timeout -k 3 12 ros2 topic echo /imu --once --field linear_acceleration_covariance 2>/dev/null | sed 's/^/    /'
echo -n "  imu 频率: "; timeout -k 3 12 ros2 topic hz /imu 2>/dev/null | head -1

echo
echo "--- joint_states ---"
echo -n "  关节数: "; timeout -k 3 12 ros2 topic echo /joint_states --once --field name 2>/dev/null | grep -c '^- '
timeout -k 3 12 ros2 topic echo /joint_states --once --field name 2>/dev/null | head -4 | sed 's/^/    /'

echo
echo "--- TF 链路 ---"
for pair in "odom base_footprint" "base_footprint base_link" "base_link $EXPECT" "base_link imu_link" "base_link camera_link"; do
  set -- $pair
  if timeout -k 3 12 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q Translation; then
    echo "  OK   $1 -> $2"
  else
    echo "  FAIL $1 -> $2"
  fi
done

echo
echo "--- 日志中的 ERROR（排除已知无害项）---"
grep -E "\[ERROR\]" "$LOG" 2>/dev/null | grep -viE "does not exist|EOL" | head -8 | sed 's/^/  /'
echo "  (以上为全部)"

echo
echo "--- 已知无害告警 ---"
grep -E "does not exist" "$LOG" 2>/dev/null | head -3 | sed 's/^/  /'

echo
echo "--- 地图内容校验（GT 栅格与 meta 一致性）---"
M=$WS/install/sim_worlds/share/sim_worlds/maps
python3 - "$M/${WORLD}.yaml" "$M/${WORLD}_meta.json" <<'PY'
import json, sys, yaml, os
my, mj = sys.argv[1], sys.argv[2]
m = yaml.safe_load(open(my))
j = json.load(open(mj))
print(f"  网格 {j['grid_nx']}x{j['grid_ny']}  分辨率 {m['resolution']}  "
      f"origin {m['origin']}  可通行 {j['free_area_ratio']*100:.1f}%  连通 {j['connectivity_from_spawn']*100:.1f}%")
ok = (j['resolution'] == m['resolution'] and
      abs(j['origin'][0]-m['origin'][0]) < 1e-6 and
      abs(j['origin'][1]-m['origin'][1]) < 1e-6)
print(f"  yaml 与 meta 一致: {'OK' if ok else 'FAIL'}")
PY

echo
echo "########## 结束 ##########"
