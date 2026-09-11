#!/usr/bin/env bash
# 端到端测试 v2 —— 用「直接订阅」做就绪判据，不依赖 ros2 daemon
# 用法：bash e2e2.sh [world] [sensor_profile]
WORLD="${1:-simple}"
PROF="${2:-lidar3d_imu_cam}"
OUT="/tmp/slam_nav_sim_e2e2_${WORLD}.txt"
exec > "$OUT" 2>&1

source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1
source install/setup.bash

# 关键：先把 daemon 弄干净，否则所有 *list 命令都会撒谎
timeout -k 3 20 ros2 daemon stop >/dev/null 2>&1
sleep 2
timeout -k 3 20 ros2 daemon start >/dev/null 2>&1
sleep 3
rm -f /dev/shm/fastrtps_* 2>/dev/null
pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[r]obot_state_publisher' 2>/dev/null
sleep 2

LOG=/tmp/e2e2_${WORLD}.log
echo "########## 启动仿真：world=$WORLD profile=$PROF 无头 ##########"
setsid ros2 launch sim_bringup sim.launch.py \
  world:=$WORLD headless:=true mapping:=none navigation:=none \
  viz:=false metrics:=false check:=false sensor_profile:=$PROF \
  > "$LOG" 2>&1 &
LPID=$!

cleanup() {
  kill -9 -- -"$LPID" 2>/dev/null
  pkill -9 -f '[g]zserver' 2>/dev/null
  pkill -9 -f '[g]zclient' 2>/dev/null
  pkill -9 -f '[r]obot_state_publisher' 2>/dev/null
  pkill -9 -f '[s]pawn_entity' 2>/dev/null
  pkill -9 -f '[s]im_joint_state_bridge' 2>/dev/null
  pkill -9 -f '[g]imbal_stabilizer' 2>/dev/null
  sleep 2
}
trap cleanup EXIT

# ---- 就绪判据：直接订阅 /clock（不走 daemon，最可靠）----
echo "  >> 等待 /clock（直接订阅 30s 超时）…"
READY=0
for i in $(seq 1 5); do
  if timeout -k 3 30 ros2 topic echo /clock --once >/dev/null 2>&1; then READY=1; break; fi
  echo "     第 $i 次未就绪，重试"
done
if [ $READY -ne 1 ]; then
  echo "  !! /clock 未上线"; tail -30 "$LOG" | sed 's/^/    /'; exit 1
fi
echo "  /clock 已上线"

echo "  >> 给 spawn_entity / 传感器留初始化时间（20s）…"
sleep 20

echo
echo "########## 1. 话题清单（daemon 已修复）##########"
timeout -k 3 20 ros2 topic list 2>/dev/null | sort | sed 's/^/  /'

echo
echo "########## 2. 节点与服务 ##########"
echo "  node list   : $(timeout -k 3 20 ros2 node list --no-daemon 2>/dev/null | tr '\n' ' ')"
echo "  gazebo 服务数: $(timeout -k 3 20 ros2 service list 2>/dev/null | grep -c '/gazebo/')"
echo "  robot 相关服务:"
timeout -k 3 20 ros2 service list 2>/dev/null | grep -iE 'robot|controller' | head -5 | sed 's/^/    /'

echo
echo "########## 3. 各传感器是否真的有数据（直接订阅）##########"
check() {
  local t="$1" desc="$2"
  if timeout -k 3 15 ros2 topic echo "$t" --once >/dev/null 2>&1; then
    printf "  %-24s %-14s OK\n" "$t" "$desc"
  else
    printf "  %-24s %-14s FAIL\n" "$t" "$desc"
  fi
}
check /points "3D 点云"
check /imu "IMU"
check /odom "里程计"
check /ground_truth/odom "真值里程计"
check /joint_states "关节状态"
check /camera/points "深度点云"
check /camera/image_raw "RGB 图像"

echo
echo "########## 4. 数据内容抽样 ##########"
echo -n "  /points frame_id : "; timeout -k 3 15 ros2 topic echo /points --once --field header.frame_id 2>/dev/null || echo "(取不到)"
echo -n "  /points 点数     : "; timeout -k 3 15 ros2 topic echo /points --once --field width 2>/dev/null || echo "(取不到)"
echo -n "  /points 频率     : "; timeout -k 3 12 ros2 topic hz /points 2>/dev/null | head -1 || echo "(取不到)"
echo -n "  /imu 频率        : "; timeout -k 3 12 ros2 topic hz /imu 2>/dev/null | head -1 || echo "(取不到)"
echo "  --- 里程计 frame 校验（契约要求 child_frame_id=base_footprint）---"
for t in /odom /ground_truth/odom; do
  echo -n "    $t parent_frame_id: "; timeout -k 3 15 ros2 topic echo "$t" --once --field header.frame_id 2>/dev/null || echo "(取不到)"
  echo -n "    $t child_frame_id : "; timeout -k 3 15 ros2 topic echo "$t" --once --field child_frame_id 2>/dev/null || echo "(取不到)"
done
echo "  --- IMU 协方差是否非零（LIO 关键前提）---"
timeout -k 3 15 ros2 topic echo /imu --once --field orientation_covariance 2>/dev/null | head -8 | sed 's/^/    /'
echo "  --- joint_states 关节数 ---"
timeout -k 3 15 ros2 topic echo /joint_states --once 2>/dev/null | grep -c '^  - ' | sed 's/^/    条目数 /'

echo
echo "########## 5. TF 树 ##########"
for pair in "odom base_footprint" "base_footprint base_link" "base_link lidar_stabilized" "base_link lidar_link" "base_link imu_link" "base_link camera_link"; do
  set -- $pair
  if timeout -k 3 12 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q "Translation"; then
    printf "  %-14s -> %-20s OK\n" "$1" "$2"
  else
    printf "  %-14s -> %-20s 缺失\n" "$1" "$2"
  fi
done

echo
echo "########## 6. 仿真/机器人实体是否真的存在 ##########"
timeout -k 3 20 ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList '{}' 2>/dev/null \
  | grep -A12 'name:' | head -14 | sed 's/^/  /' || echo "  (调用失败)"

echo
echo "########## 7. 日志中的报错 ##########"
grep -inE "error|fail|exception|cannot|unable|timeout" "$LOG" 2>/dev/null \
  | grep -viE "verbose|deprecat|EOL|end-of-life" | head -12 | sed 's/^/  /'
echo "  (以上为全部匹配)"

echo
echo "########## 8. 日志尾部 ##########"
tail -15 "$LOG" | sed 's/^/  /'

echo
echo "### 结束 ###"
