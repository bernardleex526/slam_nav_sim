#!/usr/bin/env bash
# 云台 / 雷达档位 / 传感器组合的最终验证
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1
source install/setup.bash

# 提取雷达插件上报的点云 frame（<frameName> 是 MID360，<frame_name> 是原生 ray）
lidar_frame() {
  xacro "$1" "sensor_profile:=$2" "use_gimbal:=$3" 2>/dev/null \
    | grep -oE '<frame(Name|_name)>[^<]*' \
    | grep -E 'lidar|stabilized' \
    | head -1 \
    | sed 's/^<[^>]*>//'
}

echo "########## 1. 稳定云台：点云 frame 是否正确切换 ##########"
printf "  %-16s %-8s %-22s %s\n" "档位" "云台" "实际 frame" "结果"
PASS=0; FAIL=0
for prof in mid360_imu_cam lidar3d_imu_cam; do
  for g in true false; do
    F=$(lidar_frame src/robot_descriptions/xacro/wheeled_quadruped/robot.urdf.xacro "$prof" "$g")
    EXP=lidar_link; [ "$g" = "true" ] && EXP=lidar_stabilized
    if [ "$F" = "$EXP" ]; then R="OK"; PASS=$((PASS+1)); else R="FAIL"; FAIL=$((FAIL+1)); fi
    printf "  %-16s %-8s %-22s %s\n" "$prof" "$g" "${F:-<空>}" "$R"
  done
done

echo
echo "########## 2. 四足平台 ##########"
for g in true false; do
  F=$(lidar_frame src/robot_descriptions/xacro/quadruped/robot.urdf.xacro mid360_imu_cam "$g")
  EXP=lidar_link; [ "$g" = "true" ] && EXP=lidar_stabilized
  if [ "$F" = "$EXP" ]; then R="OK"; PASS=$((PASS+1)); else R="FAIL"; FAIL=$((FAIL+1)); fi
  printf "  %-16s %-8s %-22s %s\n" "quadruped" "$g" "${F:-<空>}" "$R"
done

echo
echo "########## 3. 回归：全档位 x 全云台 x 两平台 ##########"
BAD=0; N=0
for prof in lidar3d_imu lidar3d_imu_cam mid360_imu mid360_imu_cam cam_only; do
  for g in true false; do
    for r in wheeled_quadruped quadruped; do
      N=$((N+1))
      xacro src/robot_descriptions/xacro/$r/robot.urdf.xacro \
        sensor_profile:=$prof use_gimbal:=$g odom_source:=gt >/dev/null 2>&1 \
        || { echo "  FAIL $r / $prof / gimbal=$g"; BAD=$((BAD+1)); }
    done
  done
done
echo "  $N 种组合，失败 $BAD"

echo
echo "########## 4. 云台节点的 frame 名与 xacro 是否一致 ##########"
X=$(lidar_frame src/robot_descriptions/xacro/wheeled_quadruped/robot.urdf.xacro mid360_imu_cam true)
N=$(grep -oP 'declare_parameter\("stabilized_frame",\s*"\K[^"]*' \
    src/robot_descriptions/scripts/gimbal_stabilizer.py)
echo "  xacro 产出 : ${X:-<空>}"
echo "  节点默认值 : ${N:-<空>}"
if [ "$X" = "$N" ] && [ -n "$X" ]; then
  echo "  一致 OK"
else
  echo "  不一致 FAIL —— 云台 TF 会缺失"
  FAIL=$((FAIL+1))
fi

echo
echo "########## 汇总：$PASS 通过 / $FAIL 失败 ##########"
