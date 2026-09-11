#!/usr/bin/env bash
# WSL 内环境探测：确认 ROS2 / Gazebo / Nav2 / 依赖是否就绪
# 用法：bash /mnt/c/.../probe_env.sh
echo "########## 系统 ##########"
lsb_release -ds 2>/dev/null || cat /etc/os-release | head -2
echo "kernel : $(uname -r)"
echo "WSL    : $(grep -qi microsoft /proc/version && echo yes || echo no)"
echo "arch   : $(dpkg --print-architecture)"
echo "CPU    : $(nproc) 核"
echo "MEM    : $(free -h | awk '/^Mem:/{print $2}')"

echo
echo "########## 图形 / 代理 ##########"
echo "DISPLAY          = ${DISPLAY:-<空>}"
echo "WAYLAND_DISPLAY  = ${WAYLAND_DISPLAY:-<空>}"
echo "http_proxy       = ${http_proxy:-<空>}"
echo "https_proxy      = ${https_proxy:-<空>}"
ls -d /mnt/wslg 2>/dev/null && echo "WSLg 挂载: 存在" || echo "WSLg 挂载: 不存在"
ldconfig -p 2>/dev/null | grep -c "libGL.so\|libEGL.so" | xargs -I{} echo "GL 库数量: {}"

echo
echo "########## ROS2 ##########"
if [ -d /opt/ros ]; then
  echo "已安装发行版: $(ls /opt/ros | tr '\n' ' ')"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null && echo "humble source: OK" || echo "humble source: 失败"
  echo "ROS_DISTRO   = ${ROS_DISTRO:-<空>}"
  echo "ros2 命令    : $(command -v ros2 || echo 未找到)"
  echo "RMW          = ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp(默认)}"
else
  echo "!! /opt/ros 不存在，ROS2 未安装"
fi

echo
echo "########## 关键 ROS 包 ##########"
for p in gazebo_ros gazebo_plugins xacro robot_state_publisher rviz2 \
         nav2_bringup nav2_smac_planner nav2_mppi_controller \
         slam_toolbox pointcloud_to_laserscan rtabmap_ros tf2_sensor_msgs \
         livox_laser_simulation; do
  if [ -d "/opt/ros/humble/share/$p" ]; then
    echo "  OK   $p"
  else
    echo "  --   $p  (缺)"
  fi
done

echo
echo "########## Gazebo ##########"
for c in gazebo gzserver gzclient gz ign; do
  printf "  %-10s %s\n" "$c" "$(command -v $c || echo 未找到)"
done
gazebo --version 2>/dev/null | head -1

echo
echo "########## Python ##########"
echo "python3      : $(python3 -V 2>&1)"
echo "colcon       : $(command -v colcon || echo 未找到)"
for m in numpy yaml evo transforms3d; do
  python3 -c "import $m; print('  OK   $m', getattr($m,'__version__',''))" 2>/dev/null \
    || echo "  --   $m (缺)"
done

echo
echo "########## 工作空间 ##########"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$WS" ]; then
  echo "sim_ws: 可见"
  echo "包数量: $(ls -d $WS/src/*/ 2>/dev/null | wc -l)"
  ls $WS/src 2>/dev/null | tr '\n' ' '
  echo
  echo "已有构建产物: $(ls -d $WS/build $WS/install 2>/dev/null | wc -l) / 2"
else
  echo "!! 看不到 $WS"
fi

echo
echo "########## 磁盘 ##########"
df -h / /mnt/c 2>/dev/null | tail -3
echo
echo "### 探测结束 ###"
