#!/usr/bin/env bash
# Ubuntu 22.04 with ROS 2 Humble already installed. No mirrors or shell profiles are modified.
set -eo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config git \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-pil python3-matplotlib python3-transforms3d \
  gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner ros-humble-nav2-mppi-controller \
  ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-slam-toolbox \
  ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-tf2-tools ros-humble-tf2-sensor-msgs ros-humble-tf-transformations \
  ros-humble-rviz2
echo 'Core dependencies installed. Run: bash build.sh'
echo 'FAST-LIO, LIO-SAM, RTAB-Map and Livox plugins are not installed by this script.'
