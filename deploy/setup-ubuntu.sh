#!/usr/bin/env bash
# Native Ubuntu 24.04 setup for a dedicated validation VM.
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo "Requires Ubuntu 24.04" >&2
  exit 3
fi
plugin_root="$(cd "$(dirname "$0")/.." && pwd)"
sudo apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update
sudo env DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 install -y --no-install-recommends \
  ca-certificates curl locales software-properties-common
sudo locale-gen en_US.UTF-8
sudo add-apt-repository -y universe
ros_apt_package="$(mktemp --suffix=.deb)"
trap 'rm -f "$ros_apt_package"' EXIT
curl -fL --retry 3 --connect-timeout 20 --max-time 120 \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb \
  -o "$ros_apt_package"
printf '%s  %s\n' f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296 "$ros_apt_package" | sha256sum -c -
sudo dpkg -i "$ros_apt_package"
sudo apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update
sudo env DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 install -y --no-install-recommends \
  ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-tf2-ros ros-jazzy-rmw-cyclonedds-cpp python3-venv python3-dev build-essential cmake \
  libgl1-mesa-dri libegl1 mesa-utils
set +u
source /opt/ros/jazzy/setup.bash
set -u
cmake -S "$plugin_root/deploy/contact_monitor" -B "$plugin_root/build/contact_monitor"
cmake --build "$plugin_root/build/contact_monitor" -j2
sudo mkdir -p /opt/validation-venv
sudo chown "$(id -u):$(id -g)" /opt/validation-venv
python3 -m venv --system-site-packages /opt/validation-venv
/opt/validation-venv/bin/pip install 'setuptools>=68' wheel
/opt/validation-venv/bin/pip install --no-build-isolation "$plugin_root/third_party/breezyslam/python"
/opt/validation-venv/bin/pip install -e "$plugin_root"
mkdir -p "$plugin_root/results/ros2-environment"
/opt/validation-venv/bin/pip freeze > "$plugin_root/results/ros2-environment/python-resolved.txt"
dpkg-query -W > "$plugin_root/results/ros2-environment/apt-resolved.txt"
