#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
exec 9>/tmp/sim_ws_delivery.lock
flock -n 9 || { echo "Another delivery run owns the lock"; exit 1; }
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11396}"
python3 -u src/sim_benchmark/scripts/stage_a_runner.py --config src/sim_benchmark/config/delivery_stage_a.yaml
python3 -u src/sim_benchmark/scripts/stage_b_runner.py --config src/sim_benchmark/config/delivery_stage_b.yaml
python3 tools/verify_delivery.py
