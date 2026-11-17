#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
exec robot-validate "$@"
