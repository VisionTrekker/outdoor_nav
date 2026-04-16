#!/usr/bin/env bash
set -eo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_launch outdoor_all.launch.py "$@"
