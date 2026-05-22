#!/usr/bin/env bash
# Launch MAVROS only (for standalone PX4 testing)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

PLUGINLISTS_YAML="${SCRIPT_DIR}/install/robot_launch/share/robot_launch/config/mavros_pluginlists.yaml"

ros2 launch robot_launch mavros_px4.launch.py \
    fcu_url:="/dev/ttyACM0:115200" \
    pluginlists_yaml:="${PLUGINLISTS_YAML}"
