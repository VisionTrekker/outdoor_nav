#!/usr/bin/env bash
# Launch Go2W full system: go2w_driver + MAVROS + Nav2 + auto gp_origin

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/setup_go2w.sh"
source install/setup.bash

ros2 launch robot_launch outdoor_all_go2w.launch.py "$@"