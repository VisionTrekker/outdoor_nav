#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source Unitree workspace for unitree_go package (required by go2w_driver)
source "$HOME/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash"

colcon build --symlink-install --packages-skip fast_lio --cmake-args -DBUILD_TESTING=OFF "$@"
