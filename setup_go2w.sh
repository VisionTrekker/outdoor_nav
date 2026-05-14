#!/usr/bin/env bash
# Go2W DDS environment setup for NX
# Source this before running launch_go2w.sh or launch_all_go2w.sh

set -eo pipefail

if [ -z "${ROS_DISTRO}" ]; then
    source /opt/ros/humble/setup.bash
fi

# Unitree ROS2 cyclonedds workspace (contains unitree_api messages)
source "${HOME}/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash"

# CycloneDDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check GO2W_NET_IFACE
if [ -z "${GO2W_NET_IFACE}" ]; then
    echo "[setup_go2w] ERROR: GO2W_NET_IFACE is not set"
    echo "  Please set GO2W_NET_IFACE to your USB network interface (e.g., enx0826ae32db83)"
    echo "  Example: export GO2W_NET_IFACE=enx0826ae32db83"
    return 1 2>/dev/null || exit 1
fi

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${GO2W_NET_IFACE}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

echo "[setup_go2w] Environment ready"
echo "  GO2W_NET_IFACE=${GO2W_NET_IFACE}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"