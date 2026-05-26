"""Custom MAVROS PX4 launch that properly supports custom pluginlists."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros_share = get_package_share_directory("mavros")
    robot_launch_share = get_package_share_directory("robot_launch")

    default_pluginlists = os.path.join(
        robot_launch_share, "config", "mavros_pluginlists.yaml"
    )
    default_config = os.path.join(mavros_share, "launch", "px4_config.yaml")

    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:230400"
    )
    declare_gcs_url = DeclareLaunchArgument(
        "gcs_url", default_value=""
    )
    declare_pluginlists_yaml = DeclareLaunchArgument(
        "pluginlists_yaml", default_value=default_pluginlists
    )
    declare_config_yaml = DeclareLaunchArgument(
        "config_yaml", default_value=default_config
    )

    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    pluginlists_yaml = LaunchConfiguration("pluginlists_yaml")
    config_yaml = LaunchConfiguration("config_yaml")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": 1},
            {"tgt_component": 1},
            {"fcu_protocol": "v2.0"},
            pluginlists_yaml,
            config_yaml,
        ],
    )

    return LaunchDescription(
        [
            declare_fcu_url,
            declare_gcs_url,
            declare_pluginlists_yaml,
            declare_config_yaml,
            mavros_node,
        ]
    )
