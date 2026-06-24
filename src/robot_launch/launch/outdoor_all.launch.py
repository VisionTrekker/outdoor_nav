"""One-click launch for outdoor nav: car driver + PX4 + nav2 + auto gp_origin setup."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_ref_from_yaml(yaml_path):
    """Load reference lat/lon/alt from nav2_input_bridge params."""
    with open(yaml_path, "r") as f:
        params = yaml.safe_load(f)
    bridge = params.get("nav2_input_bridge", {}).get("ros__parameters", {})
    return (
        bridge.get("reference_latitude", 30.8135718),
        bridge.get("reference_longitude", 120.8338169),
        bridge.get("reference_altitude", 12.318126322268082),
    )


def generate_launch_description():
    robot_launch_share = get_package_share_directory("robot_launch")
    nav2_launch_file = os.path.join(robot_launch_share, "launch", "gp_goal_nx_nav2.launch.py")
    nav2_params_file = os.path.join(robot_launch_share, "config", "gp_goal_nx_nav2.yaml")

    # Load reference from YAML so only one file needs editing per site
    ref_lat, ref_lon, ref_alt = load_ref_from_yaml(nav2_params_file)

    declare_use_auto_origin = DeclareLaunchArgument(
        "use_auto_origin",
        default_value="true",
        description="Whether to run the auto_set_gp_origin script on startup.",
    )
    declare_fcu_url = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:230400",
        description="MAVROS FCU URL.",
    )

    fcu_url = LaunchConfiguration("fcu_url")
    use_auto_origin = LaunchConfiguration("use_auto_origin")

    mavros_pluginlists_yaml = os.path.join(robot_launch_share, "config", "mavros_pluginlists.yaml")

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_share, "launch", "mavros_px4.launch.py")
        ),
        launch_arguments={
            "fcu_url": fcu_url,
            "pluginlists_yaml": mavros_pluginlists_yaml,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file)
    )

    auto_set_origin_node = Node(
        package="robot_launch",
        executable="auto_set_gp_origin",
        name="auto_set_gp_origin",
        output="screen",
        parameters=[
            {
                "latitude": ref_lat,
                "longitude": ref_lon,
                "altitude": ref_alt,
                "startup_delay_s": 10.0,
                "verify_tolerance_deg": 1e-6,
                "max_retries": 3,
                "verify_timeout_s": 5.0,
            }
        ],
        condition=IfCondition(use_auto_origin),
    )

    return LaunchDescription(
        [
            declare_use_auto_origin,
            declare_fcu_url,
            mavros_launch,
            nav2_launch,
            auto_set_origin_node,
        ]
    )
