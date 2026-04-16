"""One-click launch for outdoor nav: car driver + PX4 + nav2 + auto gp_origin setup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_launch_share = get_package_share_directory("robot_launch")
    nav2_launch_file = os.path.join(robot_launch_share, "launch", "gp_goal_nx_nav2.launch.py")

    declare_ref_lat = DeclareLaunchArgument(
        "reference_latitude",
        default_value="30.8135718",
        description="ENU origin latitude for auto gp_origin set.",
    )
    declare_ref_lon = DeclareLaunchArgument(
        "reference_longitude",
        default_value="120.8338169",
        description="ENU origin longitude for auto gp_origin set.",
    )
    declare_ref_alt = DeclareLaunchArgument(
        "reference_altitude",
        default_value="12.318126322268082",
        description="ENU origin altitude for auto gp_origin set.",
    )
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

    ref_lat = LaunchConfiguration("reference_latitude")
    ref_lon = LaunchConfiguration("reference_longitude")
    ref_alt = LaunchConfiguration("reference_altitude")
    fcu_url = LaunchConfiguration("fcu_url")
    use_auto_origin = LaunchConfiguration("use_auto_origin")

    car_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros2can"), "launch", "car_driver.launch.py")
        )
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mavros"), "launch", "px4.launch")
        ),
        launch_arguments={"fcu_url": fcu_url}.items(),
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
            declare_ref_lat,
            declare_ref_lon,
            declare_ref_alt,
            declare_use_auto_origin,
            declare_fcu_url,
            car_driver_launch,
            mavros_launch,
            nav2_launch,
            auto_set_origin_node,
        ]
    )
