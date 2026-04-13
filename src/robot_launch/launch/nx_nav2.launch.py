"""Nav2 bringup with Livox MID360 driver (same stack as nav2.launch.py + msg_MID360)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_params = os.path.join(pkg_share, "config", "nav2.yaml")
    default_rviz = os.path.join(pkg_share, "config", "nav2.rviz")

    livox_share = get_package_share_directory("livox_ros_driver2")
    mid360_launch_file = os.path.join(livox_share, "launch", "msg_MID360_launch.py")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Override use_sim_time for launched nodes.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2-style multi-node YAML.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="RViz2 config path.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2.",
    )
    declare_use_mid360 = DeclareLaunchArgument(
        "use_mid360",
        default_value="true",
        description="Launch Livox MID360 driver (msg_MID360_launch from livox_ros_driver2).",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mid360 = LaunchConfiguration("use_mid360")

    sim_time_param = {"use_sim_time": use_sim_time}

    mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch_file),
        condition=IfCondition(use_mid360),
    )

    # base_link -> lidar (x y z qx qy qz qw parent child); edit values here
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "livox_frame"],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
    )

    straight_planner_node = Node(
        package="nav2_straight_planner",
        executable="straight_planner_node",
        name="straight_planner",
        output="screen",
        parameters=[params_file, sim_time_param],
        remappings=[("goal_pose", "/goal_pose")],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
        output="screen",
    )

    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_rviz_config,
            declare_use_rviz,
            declare_use_mid360,
            mid360_driver,
            static_tf_base_lidar,
            static_tf_map_odom,
            controller_server_node,
            straight_planner_node,
            lc_manager_node,
            rviz_node,
        ]
    )
