from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_mode",
                default_value="default",
                description="Motion pattern: default, mode1, mode2, mode3, mode4",
            ),
            Node(
                package="ros2can",
                executable="control_motion",
                name="control_motion",
                output="screen",
                parameters=[
                    {
                        "motion_mode": ParameterValue(
                            LaunchConfiguration("motion_mode"), value_type=str
                        )
                    }
                ],
            ),
        ]
    )
