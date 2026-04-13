from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2can",
                executable="control_subscriber",
                name="control_subscriber",
                output="screen",
            ),
        ]
    )
