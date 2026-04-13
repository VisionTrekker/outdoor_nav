from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2can",
                executable="control_publisher",
                name="control_publisher",
                output="screen",
            ),
        ]
    )
