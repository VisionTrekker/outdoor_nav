from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2can",
                executable="car_driver",
                name="car_driver",
                output="screen",
            ),
        ]
    )
