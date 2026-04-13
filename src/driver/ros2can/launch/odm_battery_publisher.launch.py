from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2can",
                executable="odm_battery_publisher",
                name="odm_battery_publisher",
                output="screen",
            ),
        ]
    )
