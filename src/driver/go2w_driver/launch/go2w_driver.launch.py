from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="go2w_driver",
                executable="go2w_driver_node",
                name="go2w_driver",
                output="screen",
                parameters=[
                    {
                        "vx_max": 1.5,
                        "vy_max": 0.6,
                        "vyaw_max": 1.0,
                        "cmd_timeout_sec": 0.5,
                        "startup_delay_sec": 2.0,
                    }
                ],
            ),
        ]
    )
