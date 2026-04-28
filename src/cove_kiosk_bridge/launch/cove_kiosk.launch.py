from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cove_kiosk_bridge",
                executable="kiosk_bridge",
                output="screen",
            )
        ]
    )
