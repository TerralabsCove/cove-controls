import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("canhat_one_motor")
    xacro_file = os.path.join(pkg_dir, "urdf", "one_motor_bot.urdf.xacro")
    rviz_config = os.path.join(pkg_dir, "config", "one_motor.rviz")

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="canhat_one_motor_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"publish_frequency": 60.0},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
