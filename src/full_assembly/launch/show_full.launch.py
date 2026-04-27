from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("full_assembly"))
    urdf_path = pkg_share / "urdf" / "simple_assembly.urdf"
    rviz_config = pkg_share / "rviz" / "show_full.rviz"

    robot_description = {"robot_description": urdf_path.read_text()}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_root",
            arguments=["--frame-id", "map", "--child-frame-id", "root"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        ),
    ])
