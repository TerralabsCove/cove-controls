from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "simple_assembly", package_name="simple_assembly_moveit_config"
    ).to_moveit_configs()

    pkg = moveit_config.package_path
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg / "launch/static_virtual_joint_tfs.launch.py")
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/rsp.launch.py"))
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/move_group.launch.py"))
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(pkg / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg / "launch/spawn_controllers.launch.py")
            )
        )
    )

    return ld
