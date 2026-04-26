from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("calibrate_on_start", default_value="false"))
    ld.add_action(
        DeclareLaunchArgument(
            "zero_offsets_file",
            default_value="/home/terralabscove/.ros/damiao_zero_offsets.yaml",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly", package_name="simple_assembly_moveit_config"
        )
        .robot_description(
            file_path="config/simple_assembly.urdf.xacro",
            mappings={
                "calibrate_on_start": LaunchConfiguration("calibrate_on_start"),
                "zero_offsets_file": LaunchConfiguration("zero_offsets_file"),
            },
        )
        .to_moveit_configs()
    )

    pkg = moveit_config.package_path

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
