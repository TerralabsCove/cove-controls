from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
)


def _with_declared_entities(moveit_launch):
    return moveit_launch.entities


def generate_launch_description():
    bind_port = LaunchConfiguration("bind_port")
    motion_script_command = LaunchConfiguration("motion_script_command")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly",
            package_name="simple_assembly_moveit_config",
        )
        .robot_description(
            file_path="config/simple_assembly.urdf.xacro",
            mappings={"use_fake_hardware": use_fake_hardware},
        )
        .to_moveit_configs()
    )

    pkg = moveit_config.package_path
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("bind_port", default_value="8080"))
    ld.add_action(DeclareLaunchArgument("motion_script_command", default_value=""))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    for entity in _with_declared_entities(generate_static_virtual_joint_tfs_launch(moveit_config)):
        ld.add_action(entity)

    for entity in _with_declared_entities(generate_rsp_launch(moveit_config)):
        ld.add_action(entity)

    for entity in _with_declared_entities(generate_move_group_launch(moveit_config)):
        ld.add_action(entity)

    for entity in _with_declared_entities(generate_moveit_rviz_launch(moveit_config)):
        ld.add_action(entity)

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(pkg / "config" / "ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    for entity in _with_declared_entities(generate_spawn_controllers_launch(moveit_config)):
        ld.add_action(entity)

    ld.add_action(
        Node(
            package="cove_kiosk_bridge",
            executable="kiosk_bridge",
            output="screen",
            parameters=[
                {
                    "bind_port": bind_port,
                    "motion_script_command": motion_script_command,
                }
            ],
        )
    )

    return ld
