from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly", package_name="simple_assembly_moveit_config"
        )
        .robot_description(file_path="config/simple_assembly_canhat.urdf.xacro")
        .joint_limits(file_path="config/joint_limits_canhat.yaml")
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
    ld.add_action(DeclareLaunchArgument("local_robot_state_publisher", default_value="true"))

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ubuntu_robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
            condition=IfCondition(LaunchConfiguration("local_robot_state_publisher")),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
    )

    return ld
