from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly_tracking",
            package_name="simple_assembly_tracking_moveit_config",
        )
        .robot_description(file_path="config/simple_assembly_tracking.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_assembly_tracking.srdf")
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_robot.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            Node(
                package="rviz2",
                executable="rviz2",
                name="apriltag_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
