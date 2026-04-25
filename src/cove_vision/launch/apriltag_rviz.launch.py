from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
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
    robot_description = {
        "robot_description": moveit_config.robot_description["robot_description"]
    }
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_robot.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="cove_vision",
                executable="robot_description_topic.py",
                name="robot_description_topic",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"publish_frequency": 60.0}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="apriltag_rviz",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
