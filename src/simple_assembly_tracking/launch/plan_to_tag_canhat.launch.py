from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = PathJoinSubstitution(
        [
            FindPackageShare("simple_assembly_tracking_moveit_config"),
            "launch",
            "robot_canhat.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("tag_size", default_value="0.162"),
            DeclareLaunchArgument("tag_id", default_value="0"),
            DeclareLaunchArgument("tag_frame", default_value="tag_0"),
            DeclareLaunchArgument("approach_distance", default_value="0.0"),
            DeclareLaunchArgument("proxy_plan_step", default_value="0.0"),
            DeclareLaunchArgument("max_plan_step", default_value="0.0"),
            DeclareLaunchArgument("ik_search_samples", default_value="10"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("orient_to_tag", default_value="true"),
            DeclareLaunchArgument("orientation_tolerance", default_value="0.2"),
            DeclareLaunchArgument("target_link", default_value="wrist_link"),
            DeclareLaunchArgument("execute", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    "video_device": LaunchConfiguration("video_device"),
                    "tag_id": LaunchConfiguration("tag_id"),
                    "enable_tracker": "false",
                }.items(),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/camera/set_capture",
                            "std_srvs/srv/SetBool",
                            "{data: true}",
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="simple_assembly_tracking",
                executable="plan_to_tag.py",
                name="plan_to_tag",
                output="screen",
                parameters=[
                    {
                        "tag_frame": LaunchConfiguration("tag_frame"),
                        "approach_distance": ParameterValue(
                            LaunchConfiguration("approach_distance"), value_type=float
                        ),
                        "proxy_plan_step": ParameterValue(
                            LaunchConfiguration("proxy_plan_step"), value_type=float
                        ),
                        "max_plan_step": ParameterValue(
                            LaunchConfiguration("max_plan_step"), value_type=float
                        ),
                        "ik_search_samples": ParameterValue(
                            LaunchConfiguration("ik_search_samples"), value_type=int
                        ),
                        "tag_size": ParameterValue(
                            LaunchConfiguration("tag_size"), value_type=float
                        ),
                        "goal_tolerance": ParameterValue(
                            LaunchConfiguration("goal_tolerance"), value_type=float
                        ),
                        "orient_to_tag": ParameterValue(
                            LaunchConfiguration("orient_to_tag"), value_type=bool
                        ),
                        "orientation_tolerance": ParameterValue(
                            LaunchConfiguration("orientation_tolerance"), value_type=float
                        ),
                        "target_link": LaunchConfiguration("target_link"),
                        "execute": ParameterValue(
                            LaunchConfiguration("execute"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
