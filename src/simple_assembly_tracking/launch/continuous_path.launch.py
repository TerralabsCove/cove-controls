from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
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
            "robot.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("orientation_tolerance", default_value="0.12"),
            DeclareLaunchArgument("constrain_orientation", default_value="true"),
            DeclareLaunchArgument("velocity_scale", default_value="0.3"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.3"),
            DeclareLaunchArgument("wait_after_home", default_value="1.0"),
            DeclareLaunchArgument("wait_before_release", default_value="1.0"),
            DeclareLaunchArgument("waypoint_file"),
            DeclareLaunchArgument("gpio_chip", default_value="gpiochip4"),
            DeclareLaunchArgument("gpio_line", default_value="17"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    "video_device": LaunchConfiguration("video_device"),
                    "enable_tracker": "false",
                }.items(),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "service", "call",
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
                executable="continuous_path.py",
                name="continuous_path",
                output="screen",
                parameters=[
                    {
                        "goal_tolerance": ParameterValue(
                            LaunchConfiguration("goal_tolerance"), value_type=float
                        ),
                        "orientation_tolerance": ParameterValue(
                            LaunchConfiguration("orientation_tolerance"),
                            value_type=float,
                        ),
                        "constrain_orientation": ParameterValue(
                            LaunchConfiguration("constrain_orientation"),
                            value_type=bool,
                        ),
                        "velocity_scale": ParameterValue(
                            LaunchConfiguration("velocity_scale"), value_type=float
                        ),
                        "acceleration_scale": ParameterValue(
                            LaunchConfiguration("acceleration_scale"), value_type=float
                        ),
                        "wait_after_home": ParameterValue(
                            LaunchConfiguration("wait_after_home"), value_type=float
                        ),
                        "wait_before_release": ParameterValue(
                            LaunchConfiguration("wait_before_release"),
                            value_type=float,
                        ),
                        "waypoint_file": LaunchConfiguration("waypoint_file"),
                        "gpio_chip": LaunchConfiguration("gpio_chip"),
                        "gpio_line": ParameterValue(
                            LaunchConfiguration("gpio_line"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
