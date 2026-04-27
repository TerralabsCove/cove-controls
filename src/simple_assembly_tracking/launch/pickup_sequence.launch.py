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
            DeclareLaunchArgument("velocity_scale", default_value="0.25"),
            DeclareLaunchArgument("scan_step_deg", default_value="30.0"),
            DeclareLaunchArgument("scan_max_deg", default_value="180.0"),
            DeclareLaunchArgument("scan_dwell", default_value="0.8"),
            DeclareLaunchArgument("approach_above", default_value="0.08"),
            DeclareLaunchArgument("descend_offset", default_value="0.02"),
            DeclareLaunchArgument("lift_height", default_value="0.10"),
            DeclareLaunchArgument("magnet_dwell", default_value="1.0"),
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
                executable="pickup_sequence.py",
                name="pickup_sequence",
                output="screen",
                parameters=[
                    {
                        "velocity_scale": ParameterValue(
                            LaunchConfiguration("velocity_scale"), value_type=float
                        ),
                        "scan_step_deg": ParameterValue(
                            LaunchConfiguration("scan_step_deg"), value_type=float
                        ),
                        "scan_max_deg": ParameterValue(
                            LaunchConfiguration("scan_max_deg"), value_type=float
                        ),
                        "scan_dwell": ParameterValue(
                            LaunchConfiguration("scan_dwell"), value_type=float
                        ),
                        "approach_above": ParameterValue(
                            LaunchConfiguration("approach_above"), value_type=float
                        ),
                        "descend_offset": ParameterValue(
                            LaunchConfiguration("descend_offset"), value_type=float
                        ),
                        "lift_height": ParameterValue(
                            LaunchConfiguration("lift_height"), value_type=float
                        ),
                        "magnet_dwell": ParameterValue(
                            LaunchConfiguration("magnet_dwell"), value_type=float
                        ),
                        "gpio_chip": LaunchConfiguration("gpio_chip"),
                        "gpio_line": ParameterValue(
                            LaunchConfiguration("gpio_line"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
