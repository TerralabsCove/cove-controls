from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument("loop_rate", default_value="50.0"),
        DeclareLaunchArgument("control_mode", default_value="status"),
        DeclareLaunchArgument("auto_enable", default_value="false"),
        DeclareLaunchArgument("switch_mode_on_start", default_value="false"),
        DeclareLaunchArgument("disable_on_shutdown", default_value="true"),
        DeclareLaunchArgument(
            "motors",
            default_value="revolute_7_0:DM10010:0x01:0x11",
        ),
        Node(
            package="damiao_socketcan_driver",
            executable="damiao_socketcan_node",
            name="damiao_socketcan_driver",
            output="screen",
            parameters=[{
                "can_interface": LaunchConfiguration("can_interface"),
                "loop_rate": LaunchConfiguration("loop_rate"),
                "control_mode": LaunchConfiguration("control_mode"),
                "auto_enable": LaunchConfiguration("auto_enable"),
                "switch_mode_on_start": LaunchConfiguration("switch_mode_on_start"),
                "disable_on_shutdown": LaunchConfiguration("disable_on_shutdown"),
                "motors": LaunchConfiguration("motors"),
            }],
        ),
    ])
