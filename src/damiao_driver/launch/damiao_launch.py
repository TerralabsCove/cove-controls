from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='921600'),
        DeclareLaunchArgument('loop_rate', default_value='100.0'),
        DeclareLaunchArgument('control_mode', default_value='velocity'),
        DeclareLaunchArgument('motors', default_value='pan:DM4340:0x02:0x12,tilt:DM10010:0x01:0x11'),

        Node(
            package='damiao_driver',
            executable='damiao_node',
            name='damiao_driver',
            output='screen',
            parameters=[{
                'serial_port':  LaunchConfiguration('serial_port'),
                'baud_rate':    LaunchConfiguration('baud_rate'),
                'loop_rate':    LaunchConfiguration('loop_rate'),
                'control_mode': LaunchConfiguration('control_mode'),
                'motors':       LaunchConfiguration('motors'),
            }],
        ),
    ])
