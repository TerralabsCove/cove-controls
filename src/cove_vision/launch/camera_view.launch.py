from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            Node(
                package="image_view",
                executable="image_view",
                name="camera_view",
                output="screen",
                parameters=[
                    {
                        "image": LaunchConfiguration("image_topic"),
                        "image_transport": "compressed",
                    }
                ],
            ),
        ]
    )
