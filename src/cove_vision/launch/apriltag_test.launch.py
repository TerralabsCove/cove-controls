from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "usb_cam.params.yaml"]
    )
    tag_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_36h11.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("tag_size", default_value="0.162"),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace="camera",
                output="screen",
                parameters=[
                    camera_params,
                    {"video_device": LaunchConfiguration("video_device")},
                ],
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[
                    tag_params,
                    {"size": LaunchConfiguration("tag_size")},
                ],
                remappings=[
                    ("image_rect", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            ),
        ]
    )
