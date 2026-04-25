from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "usb_cam.params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("framerate", default_value="30.0"),
            DeclareLaunchArgument("pixel_format", default_value="mjpeg2rgb"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera"),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace="camera",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "pixel_format": LaunchConfiguration("pixel_format"),
                        "camera_name": LaunchConfiguration("camera_name"),
                        "frame_id": LaunchConfiguration("camera_frame_id"),
                    },
                ],
            ),
        ]
    )
