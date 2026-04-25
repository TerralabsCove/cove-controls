from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "usb_cam.params.yaml"]
    )
    tag_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_36h11.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_test.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument(
                "camera_frame_id", default_value="camera_optical_frame"
            ),
            DeclareLaunchArgument("tag_size", default_value="0.162"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="apriltag_debug_camera_tf",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "apriltag_debug_root",
                    "--child-frame-id",
                    LaunchConfiguration("camera_frame_id"),
                ],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace="camera",
                output="screen",
                parameters=[
                    camera_params,
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "frame_id": LaunchConfiguration("camera_frame_id"),
                    },
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
            Node(
                package="cove_vision",
                executable="tag_tf_marker.py",
                name="tag_tf_marker",
                output="screen",
                parameters=[
                    {
                        "source_frame": LaunchConfiguration("camera_frame_id"),
                        "tag_size": ParameterValue(
                            LaunchConfiguration("tag_size"), value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="apriltag_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
                arguments=["-d", rviz_config],
            ),
        ]
    )
