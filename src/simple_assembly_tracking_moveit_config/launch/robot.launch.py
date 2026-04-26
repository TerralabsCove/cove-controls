from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("video_device", default_value="/dev/video0"))
    ld.add_action(DeclareLaunchArgument("camera_frame_id", default_value="camera_optical_frame"))
    ld.add_action(DeclareLaunchArgument("tag_size", default_value="0.162"))
    ld.add_action(DeclareLaunchArgument("tag_id", default_value="0"))
    ld.add_action(DeclareLaunchArgument("kp_pan", default_value="0.0015"))
    ld.add_action(DeclareLaunchArgument("kp_tilt", default_value="0.0015"))
    ld.add_action(DeclareLaunchArgument("pan_sign", default_value="-1.0"))
    ld.add_action(DeclareLaunchArgument("tilt_sign", default_value="-1.0"))
    ld.add_action(DeclareLaunchArgument("deadband_px", default_value="25.0"))
    ld.add_action(DeclareLaunchArgument("trajectory_duration", default_value="0.15"))
    ld.add_action(DeclareLaunchArgument("enable_tracker", default_value="false"))
    ld.add_action(DeclareLaunchArgument("calibrate_on_start", default_value="false"))
    ld.add_action(
        DeclareLaunchArgument(
            "zero_offsets_file",
            default_value="/home/terralabscove/.ros/damiao_zero_offsets.yaml",
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly_tracking",
            package_name="simple_assembly_tracking_moveit_config",
        )
        .robot_description(
            file_path="config/simple_assembly_tracking.urdf.xacro",
            mappings={
                "calibrate_on_start": LaunchConfiguration("calibrate_on_start"),
                "zero_offsets_file": LaunchConfiguration("zero_offsets_file"),
            },
        )
        .robot_description_semantic(file_path="config/simple_assembly_tracking.srdf")
        .to_moveit_configs()
    )
    pkg = moveit_config.package_path

    # --- Robot stack ---
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/static_virtual_joint_tfs.launch.py"))
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/rsp.launch.py"))
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/move_group.launch.py"))
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(pkg / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg / "launch/spawn_controllers.launch.py"))
        )
    )

    # --- Camera ---
    camera_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "usb_cam.params.yaml"]
    )
    ld.add_action(
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
        )
    )

    # --- AprilTag detector ---
    tag_params = PathJoinSubstitution(
        [FindPackageShare("cove_vision"), "config", "apriltag_36h11.yaml"]
    )
    ld.add_action(
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
        )
    )
    ld.add_action(
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
        )
    )

    # --- Tag tracker ---
    ld.add_action(
        Node(
            package="simple_assembly_tracking",
            executable="tag_tracker.py",
            name="tag_tracker",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_tracker")),
            parameters=[
                {
                    "tag_id": ParameterValue(
                        LaunchConfiguration("tag_id"), value_type=int
                    ),
                    "kp_pan": ParameterValue(
                        LaunchConfiguration("kp_pan"), value_type=float
                    ),
                    "kp_tilt": ParameterValue(
                        LaunchConfiguration("kp_tilt"), value_type=float
                    ),
                    "pan_sign": ParameterValue(
                        LaunchConfiguration("pan_sign"), value_type=float
                    ),
                    "tilt_sign": ParameterValue(
                        LaunchConfiguration("tilt_sign"), value_type=float
                    ),
                    "deadband_px": ParameterValue(
                        LaunchConfiguration("deadband_px"), value_type=float
                    ),
                    "trajectory_duration": ParameterValue(
                        LaunchConfiguration("trajectory_duration"), value_type=float
                    ),
                }
            ],
        )
    )

    return ld
