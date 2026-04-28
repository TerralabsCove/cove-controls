from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


WAREHOUSE_PLUGIN = "warehouse_ros_sqlite::DatabaseConnection"
WAREHOUSE_DB_FILENAME = "simple_assembly_tracking_warehouse.sqlite"


def generate_moveit_rviz_launch(moveit_config):
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), ".ros", WAREHOUSE_DB_FILENAME]
            ),
        )
    )
    ld.add_action(DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"))

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {
            "warehouse_plugin": WAREHOUSE_PLUGIN,
            "warehouse_host": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": ParameterValue(
                LaunchConfiguration("moveit_warehouse_port"), value_type=int
            ),
        },
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly_tracking",
            package_name="simple_assembly_tracking_moveit_config",
        )
        .robot_description(file_path="config/simple_assembly_tracking_canhat.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_assembly_tracking.srdf")
        .joint_limits(file_path="config/joint_limits_canhat.yaml")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
