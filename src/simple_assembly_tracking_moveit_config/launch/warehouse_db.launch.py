from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WAREHOUSE_PLUGIN = "warehouse_ros_sqlite::DatabaseConnection"
WAREHOUSE_DB_FILENAME = "simple_assembly_tracking_warehouse.sqlite"


def generate_launch_description():
    warehouse_params = {
        "warehouse_plugin": WAREHOUSE_PLUGIN,
        "warehouse_host": LaunchConfiguration("moveit_warehouse_database_path"),
        "warehouse_port": ParameterValue(
            LaunchConfiguration("moveit_warehouse_port"), value_type=int
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "moveit_warehouse_database_path",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), ".ros", WAREHOUSE_DB_FILENAME]
                ),
            ),
            DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"),
            Node(
                package="moveit_ros_warehouse",
                executable="moveit_warehouse_services",
                name="moveit_warehouse_services",
                output="screen",
                parameters=[warehouse_params],
            ),
        ]
    )
