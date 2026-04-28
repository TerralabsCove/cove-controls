from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly_tracking",
            package_name="simple_assembly_tracking_moveit_config",
        )
        .robot_description(file_path="config/simple_assembly_tracking_canhat.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_assembly_tracking.srdf")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
