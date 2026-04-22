from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "simple_assembly_tracking",
            package_name="simple_assembly_tracking_moveit_config",
        )
        .robot_description(file_path="config/simple_assembly_tracking.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_assembly_tracking.srdf")
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
