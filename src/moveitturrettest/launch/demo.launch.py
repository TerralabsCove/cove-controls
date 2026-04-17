import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Unset CYCLONEDDS_URI so local DDS discovery works via multicast
    # (custom CycloneDDS config breaks same-machine node discovery)
    os.environ.pop("CYCLONEDDS_URI", None)

    moveit_config = MoveItConfigsBuilder(
        "simple_turret", package_name="moveitturrettest"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
