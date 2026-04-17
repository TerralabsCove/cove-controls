import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Run RViz only — use on your local machine while MoveIt runs on the Pi."""
    pkg_dir = get_package_share_directory('simple_turret_moveit')

    urdf_path = os.path.join(pkg_dir, 'urdf', 'simple_turret.urdf')
    srdf_path = os.path.join(pkg_dir, 'srdf', 'simple_turret.srdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    kinematics_yaml = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    ompl_yaml = os.path.join(pkg_dir, 'config', 'ompl_planning.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'moveit.rviz')

    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': ompl_yaml,
    }

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            parameters=[
                moveit_config,
                {'use_sim_time': False},
            ],
            output='screen',
        ),
    ])
