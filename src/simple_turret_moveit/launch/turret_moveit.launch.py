import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_turret_moveit')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                      description='Launch RViz (set false for headless/SSH)')

    # Load files
    urdf_path = os.path.join(pkg_dir, 'urdf', 'simple_turret.urdf')
    srdf_path = os.path.join(pkg_dir, 'srdf', 'simple_turret.srdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    kinematics_yaml = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    ompl_yaml = os.path.join(pkg_dir, 'config', 'ompl_planning.yaml')
    joint_limits_yaml = os.path.join(pkg_dir, 'config', 'joint_limits.yaml')
    controllers_yaml = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(pkg_dir, 'config', 'moveit_controllers.yaml')

    # MoveIt parameters
    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': ompl_yaml,
    }

    # ---- ros2_control controller manager (mock hardware) ----
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # ---- Spawn controllers ----
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_tc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['turret_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ---- Robot state publisher ----
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ---- MoveIt move_group ----
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config,
            moveit_controllers_yaml,
            joint_limits_yaml,
            {'use_sim_time': False},
            {'planning_scene_monitor_options': {
                'robot_description': 'robot_description',
                'joint_state_topic': '/joint_states',
            }},
        ],
    )

    # ---- RViz with MoveIt plugin (optional) ----
    rviz_config_path = os.path.join(pkg_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[
            moveit_config,
            {'use_sim_time': False},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        ros2_control_node,
        robot_state_pub,
        spawn_jsb,
        spawn_tc,
        move_group_node,
        rviz_node,
    ])
