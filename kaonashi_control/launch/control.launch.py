from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    controller_arg = DeclareLaunchArgument(
        name='controller',
        description='The controller to spawn by default.',
        default_value='joint_group_position_controller')

    # Controller Manager node
    controller_config = PathJoinSubstitution(
        [FindPackageShare('kaonashi_control'), 'control', 'controllers.yaml'])
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('xacro_path')]))},
            controller_config
        ],
        output='screen',
    )

    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawn joint_group_position_controller
    condition = IfCondition(EqualsSubstitution(
        LaunchConfiguration('controller'), 'joint_group_position_controller'))
    spawn_joint_group_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
        condition=condition,
    )

    # Spawn joint_trajectory_controller
    condition = IfCondition(EqualsSubstitution(
        LaunchConfiguration('controller'), 'joint_trajectory_controller'))
    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        condition=condition,
    )

    return LaunchDescription([
        controller_arg,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_joint_group_position_controller,
        spawn_joint_trajectory_controller,
    ])
