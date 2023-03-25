from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Description Launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('kaonashi_description'), 'launch', 'description.launch.py']
            )
        ),
        launch_arguments={
            'xacro_path': PathJoinSubstitution(
                [FindPackageShare('kaonashi_control'), 'urdf', 'kaonashi.urdf.xacro'])
        }.items()
    )

    # Control Launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('kaonashi_control'), 'launch', 'control.launch.py']
            )
        )
    )

    return LaunchDescription([
        description_launch,
        control_launch,
    ])
