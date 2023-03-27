# Copyright 2023 Kenji Brameld <kenjibrameld@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    bringup_params_arg = DeclareLaunchArgument(
        name='bringup_params',
        description='Path to the yaml file containing parameters for this launch file.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('kaonashi_bringup'), 'configs', 'demo_params.yaml']))

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

    # Camera
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[LaunchConfiguration('bringup_params')],
    )

    # IPM Image Node
    ipm_image_node = Node(
        package='ipm_image_node',
        executable='ipm',
        name='ipm',
        output='screen',
        remappings=[
            ('input', 'image_raw'),
        ],
        parameters=[LaunchConfiguration('bringup_params')],
    )

    # Floor to base_link transform
    static_transform_publisher_node = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "1.25", "0", "0", "0", "floor", "base_link"])

    # Rviz
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('kaonashi_bringup'), 'rviz', 'demo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_path,
        ],
    )

    return LaunchDescription([
        bringup_params_arg,
        description_launch,
        control_launch,
        usb_cam_node,
        ipm_image_node,
        static_transform_publisher_node,
        rviz_node,
    ])
