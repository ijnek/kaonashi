# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
# Copyright 2022 Kenji Brameld <kenjibrameld@gmail.com>
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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

    # Rviz
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('kaonashi_bringup'), 'rviz', 'demo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        description_launch,
        control_launch,
        rviz_node,
    ])
