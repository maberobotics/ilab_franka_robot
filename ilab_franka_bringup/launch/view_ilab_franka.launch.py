#  Copyright 2023 MABE-ROBOTICS
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    load_gripper = LaunchConfiguration('load_gripper')
    namespace = LaunchConfiguration('namespace')

    franka_xacro_file = os.path.join(get_package_share_directory('ilab_franka_description'), 'urdf',
                                     'ilab_franka.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper])

    rviz_file = os.path.join(get_package_share_directory('ilab_franka_description'), 'rviz',
                             'view_ilab_franka.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),
        DeclareLaunchArgument(
            'namespace',
            default_value='/franka',
            description='Robot Namespace'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            namespace=namespace,
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=namespace,
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
        ),
    ])
