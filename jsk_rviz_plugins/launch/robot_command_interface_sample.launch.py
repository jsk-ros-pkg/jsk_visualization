# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_rviz_plugins')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config',
                                          'robot_command_interface_sample.rviz')],
            parameters=[os.path.join(share_dir, 'config',
                                     'default_robot_command.yaml')],
            output='screen',
        ),
    ])
