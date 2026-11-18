# -*- coding: utf-8 -*-

# ROS 2 counterpart of service_call_panel.launch

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
            # the node name config/ros2/command_samples.yaml is for
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'ros2',
                                          'service_call_panel.rviz')],
            parameters=[os.path.join(share_dir, 'config', 'ros2',
                                     'command_samples.yaml')],
            output='screen',
        ),
    ])
