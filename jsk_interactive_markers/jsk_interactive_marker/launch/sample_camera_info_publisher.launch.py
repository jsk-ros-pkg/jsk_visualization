# -*- coding: utf-8 -*-

# ROS 2 counterpart of sample_camera_info_publisher.launch

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_interactive_marker')
    return LaunchDescription([
        Node(
            package='jsk_interactive_marker',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
        ),
        Node(
            package='jsk_interactive_marker',
            executable='camera_info_publisher',
            name='camera_info_publisher_with_yaml',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.join(share_dir, 'config',
                                              'sample_camera_info.yaml'),
            }],
        ),
    ])
