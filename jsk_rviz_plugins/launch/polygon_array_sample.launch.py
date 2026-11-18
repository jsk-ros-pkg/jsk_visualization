# -*- coding: utf-8 -*-

# ROS 2 counterpart of polygon_array_sample.launch (jsk_recognition_msgs is needed,
# see CMakeLists.txt)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_rviz_plugins')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_rviz_plugins',
            executable='polygon_array_sample.py',
            name='polygon_array_sample',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('gui')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'ros2',
                                          'polygon_array_sample.rviz')],
        ),
    ])
