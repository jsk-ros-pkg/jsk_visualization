# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_rviz_plugins')
    rviz_share_dir = get_package_share_directory('rviz_common')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_sample.py',
            name='overlay_sample',
            respawn=True,
        ),
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_menu_sample.py',
            name='test_menu',
            respawn=True,
        ),
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='image_publisher',
            arguments=[os.path.join(rviz_share_dir, 'images', 'splash.png')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('gui')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'overlay_sample.rviz')],
            output='screen',
        ),
    ])
