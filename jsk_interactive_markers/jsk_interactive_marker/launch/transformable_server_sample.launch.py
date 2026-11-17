# -*- coding: utf-8 -*-

# ROS 2 counterpart of transformable_server_sample.launch, without the
# KORG nanoKONTROL and spacenav nodes (korg_nanokontrol and the configure
# scripts are ROS 1 only)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_interactive_marker')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='jsk_interactive_marker',
            executable='transformable_server_sample',
            name='server_sample',
            output='screen',
            # torus_udiv/torus_vdiv of the .launch are global parameters,
            # which the node (~torus_udiv) does not read, so not here either
            parameters=[{
                'display_interactive_manipulator': False,
                'yaml_filename': os.path.join(share_dir, 'config',
                                              'default_menu.yaml'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('gui')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'ros2',
                                          'transformable_server_sample.rviz')],
            remappings=[('request_marker_operate',
                         '/server_sample/request_marker_operate')],
        ),
    ])
