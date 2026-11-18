# -*- coding: utf-8 -*-

# ROS 2 counterpart of marker_6dof_sample.launch

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('jsk_interactive_marker')
    markers = [
        ('marker_cube', {'object_type': 'cube',
                         'object_x': 0.2, 'object_y': 0.2, 'object_z': 0.3,
                         'object_r': 1.0, 'object_g': 0.2, 'object_b': 0.2,
                         'object_a': 1.0}),
        ('marker_sphere', {'object_type': 'sphere',
                           'object_x': 0.2, 'object_y': 0.2, 'object_z': 0.3,
                           'object_r': 0.2, 'object_g': 1.0, 'object_b': 0.2,
                           'object_a': 1.0}),
        ('marker_line', {'object_type': 'line',
                         'object_x': 0.1,
                         'object_r': 0.2, 'object_g': 0.2, 'object_b': 1.0,
                         'object_a': 1.0}),
        ('marker_mesh', {'object_type': 'mesh',
                         'object_x': 1.0, 'object_y': 0.5, 'object_z': 1.0,
                         'object_r': 1.0, 'object_g': 0.2, 'object_b': 1.0,
                         'object_a': 1.0,
                         'mesh_file': 'package://jsk_interactive_marker/'
                                      'models/sample_mesh.dae'}),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
    ] + [
        Node(
            package='jsk_interactive_marker',
            executable='marker_6dof',
            name=name,
            parameters=[dict(params, frame_id='map')],
        )
        for name, params in markers
    ] + [
        Node(
            condition=IfCondition(LaunchConfiguration('gui')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'ros2',
                                          'marker_6dof_sample.rviz')],
        ),
    ])
