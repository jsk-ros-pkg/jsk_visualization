# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topics = ['/sample_data/output1/data',
              '/sample_data/output2/data',
              '/sample_data/output3/data']
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='sample_3d_plot.py',
            name='sample_data',
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_3d_plot',
            name='polygon_plot',
            arguments=topics + ['--buffer', '1000'],
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_3d_plot',
            name='line_plot',
            arguments=topics + ['--line', '--buffer', '1000'],
        ),
    ])
