# -*- coding: utf-8 -*-

# ROS 2 counterpart of sample_2dplot.launch (jsk_recognition_msgs is needed,
# see CMakeLists.txt)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topic = '/sample_data/output'
    plots = [
        ('scatter_plot', []),
        ('line_plot', ['--line']),
        ('scatter_plot_fit', ['--fit-line']),
        ('line_plot_fit', ['--line', '--fit-line']),
        ('line_plot_fit_ransac', ['--line', '--fit-line-ransac']),
    ]
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_2d_plot',
            name=name,
            arguments=[topic] + args,
        ) for name, args in plots
    ] + [
        Node(
            package='jsk_rqt_plugins',
            executable='sample_2d_plot.py',
            name='sample_data',
        ),
    ])
