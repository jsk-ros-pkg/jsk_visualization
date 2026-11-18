# -*- coding: utf-8 -*-

# ROS 2 counterpart of sample_histogram.launch (jsk_recognition_msgs is
# needed, see CMakeLists.txt)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='sample_hist_pub.py',
            name='sample_hist_pub',
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_histogram_plot',
            name='generic_array_plot',
            arguments=['/normal_array/data'],
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_histogram_plot',
            name='histogram_with_range_plot',
            arguments=['/range_array'],
        ),
    ])
