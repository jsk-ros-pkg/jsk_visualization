# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_drc_mini_maxwell',
            name='rqt_drc_mini_maxwell',
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_drc_mini_maxwell.py',
            name='pub_sample_state',
        ),
    ])
