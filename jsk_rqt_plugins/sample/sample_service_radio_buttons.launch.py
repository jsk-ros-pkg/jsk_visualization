# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perspective = os.path.join(
        get_package_share_directory('jsk_rqt_plugins'), 'resource',
        'rqt_service_radio_buttons.perspective')
    return LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_service_radio_buttons',
            arguments=['--perspective-file', perspective],
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_radio_buttons.py',
            name='sample_service_radio_buttons',
            output='screen',
        ),
    ])
