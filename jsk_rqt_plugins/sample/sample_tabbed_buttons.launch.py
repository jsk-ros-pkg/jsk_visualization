# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

# ROS 2 parameters cannot hold a nested structure, so the tabs of
# jsk_rqt_plugin/ServiceTabbedButtons are described as a tab_list plus one
# parameter group per tab.
TABBED_LAYOUT = {
    'tabbed_layout.tab_list': ['push', 'radio'],
    'tabbed_layout.push.name': 'push button',
    'tabbed_layout.push.namespace': 'push',
    'tabbed_layout.push.type': 'push',
    'tabbed_layout.push.yaml_file':
        'package://jsk_rqt_plugins/resource/service_button_layout.yaml',
    'tabbed_layout.radio.name': 'radio button',
    'tabbed_layout.radio.namespace': 'radio',
    'tabbed_layout.radio.type': 'radio',
    'tabbed_layout.radio.yaml_file':
        'package://jsk_rqt_plugins/resource/service_radio_button_layout.yaml',
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jsk_rqt_plugins',
            executable='rqt_tabbed_buttons',
            name='sample_tabbed_buttons',
            output='screen',
            parameters=[TABBED_LAYOUT],
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_buttons.py',
            name='push_sample_service_buttons',
            namespace='push',
            output='screen',
        ),
        Node(
            package='jsk_rqt_plugins',
            executable='sample_service_radio_buttons.py',
            name='sample_service_radio_buttons',
            namespace='radio',
            output='screen',
        ),
    ])
