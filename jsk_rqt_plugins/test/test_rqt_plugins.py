#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Check that the rqt plugins come up and subscribe the expected topics."""

import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy

# rqt needs a display; without one the test skips itself
DISPLAY_REQUIRED_MESSAGE = 'DISPLAY is not set, run the test under xvfb-run'

BUTTON_LAYOUT = ('package://jsk_rqt_plugins/resource/'
                 'service_button_layout.yaml')
RADIO_BUTTON_LAYOUT = ('package://jsk_rqt_plugins/resource/'
                       'service_radio_button_layout.yaml')
TABBED_LAYOUT = {
    'tabbed_layout.tab_list': ['push', 'radio'],
    'tabbed_layout.push.name': 'push button',
    'tabbed_layout.push.namespace': 'push',
    'tabbed_layout.push.type': 'push',
    'tabbed_layout.push.yaml_file': BUTTON_LAYOUT,
    'tabbed_layout.radio.name': 'radio button',
    'tabbed_layout.radio.namespace': 'radio',
    'tabbed_layout.radio.type': 'radio',
    'tabbed_layout.radio.yaml_file': RADIO_BUTTON_LAYOUT,
}

# rqt node names, set through the launch `name` of each process
RQT_NODES = [
    'rqt_3d_plot',
    'rqt_drc_mini_maxwell',
    'rqt_service_buttons',
    'rqt_status_light',
    'rqt_string_label',
    'rqt_tabbed_buttons',
]

DRC_TOPICS = [
    '/drc_2015_environment/is_blackout',
    '/drc_2015_environment/is_disabled',
    '/drc_2015_environment/next_whiteout_time',
]

PLOT_TOPIC = '/pub_sample_3d_data/output1'


def has_display():
    return bool(os.environ.get('DISPLAY'))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    if not has_display():
        # Nothing to launch without a display; the test cases below are skipped.
        return LaunchDescription([launch_testing.actions.ReadyToTest()])
    return LaunchDescription([
        # publishers the plugins are checked against
        Node(package='jsk_rqt_plugins', executable='sample_3d_plot.py',
             name='pub_sample_3d_data'),
        Node(package='jsk_rqt_plugins', executable='sample_drc_mini_maxwell.py',
             name='pub_sample_drc_mini_maxwell'),
        Node(package='jsk_rqt_plugins', executable='sample_service_buttons.py',
             name='sample_service_buttons'),
        # rqt plugins
        Node(package='jsk_rqt_plugins', executable='rqt_drc_mini_maxwell',
             name='rqt_drc_mini_maxwell'),
        Node(package='jsk_rqt_plugins', executable='rqt_service_buttons',
             name='rqt_service_buttons',
             parameters=[{'layout_yaml_file': BUTTON_LAYOUT}]),
        Node(package='jsk_rqt_plugins', executable='rqt_tabbed_buttons',
             name='rqt_tabbed_buttons', parameters=[TABBED_LAYOUT]),
        Node(package='jsk_rqt_plugins', executable='rqt_status_light',
             name='rqt_status_light'),
        Node(package='jsk_rqt_plugins', executable='rqt_string_label',
             name='rqt_string_label'),
        # Plot3D resolves the topic type when it subscribes and retries until
        # it succeeds; a small head start just avoids the first retry.
        TimerAction(period=5.0, actions=[
            Node(package='jsk_rqt_plugins', executable='rqt_3d_plot',
                 name='rqt_3d_plot',
                 arguments=[PLOT_TOPIC + '/data']),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestRqtPlugins(unittest.TestCase):

    # how long to wait for every rqt plugin to show up in the graph
    startup_timeout = 90.0
    # how long the plugins have to stay alive afterwards
    test_duration = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_rqt_plugins')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _node_names(self):
        return [name for name, _ in self.node.get_node_names_and_namespaces()]

    def _subscriptions_of(self, node_name):
        try:
            names_and_types = self.node.get_subscriber_names_and_types_by_node(
                node_name, '/')
        except Exception as e:
            self.node.get_logger().warn(
                'Could not read the subscriptions of {}: {}'.format(
                    node_name, e))
            return []
        return [name for name, _ in names_and_types]

    def _wait_for_nodes(self, expected):
        t_start = time.time()
        missing = list(expected)
        while missing:
            names = self._node_names()
            missing = [n for n in expected if n not in names]
            if not missing:
                return
            if time.time() - t_start > self.startup_timeout:
                self.fail('Timed out ({}s) waiting for {}'.format(
                    self.startup_timeout, missing))
            self.node.get_logger().warn('Waiting for {}'.format(missing))
            time.sleep(1.0)

    def _wait_for_subscription(self, node_name, topic, timeout):
        t_start = time.time()
        while time.time() - t_start < timeout:
            if topic in self._subscriptions_of(node_name):
                return True
            time.sleep(1.0)
        return False

    def test_rqt_plugins_are_alive(self):
        self._wait_for_nodes(RQT_NODES)
        # every plugin has to survive test_duration seconds
        t_start = time.time()
        while time.time() - t_start < self.test_duration:
            names = self._node_names()
            for rqt_node in RQT_NODES:
                self.assertIn(
                    rqt_node, names,
                    '{} disappeared after {:.3f} sec, the plugin crashed?'
                    .format(rqt_node, time.time() - t_start))
            time.sleep(0.5)

    def test_rqt_drc_mini_maxwell(self):
        self._wait_for_nodes(['rqt_drc_mini_maxwell'])
        for topic in DRC_TOPICS:
            self.assertTrue(
                self._wait_for_subscription(
                    'rqt_drc_mini_maxwell', topic, 30.0),
                'rqt_drc_mini_maxwell does not subscribe {}'.format(topic))

    def test_rqt_3d_plot(self):
        self._wait_for_nodes(['rqt_3d_plot'])
        self.assertTrue(
            self._wait_for_subscription('rqt_3d_plot', PLOT_TOPIC, 30.0),
            'rqt_3d_plot does not subscribe {}'.format(PLOT_TOPIC))


@launch_testing.post_shutdown_test()
@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestRqtPluginsAfterShutdown(unittest.TestCase):

    def test_no_plugin_crashed(self, proc_info):
        # 0 on a clean quit, -2/-15 when launch_testing signals the processes
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
