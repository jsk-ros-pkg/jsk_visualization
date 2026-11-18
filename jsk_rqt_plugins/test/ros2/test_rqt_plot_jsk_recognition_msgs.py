#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Check that rqt_histogram_plot and rqt_2d_plot, the plugins of
jsk_recognition_msgs (installed only when it is found, see CMakeLists.txt),
come up and subscribe the topics of their samples."""

import os
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy

# rqt needs a display; without one the test skips itself
DISPLAY_REQUIRED_MESSAGE = 'DISPLAY is not set, run the test under xvfb-run'

# rqt node name -> topic it has to subscribe
RQT_NODES = {
    'rqt_histogram_plot': '/pub_sample_hist_data/range_array',
    'rqt_2d_plot': '/pub_sample_2d_data/output',
}


def has_display():
    return bool(os.environ.get('DISPLAY'))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    if not has_display():
        # Nothing to launch without a display; the test cases below are skipped.
        return LaunchDescription([launch_testing.actions.ReadyToTest()])
    return LaunchDescription([
        # publishers the plugins are checked against, in a namespace as the
        # private topics of sample_2d_plot.py
        Node(package='jsk_rqt_plugins', executable='sample_hist_pub.py',
             name='pub_sample_hist_data', namespace='pub_sample_hist_data'),
        Node(package='jsk_rqt_plugins', executable='sample_2d_plot.py',
             name='pub_sample_2d_data'),
        # rqt plugins
        Node(package='jsk_rqt_plugins', executable='rqt_histogram_plot',
             name='rqt_histogram_plot',
             arguments=[RQT_NODES['rqt_histogram_plot']]),
        Node(package='jsk_rqt_plugins', executable='rqt_2d_plot',
             name='rqt_2d_plot',
             arguments=[RQT_NODES['rqt_2d_plot']]),
        launch_testing.actions.ReadyToTest(),
    ])


@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestRqtPlotJskRecognitionMsgs(unittest.TestCase):

    # how long to wait for every rqt plugin to show up in the graph
    startup_timeout = 90.0
    # how long the plugins have to stay alive afterwards
    test_duration = 5.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_rqt_plot_jsk_recognition_msgs')

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
            self.node.get_logger().warning(
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
            self.node.get_logger().warning('Waiting for {}'.format(missing))
            time.sleep(1.0)

    def test_rqt_plots_are_alive(self):
        self._wait_for_nodes(RQT_NODES)
        t_start = time.time()
        while time.time() - t_start < self.test_duration:
            names = self._node_names()
            for rqt_node in RQT_NODES:
                self.assertIn(
                    rqt_node, names,
                    '{} disappeared after {:.3f} sec, the plugin crashed?'
                    .format(rqt_node, time.time() - t_start))
            time.sleep(0.5)

    def test_rqt_plots_subscribe(self):
        self._wait_for_nodes(RQT_NODES)
        for rqt_node, topic in RQT_NODES.items():
            t_start = time.time()
            while topic not in self._subscriptions_of(rqt_node):
                if time.time() - t_start > 30.0:
                    self.fail('{} does not subscribe {}'.format(rqt_node, topic))
                time.sleep(1.0)


@launch_testing.post_shutdown_test()
@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestRqtPlotJskRecognitionMsgsAfterShutdown(unittest.TestCase):

    def test_no_plugin_crashed(self, proc_info):
        # as test_rqt_plugins.py
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15, -6, -11])
