#!/usr/bin/env python3
###############################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Kei Okada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""
Shared pieces of the rviz integration tests.

Checks that rviz comes up with a given config, stays alive for
``test_duration`` seconds and subscribes to the expected topics.

``RvizConfigCheck`` is deliberately *not* a ``unittest.TestCase`` so that
importing it into a test module does not make launch_testing run it on its own.

Author: Kei Okada <kei.okada@gmail.com>
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import rclpy


# rviz2 always names its node `rviz`
RVIZ_NODE_NAME = 'rviz'
RVIZ_NODE_NAMESPACE = '/'

# rviz needs a display; without one the tests skip themselves
DISPLAY_REQUIRED_MESSAGE = 'DISPLAY is not set, run the test under xvfb-run'


def has_display():
    return bool(os.environ.get('DISPLAY'))


def include_sample_launch(launch_file_name, launch_arguments=None):
    """Include one of the sample launch files of this package."""
    launch_file = os.path.join(
        get_package_share_directory('jsk_rviz_plugins'), 'launch',
        launch_file_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=launch_arguments or {})


class RvizConfigCheck(object):
    """
    Assert that rviz stays alive and subscribes to the expected topics.

    Subclasses are expected to also derive from ``unittest.TestCase`` and may
    override :attr:`test_duration` and :attr:`topics`.
    """

    # check rviz is alive at least test_duration seconds
    test_duration = 5.0
    # topics rviz is expected to subscribe to
    topics = []
    # how long to wait for the rviz node to show up in the graph
    startup_timeout = 60.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rviz_config_check')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _rviz_is_up(self):
        return RVIZ_NODE_NAME in self.node.get_node_names()

    def _rviz_subscriptions(self):
        try:
            names_and_types = self.node.get_subscriber_names_and_types_by_node(
                RVIZ_NODE_NAME, RVIZ_NODE_NAMESPACE)
        except Exception as e:
            # NodeNameNonExistentError while the node is (dis)appearing
            self.node.get_logger().warn(
                'Could not read the subscriptions of {}: {}'.format(
                    RVIZ_NODE_NAME, e))
            return []
        return [name for name, _ in names_and_types]

    def test_rviz_exists(self):
        # wait for rviz to appear in the graph
        t_start = time.time()
        while not self._rviz_is_up():
            if time.time() - t_start > self.startup_timeout:
                self.fail('Timed out ({}s) waiting for the {} node'.format(
                    self.startup_timeout, RVIZ_NODE_NAME))
            self.node.get_logger().warn(
                'Waiting for the {} node for {:.3f} sec'.format(
                    RVIZ_NODE_NAME, time.time() - t_start))
            time.sleep(0.5)
        self.node.get_logger().warn(
            '{} node found after {:.3f} sec'.format(
                RVIZ_NODE_NAME, time.time() - t_start))

        # keep checking that rviz is alive and collect what it subscribes to
        subs = set()
        t_start = time.time()
        while time.time() - t_start < self.test_duration:
            self.assertTrue(
                self._rviz_is_up(),
                'The {} node disappeared after {:.3f} sec, rviz crashed?'
                .format(RVIZ_NODE_NAME, time.time() - t_start))
            subs.update(self._rviz_subscriptions())
            time.sleep(0.5)
        self.node.get_logger().warn(
            'rviz subscribes {}'.format(sorted(subs)))

        for topic in self.topics:
            self.assertIn(
                topic, subs,
                'rviz did not subscribe {}'.format(topic))

        self.node.get_logger().warn(
            'rviz kept alive for {}[sec] and found {}'.format(
                self.test_duration, self.topics))
