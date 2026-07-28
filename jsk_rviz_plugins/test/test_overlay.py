#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS 2 port of the ROS 1 overlay.test rostest (rviz crash test)."""

import os
import sys
import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

# launch_test.py loads this file by path, so make the helper module importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rviz_config_check import DISPLAY_REQUIRED_MESSAGE  # noqa: E402
from rviz_config_check import has_display  # noqa: E402
from rviz_config_check import include_sample_launch  # noqa: E402
from rviz_config_check import RvizConfigCheck  # noqa: E402


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    if not has_display():
        # Nothing to launch without a display; the test cases below are skipped.
        return LaunchDescription([launch_testing.actions.ReadyToTest()])
    return LaunchDescription([
        include_sample_launch('overlay_sample.launch.py'),
        launch_testing.actions.ReadyToTest(),
    ])


@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestOverlay(RvizConfigCheck, unittest.TestCase):

    test_duration = 5.0
    topics = [
        '/diagnostics_agg',
        '/image_publisher/image_raw',
        '/sample_string',
        '/test_menu',
        '/value_sample',
    ]


@launch_testing.post_shutdown_test()
@unittest.skipUnless(has_display(), DISPLAY_REQUIRED_MESSAGE)
class TestOverlayAfterShutdown(unittest.TestCase):

    def test_rviz_did_not_crash(self, proc_info):
        # 0 on a clean quit, -2/-15 when launch_testing signals rviz to stop
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
