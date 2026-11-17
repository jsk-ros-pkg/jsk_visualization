#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""bounding_box_marker (built when jsk_recognition_msgs is found)."""

import time
import unittest

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node
import pytest
import rclpy
from visualization_msgs.srv import GetInteractiveMarkers


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(package='jsk_interactive_marker',
             executable='bounding_box_marker',
             name='bounding_box_marker'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestBoundingBoxMarker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_bounding_box_marker')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_boxes(self):
        pub = self.node.create_publisher(
            BoundingBoxArray, '/bounding_box_marker/bounding_box_array', 1)
        boxes = BoundingBoxArray()
        boxes.header.frame_id = 'map'
        for i in range(2):
            box = BoundingBox()
            box.header.frame_id = 'map'
            box.pose.position.x = float(i)
            box.pose.orientation.w = 1.0
            box.dimensions.x = box.dimensions.y = box.dimensions.z = 0.3
            boxes.boxes.append(box)
        client = self.node.create_client(
            GetInteractiveMarkers,
            '/bounding_box_interactive_marker/get_interactive_markers')
        self.assertTrue(client.wait_for_service(timeout_sec=20.0))
        names = []
        end = time.time() + 20.0
        while time.time() < end and names != ['box_0', 'box_1']:
            pub.publish(boxes)
            future = client.call_async(GetInteractiveMarkers.Request())
            while not future.done() and time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                names = sorted(m.name for m in future.result().markers)
        self.assertEqual(names, ['box_0', 'box_1'])


@launch_testing.post_shutdown_test()
class TestBoundingBoxMarkerAfterShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
