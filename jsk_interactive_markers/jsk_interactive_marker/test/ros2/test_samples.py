#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Run the ROS 2 samples (without rviz2) and check what the nodes publish."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from jsk_interactive_marker.msg import PoseStampedWithName
from jsk_interactive_marker.srv import GetTransformableMarkerPose
from jsk_rviz_plugins.msg import TransformableMarkerOperate
from jsk_rviz_plugins.srv import RequestMarkerOperate
from jsk_interactive_marker.srv import SetTransformableMarkerPose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.srv import GetInteractiveMarkers

MARKERS = ['marker_cube', 'marker_sphere', 'marker_line', 'marker_mesh']


def include(name, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('jsk_interactive_marker'),
            'launch', name)),
        launch_arguments=arguments.items())


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        include('marker_6dof_sample.launch.py', gui='false'),
        include('transformable_server_sample.launch.py', gui='false'),
        include('sample_camera_info_publisher.launch.py'),
        include('dummy_camera.launch.py'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSamples(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_jsk_interactive_marker_samples')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, done, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end and not done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return done()

    def call(self, srv_type, name, request, timeout=20.0):
        client = self.node.create_client(srv_type, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=timeout),
                            name + ' is not available')
            future = client.call_async(request)
            self.assertTrue(self.spin_until(future.done, timeout),
                            name + ' did not respond')
            return future.result()
        finally:
            self.node.destroy_client(client)

    def first_message(self, msg_type, topic, publish=None, timeout=20.0):
        received = []
        sub = self.node.create_subscription(
            msg_type, topic, received.append, 10)
        try:
            end = time.time() + timeout
            while time.time() < end and not received:
                if publish:
                    publish()
                rclpy.spin_once(self.node, timeout_sec=0.2)
            self.assertTrue(received, 'nothing on ' + topic)
            return received[0]
        finally:
            self.node.destroy_subscription(sub)

    def test_marker_6dof(self):
        for name in MARKERS:
            res = self.call(GetInteractiveMarkers,
                            '/{}/get_interactive_markers'.format(name),
                            GetInteractiveMarkers.Request())
            self.assertEqual([m.name for m in res.markers], ['marker'])
            self.assertEqual(res.markers[0].header.frame_id, 'map')
            # ~move_marker moves the marker and is echoed to ~pose
            pub = self.node.create_publisher(
                PoseStamped, '/{}/move_marker'.format(name), 1)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = 1.5
            goal.pose.orientation.w = 1.0
            pose = self.first_message(
                PoseStamped, '/{}/pose'.format(name),
                publish=lambda: pub.publish(goal))
            self.assertAlmostEqual(pose.pose.position.x, 1.5)
            self.node.destroy_publisher(pub)

    def test_transformable_server(self):
        req = RequestMarkerOperate.Request()
        req.operate.type = TransformableMarkerOperate.BOX
        req.operate.action = TransformableMarkerOperate.INSERT
        req.operate.frame_id = 'map'
        req.operate.name = 'box1'
        req.operate.description = 'box1'
        self.call(RequestMarkerOperate,
                  '/server_sample/request_marker_operate', req)
        res = self.call(GetInteractiveMarkers,
                        '/simple_marker/get_interactive_markers',
                        GetInteractiveMarkers.Request())
        self.assertIn('box1', [m.name for m in res.markers])

        received = []
        sub = self.node.create_subscription(
            PoseStampedWithName, '/server_sample/pose_with_name',
            received.append, 10)
        req = SetTransformableMarkerPose.Request()
        req.target_name = 'box1'
        req.pose_stamped.header.frame_id = 'map'
        req.pose_stamped.pose.position.x = 1.0
        req.pose_stamped.pose.position.y = 2.0
        req.pose_stamped.pose.orientation.w = 1.0
        self.call(SetTransformableMarkerPose, '/server_sample/set_pose', req)
        self.assertTrue(self.spin_until(lambda: received, 10.0))
        self.node.destroy_subscription(sub)
        self.assertEqual(received[-1].name, 'box1')
        self.assertAlmostEqual(received[-1].pose.pose.position.y, 2.0)

        req = GetTransformableMarkerPose.Request()
        req.target_name = 'box1'
        res = self.call(GetTransformableMarkerPose,
                        '/server_sample/get_pose', req)
        self.assertEqual(res.pose_stamped.header.frame_id, 'map')
        self.assertAlmostEqual(res.pose_stamped.pose.position.x, 1.0)

    def test_camera_info_publisher(self):
        info = self.first_message(CameraInfo,
                                  '/camera_info_publisher/camera_info')
        self.assertEqual((info.width, info.height), (640, 480))
        self.assertAlmostEqual(info.k[0], 525.0)
        info = self.first_message(
            CameraInfo, '/camera_info_publisher_with_yaml/camera_info')
        self.assertEqual((info.width, info.height), (1504, 1504))

    def test_dummy_camera(self):
        info = self.first_message(CameraInfo, '/dummy_camera/camera_info')
        self.assertEqual(info.header.frame_id, 'dummy_camera')
        self.assertEqual((info.width, info.height), (640, 480))


@launch_testing.post_shutdown_test()
class TestSamplesAfterShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
