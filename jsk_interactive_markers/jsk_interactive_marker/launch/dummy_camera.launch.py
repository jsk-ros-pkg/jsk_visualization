# -*- coding: utf-8 -*-

# ROS 2 counterpart of dummy_camera.launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    name = LaunchConfiguration('dummy_camera_name').perform(context)
    origin_tf = LaunchConfiguration('origin_tf').perform(context)
    # x y z yaw pitch roll, as tf's static_transform_publisher of ROS 1
    x, y, z, yaw, pitch, roll = \
        LaunchConfiguration('tf_pose').perform(context).split()
    return [
        Node(
            package='jsk_interactive_marker',
            executable='dummy_camera.py',
            name=name,
            parameters=[{'frame_id': name}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name + '_tf',
            arguments=['--x', x, '--y', y, '--z', z,
                       '--yaw', yaw, '--pitch', pitch, '--roll', roll,
                       '--frame-id', origin_tf.lstrip('/'),
                       '--child-frame-id', name],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('origin_tf', default_value='r_gripper_tool_frame'),
        # ROS 1's $(anon dummy_carmera); launch_ros has no anon
        DeclareLaunchArgument('dummy_camera_name', default_value='dummy_camera'),
        DeclareLaunchArgument('tf_pose', default_value='0 0 1 1.5708 3.1416 0'),
        OpaqueFunction(function=launch_setup),
    ])
