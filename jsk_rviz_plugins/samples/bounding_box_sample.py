#!/usr/bin/env python


import math

try:
  from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
  # jsk_common < 3.0 has no jsk_ros1_ros2_compat (ROS 1 only)
  from jsk_rviz_plugins.rospy_compat import ros_compat

from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox


def quaternion_about_axis(angle, axis):
  # tf.transformations.quaternion_about_axis() for a unit axis; tf is ROS 1 only
  s = math.sin(angle / 2.0)
  return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0)]


node = ros_compat.init_node("bbox_sample")
pub = ros_compat.create_publisher(node, "bbox", BoundingBoxArray, queue_size=1)
r = ros_compat.Rate(node, 24)
counter = 0
while not ros_compat.is_shutdown(node):
  box_a = BoundingBox()
  box_b = BoundingBox()
  box_c = BoundingBox()
  box_a.label = 2
  box_b.label = 5
  box_b.label = 10
  box_arr = BoundingBoxArray()
  now = ros_compat.stamp_now(node)
  box_a.header.stamp = now
  box_b.header.stamp = now
  box_c.header.stamp = now
  box_arr.header.stamp = now
  box_a.header.frame_id = "map"
  box_b.header.frame_id = "map"
  box_c.header.frame_id = "map"
  box_arr.header.frame_id = "map"
  q = quaternion_about_axis((counter % 100) * math.pi * 2 / 100.0, [0, 0, 1])
  box_a.pose.orientation.x = q[0]
  box_a.pose.orientation.y = q[1]
  box_a.pose.orientation.z = q[2]
  box_a.pose.orientation.w = q[3]
  box_b.pose.orientation.w = 1.0
  box_b.pose.position.y = 2.0
  box_c.pose.position.y = 1.0
  box_c.pose.position.z = -0.52
  box_b.dimensions.x = (counter % 10 + 1) * 0.1
  box_b.dimensions.y = ((counter + 1) % 10 + 1) * 0.1
  box_b.dimensions.z = ((counter + 2) % 10 + 1) * 0.1
  box_a.dimensions.x = 1.0
  box_a.dimensions.y = 1.0
  box_a.dimensions.z = 1.0
  box_c.dimensions.x = 3.0
  box_c.dimensions.y = 3.0
  box_c.dimensions.z = 0.02
  box_a.value = (counter % 100) / 100.0
  box_b.value = 1 - (counter % 100) / 100.0
  box_c.value = (counter % 300) / 100.0
  box_arr.boxes.append(box_a)
  box_arr.boxes.append(box_b)
  box_arr.boxes.append(box_c)
  pub.publish(box_arr)
  r.sleep()
  counter = counter + 1
ros_compat.shutdown(node)
