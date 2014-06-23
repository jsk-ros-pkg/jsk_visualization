#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

rospy.init_node("dummy_camera")

pub = rospy.Publisher("~image", Image)
pub_info = rospy.Publisher("~camera_info", CameraInfo)
frame_id = rospy.get_param('~frame_id', 'dummy_camera')
width = rospy.get_param('~width', 640)
height = rospy.get_param('~height', 480)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    now = rospy.Time(0)
    dummy_camera_info = CameraInfo()
    dummy_camera_info.width = width
    dummy_camera_info.height = height
    dummy_camera_info.header.frame_id = frame_id
    dummy_camera_info.header.stamp  = now
    dummy_camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    dummy_camera_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
    dummy_camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    dummy_camera_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    dummy_camera_info.distortion_model = "plumb_bob"
    pub_info.publish(dummy_camera_info)

    dummy_img = Image()
    dummy_img.width = width
    dummy_img.height = height
    dummy_img.header.frame_id = frame_id
    dummy_img.header.stamp = now
    dummy_img.step = width
    dummy_img.encoding = "mono8"
    dummy_img.data = [0] * width * height
    pub.publish(dummy_img)
    r.sleep()
