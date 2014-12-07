#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import Pictogram

rospy.init_node("pictogram_sample")
p = rospy.Publisher("/pictogram", Pictogram)

r = rospy.Rate(1)

while not rospy.is_shutdown():
    msg = Pictogram()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.orientation.w = 1
    msg.character = "skype"
    p.publish(msg)
    r.sleep()
