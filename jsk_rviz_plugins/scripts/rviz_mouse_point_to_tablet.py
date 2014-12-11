#!/usr/bin/env python

import rospy
import numpy
from math import pi
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PointStamped
from jsk_gui_msgs.msg import Tablet

latest_camera_placement = None
def pointCallback(msg):
    global latest_camera_placement
    latest_camera_placement = msg
    
def timerCallback(event):
    global latest_camera_placement
    if not latest_camera_placement:
        return
    msg = Tablet()
    msg.header.stamp = rospy.Time.now()
    msg.action.task_name = "MoveCameraCenter"
    # chop
    msg.action.touch_x = 640 * latest_camera_placement.point.x
    msg.action.touch_y = 480 * latest_camera_placement.point.y
    print (msg.action.touch_x, msg.action.touch_y)
    pub.publish(msg)
    latest_camera_placement = None
        
rospy.init_node("rviz_mouse_point_to_tablet")
pub = rospy.Publisher("/Tablet/Command", Tablet)

sub = rospy.Subscriber("/rviz/current_mouse_point", 
                       PointStamped, pointCallback)
rospy.Timer(rospy.Duration(0.5), timerCallback)
rospy.spin()
