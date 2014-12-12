#!/usr/bin/env python

import rospy
import numpy
from math import pi
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PointStamped
from jsk_gui_msgs.msg import Tablet
from threading import Lock

lock = Lock()
prev_mouse_point = None
latest_mouse_point = None

def pointCallback(msg):
    global latest_mouse_point
    with lock:
        latest_mouse_point = msg
    
def timerCallback(event):
    global latest_mouse_point, prev_mouse_point
    # print (latest_mouse_point and latest_mouse_point.point, 
    #        prev_mouse_point and prev_mouse_point.point)
    with lock:
        if not latest_mouse_point:
            return
        if not prev_mouse_point:
            prev_mouse_point = latest_mouse_point
            latest_mouse_point = None
            return
        # compute diff
        diff_x = (latest_mouse_point.point.x - prev_mouse_point.point.x) * 640
        diff_y = (latest_mouse_point.point.y - prev_mouse_point.point.y) * 480
        # center is 320, 240
        next_x = diff_x + 320
        next_y = diff_y + 240
        msg = Tablet()
        msg.header.stamp = rospy.Time.now()
        msg.action.task_name = "MoveCameraCenter"
        # chop
        msg.action.touch_x = next_x
        msg.action.touch_y = next_y
        rospy.loginfo("neck actino to (%f, %f)" % (msg.action.touch_x, msg.action.touch_y))
        pub.publish(msg)
        #prev_mouse_point = latest_mouse_point
        prev_mouse_point = None
        latest_mouse_point = None

        
if __name__ == "__main__":
    rospy.init_node("rviz_mouse_point_to_tablet")
    pub = rospy.Publisher("/Tablet/Command", Tablet)
    sub = rospy.Subscriber("/rviz/current_mouse_point", 
                           PointStamped, pointCallback)
    rospy.Timer(rospy.Duration(0.3), timerCallback)
    rospy.spin()
