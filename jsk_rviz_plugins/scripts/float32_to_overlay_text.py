#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from threading import Lock
#from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface

g_lock = Lock()
g_msg = None

def callback(msg):
    global g_msg, g_lock
    with g_lock:
        g_msg = msg

def config_callback(config, level):
    global g_format
    g_format = config.format
    return config

def publish_text(event):
    global g_lock, g_msg, g_format
    with g_lock:
        if not g_msg:
            return
        text_interface.publish(g_format.format(g_msg.data))

if __name__ == "__main__":
    rospy.init_node("octree_info")
    text_interface = OverlayTextInterface("~text")
    g_format = rospy.get_param("~format", "value: {0}")
    sub = rospy.Subscriber("~input", Float32, callback)
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
