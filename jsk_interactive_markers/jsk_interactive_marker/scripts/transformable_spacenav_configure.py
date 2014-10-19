#!/usr/bin/env python

import rospy

PKG='sensor_msgs'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from sensor_msgs.msg import Joy
from geometry_msgs.msg import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from jsk_rviz_plugins.msg import OverlayText

x_max = 10
y_max = 10
z_max = 10

set_pose_pub = None
prev_b = 0
translate_mode = True
display_separate_mode = True
def callback(msg):
    global x_max, y_max, z_max, rx_max, ry_max, rz_max, set_pose_pub
    global prev_b, translate_mode

    a = msg.axes
    b = msg.buttons

    if prev_b == 0 and (b[0] == 1 or b[1] == 1):
        translate_mode = not translate_mode

    if b[0] == 1 or b[1] == 1:
        prev_b = 1
    else:
        prev_b = 0

    x_v1,y_v1,z_v1,rx_v1,ry_v1,rz_v1 = a

    target_pose = Pose()
    if not separate_mode or (separate_mode and translate_mode):
        target_pose.position.x = x_v1 * x_max
        target_pose.position.y = y_v1 * y_max
        target_pose.position.z = z_v1 * z_max

    target_pose.orientation.w = 1
    if not separate_mode or (separate_mode and not translate_mode):
        q = quaternion_from_euler(rx_v1 * rx_max, ry_v1 * ry_max, rz_v1 * rz_max, 'rxyz')
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

    set_pose_pub.publish(target_pose)
    if translate_mode and separate_mode:
        publishModeText("TranslateMode")
    elif separate_mode:
        publishModeText("RotateMode")

def publishModeText(message):
    send_text = OverlayText()
    send_text.text = message
    send_text.top = 100
    send_text.left = 0
    send_text.width = 300
    send_text.height = 50

    send_text.bg_color.r = 0.9;
    send_text.bg_color.b = 0.9;
    send_text.bg_color.g = 0.9;
    send_text.bg_color.a = 0.1;
    send_text.fg_color.r = 0.3;
    send_text.fg_color.g = 0.8;
    send_text.fg_color.b = 0.3;
    send_text.fg_color.a = 1;
    send_text.line_width = 1;
    send_text.text_size = 30;
    send_text_pub.publish(send_text);


if __name__ == "__main__":
    rospy.init_node("transformable_spacenav_configure")
    ns = rospy.get_param('~transformable_interactive_server_nodename', '')

    x_max = rospy.get_param("~x_max")
    y_max = rospy.get_param("~y_max")
    z_max = rospy.get_param("~z_max")
    rx_max = rospy.get_param("~rx_max")
    ry_max = rospy.get_param("~ry_max")
    rz_max = rospy.get_param("~rz_max")

    separate_mode = rospy.get_param("~separate_mode")
    display_separate_mode = rospy.get_param("~display_separate_mode")

    set_pose_pub = rospy.Publisher(ns+"/add_pose", Pose)
    send_text_pub = rospy.Publisher("separate_mode_text", OverlayText)

    s = rospy.Subscriber("input_joy", Joy, callback)
    rospy.spin()

