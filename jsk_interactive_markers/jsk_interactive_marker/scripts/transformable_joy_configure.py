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

x_max = 10
y_max = 10
z_max = 10
r_max = 10
sr_max = 10

x_small_max = 1
y_small_max = 1
z_small_max = 1
r_small_max = 1
sr_small_max = 1

MIN_VALUE=0.001

def callback(msg):
    global x_max, y_max, z_max, r_max, sr_max
    global x_small_max, y_small_max, z_small_max, r_small_max, sr_small_max
    a = msg.axes
    b = msg.buttons

    x_v1,y_v1,z_v1,r_v1,sr_v1 = a[13:]
    x_v2,y_v2,z_v2,r_v2,sr_v2 = a[0:5]

    x = Float32()
    y = Float32()
    z = Float32()
    r = Float32()
    sr= Float32()

    x.data = x_max * (x_v1+1)/2 + x_small_max * (x_v2+1)/2;
    y.data = y_max * (y_v1+1)/2 + y_small_max * (y_v2+1)/2;
    z.data = z_max * (z_v1+1)/2 + z_small_max * (z_v2+1)/2;
    r.data = r_max * (r_v1+1)/2 + r_small_max * (r_v2+1)/2;
    sr.data= sr_max * (sr_v1+1)/2 + sr_small_max * (sr_v2+1)/2;

    x.data = max(MIN_VALUE,x.data)
    y.data = max(MIN_VALUE,y.data)
    z.data = max(MIN_VALUE,z.data)
    r.data = max(MIN_VALUE,r.data)
    sr.data = max(MIN_VALUE,sr.data)

    set_x_pub.publish(x)
    set_y_pub.publish(y)
    set_z_pub.publish(z)
    set_r_pub.publish(r)
    set_sr_pub.publish(sr)

if __name__ == "__main__":
    rospy.init_node("transformable_joy_configure")
    set_x_pub = rospy.Publisher("set_x", Float32)
    set_y_pub = rospy.Publisher("set_y", Float32)
    set_z_pub = rospy.Publisher("set_z", Float32)
    set_r_pub = rospy.Publisher("set_radius", Float32)
    set_sr_pub = rospy.Publisher("set_small_radius", Float32)

    x_max = rospy.get_param("~x_max")
    y_max = rospy.get_param("~y_max")
    z_max = rospy.get_param("~z_max")
    r_max = rospy.get_param("~r_max")
    sr_max = rospy.get_param("~sr_max")

    x_small_max = rospy.get_param("~x_small_max")
    y_small_max = rospy.get_param("~y_small_max")
    z_small_max = rospy.get_param("~z_small_max")
    r_small_max = rospy.get_param("~r_small_max")
    sr_small_max = rospy.get_param("~sr_small_max")


    s = rospy.Subscriber("input_joy", Joy, callback)
    rospy.spin()

