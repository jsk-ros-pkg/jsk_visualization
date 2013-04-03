#!/usr/bin/env python


import roslib
roslib.load_manifest('tf')
import rospy, tf
from geometry_msgs.msg import WrenchStamped
from math import *

pub = rospy.Publisher('wrench_test', WrenchStamped)
rospy.init_node('wrench_test')
r = rospy.Rate(10)
br = tf.TransformBroadcaster()

count = 0;
while not rospy.is_shutdown():
    t = count/10.0
    br.sendTransform((sin(t),cos(t),sin(t/2)),
                     tf.transformations.quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     "/map", "/wrench")
    msg = WrenchStamped();
    msg.header.frame_id = "/wrench"
    msg.header.stamp = rospy.Time.now();
    msg.wrench.force.x = 100*sin(t);
    msg.wrench.force.y = 100*cos(t);
    msg.wrench.force.z = 100*sin(t/2);
    msg.wrench.torque.x = 100;
    msg.wrench.torque.y = 0;
    msg.wrench.torque.z = 0;
    print msg
    pub.publish(msg);
    r.sleep()
    count += 1;

