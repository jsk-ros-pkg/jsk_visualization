#!/usr/bin/env python

import time

import numpy as np

import rospy
from std_msgs.msg import Float32


def main():
    pub1 = rospy.Publisher('~output1', Float32, queue_size=1)
    pub2 = rospy.Publisher('~output2', Float32, queue_size=1)
    pub3 = rospy.Publisher('~output3', Float32, queue_size=1)
    rate = rospy.Rate(10.0)
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        pub1.publish(float(time.time() % 1))
        pub2.publish(np.cos(time.time()).astype(np.float32))
        pub3.publish(np.sin(time.time()).astype(np.float32))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sample_3d_plot')
    main()
