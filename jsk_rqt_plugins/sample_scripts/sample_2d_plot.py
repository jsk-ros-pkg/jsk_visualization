#!/usr/bin/env python

from jsk_recognition_msgs.msg import PlotData
import numpy as np
import rospy
import math
from random import random

if __name__ == "__main__":
    rospy.init_node("sample_2d_plot")
    pub = rospy.Publisher("~output", PlotData, queue_size=1)
    r = rospy.Rate(10)
    offset = 0
    while not rospy.is_shutdown():
        msg = PlotData()
        msg.xs = np.arange(0, 5, 0.1)
        msg.ys = [math.sin(x + offset) for x in msg.xs]
        msg.label = "sample data"
        if random() < 0.5:
            msg.type = PlotData.LINE
        else:
            msg.type = PlotData.SCATTER
        msg.fit_line = random() < 0.5
        msg.fit_line_ransac = random() < 0.5
        pub.publish(msg)
        offset = offset + 0.1
        r.sleep()
