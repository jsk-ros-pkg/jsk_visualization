#!/usr/bin/env python

from jsk_recognition_msgs.msg import PlotData
import numpy as np
from jsk_rqt_plugins.compat import ros_compat
from jsk_rqt_plugins.compat import exit_on_shutdown
import math
from random import random

if __name__ == "__main__":
    node = ros_compat.init_node("sample_2d_plot")
    pub = ros_compat.create_publisher(node, "~output", PlotData, queue_size=1)
    r = ros_compat.Rate(node, 10)
    offset = 0
    with exit_on_shutdown():
        while not ros_compat.is_shutdown(node):
            msg = PlotData()
            # a list of floats: the float32[] of ROS 2 (rclpy) messages does not
            # take a numpy array
            msg.xs = [float(x) for x in np.arange(0, 5, 0.1)]
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
