#!/usr/bin/env python

from jsk_rqt_plugins.compat import ros_compat
from std_msgs.msg import Float32MultiArray
from jsk_recognition_msgs.msg import HistogramWithRange, HistogramWithRangeBin
import numpy as np

if __name__ == "__main__":
    node = ros_compat.init_node("sample_hist_pub")
    pub = ros_compat.create_publisher(
        node, "normal_array", Float32MultiArray, queue_size=1)
    pub_range = ros_compat.create_publisher(
        node, "range_array", HistogramWithRange, queue_size=1)
    r = ros_compat.Rate(node, 1)
    while not ros_compat.is_shutdown(node):
        data = np.random.normal(size=1000)
        range_msg = HistogramWithRange()
        hist, bins = np.histogram(data, bins=50)
        # python floats and ints: the fields of ROS 2 (rclpy) messages do
        # not take numpy arrays and numpy integers
        for v, min, max in zip(hist, bins[:-1], bins[1:]):
            msg_bin = HistogramWithRangeBin()
            msg_bin.max_value = float(max)
            msg_bin.min_value = float(min)
            msg_bin.count = int(v)
            range_msg.bins.append(msg_bin)
        msg = Float32MultiArray()
        msg.data = [float(v) for v in hist]
        pub.publish(msg)
        pub_range.publish(range_msg)
        r.sleep()
