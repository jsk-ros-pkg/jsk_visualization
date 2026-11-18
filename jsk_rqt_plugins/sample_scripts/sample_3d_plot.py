#!/usr/bin/env python

import time

import numpy as np

from jsk_rqt_plugins.compat import ros_compat
from jsk_rqt_plugins.compat import exit_on_shutdown
from std_msgs.msg import Float32


def main():
    pub1 = ros_compat.create_publisher(node, '~output1', Float32, queue_size=1)
    pub2 = ros_compat.create_publisher(node, '~output2', Float32, queue_size=1)
    pub3 = ros_compat.create_publisher(node, '~output3', Float32, queue_size=1)
    rate = ros_compat.Rate(node, 10.0)
    ros_compat.sleep(node, 1.0)

    while not ros_compat.is_shutdown(node):
        pub1.publish(Float32(data=float(time.time() % 1)))
        pub2.publish(Float32(data=float(np.cos(time.time()))))
        pub3.publish(Float32(data=float(np.sin(time.time()))))
        rate.sleep()


if __name__ == '__main__':
    node = ros_compat.init_node('sample_3d_plot')
    with exit_on_shutdown():
        main()
