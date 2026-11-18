#!/usr/bin/env python

import math

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat (ROS 1 only)
    from jsk_rviz_plugins.rospy_compat import ros_compat
from std_msgs.msg import Float32


class PieChartSample():

    def __init__(self, node):
        self.pub = ros_compat.create_publisher(
            node, '/sample_piechart', Float32, queue_size=1)
        self.timer = ros_compat.create_timer(
            node, 0.1, self._timer_cb)
        self.count = 0

    def _timer_cb(self, event):
        msg = Float32()
        msg.data = abs(math.sin(3.14 * self.count / 100.0))
        self.count = self.count + 1
        self.pub.publish(msg)


if __name__ == '__main__':
    node = ros_compat.init_node('piechart_sample')
    app = PieChartSample(node)
    ros_compat.spin(node)
