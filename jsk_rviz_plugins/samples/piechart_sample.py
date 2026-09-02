#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PieChartSample(Node):
    """Publish the value shown by config/linear_gauge_sample.rviz."""

    def __init__(self):
        super().__init__('piechart_sample')
        self.pub = self.create_publisher(Float32, '/sample_piechart', 1)
        self.count = 0
        self.timer = self.create_timer(0.1, self._timer_cb)

    def _timer_cb(self):
        msg = Float32()
        msg.data = abs(math.sin(math.pi * self.count / 100.0))
        self.count += 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PieChartSample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
