#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from builtin_interfaces.msg import Time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool


class SampleDRCMiniMaxwell(Node):

    def __init__(self):
        super().__init__('sample_drc_mini_maxwell')
        self.pub_is_disabled = self.create_publisher(
            Bool, '/drc_2015_environment/is_disabled', 1)
        self.pub_is_blackout = self.create_publisher(
            Bool, '/drc_2015_environment/is_blackout', 1)
        self.pub_next_whiteout_time = self.create_publisher(
            Time, '/drc_2015_environment/next_whiteout_time', 1)
        # ROS 1 used a sequence of sleeps inside one loop; a timer stepping
        # through the same phases keeps the node responsive.
        self.phase = 0
        self.timer = self.create_timer(1.0, self._timer_cb)

    def _timer_cb(self):
        if self.phase == 0:
            self.pub_is_blackout.publish(Bool(data=False))
            self.pub_is_disabled.publish(Bool(data=False))
        elif self.phase == 1:
            self.pub_is_disabled.publish(Bool(data=True))
        else:
            self.pub_is_disabled.publish(Bool(data=False))
            self.pub_is_blackout.publish(Bool(data=True))
            whiteout = self.get_clock().now() + Duration(seconds=2.0)
            # std_msgs/Time does not exist in ROS 2, so the stamp is published
            # as builtin_interfaces/Time directly
            self.pub_next_whiteout_time.publish(whiteout.to_msg())
        self.phase = (self.phase + 1) % 4


def main(args=None):
    rclpy.init(args=args)
    node = SampleDRCMiniMaxwell()
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
