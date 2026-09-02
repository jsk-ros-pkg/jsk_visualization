#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SampleServiceRadioButtons(Node):

    def __init__(self):
        super().__init__('sample_service_radio_buttons')
        self._services = [
            self.create_service(Empty, 'dummy/button' + name, self._empty_cb)
            for name in ['A', 'B', 'C', 'D', 'E', 'F']
        ]

    def _empty_cb(self, req, res):
        self.get_logger().info(
            '{} | Empty service called'.format(self.get_name()))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SampleServiceRadioButtons()
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
