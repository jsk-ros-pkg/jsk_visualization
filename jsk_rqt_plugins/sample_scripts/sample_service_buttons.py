#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


class SampleServiceButtons(Node):

    def __init__(self):
        super().__init__('sample_service_buttons')
        self._services = [
            self.create_service(SetBool, 'dummy/buttonA', self._set_bool_cb),
            self.create_service(SetBool, 'dummy/buttonB', self._set_bool_cb),
            self.create_service(SetBool, 'dummy/buttonC', self._set_bool_cb),
            self.create_service(Trigger, 'dummy/buttonD', self._trigger_cb),
            self.create_service(Empty, 'dummy/buttonE', self._empty_cb),
            self.create_service(Empty, 'dummy/buttonF', self._empty_cb),
        ]

    def _set_bool_cb(self, req, res):
        self.get_logger().info(
            '{} | SetBool service called: req.data={}'.format(
                self.get_name(), req.data))
        res.success = True
        return res

    def _trigger_cb(self, req, res):
        self.get_logger().info(
            '{} | Trigger service called'.format(self.get_name()))
        res.success = True
        return res

    def _empty_cb(self, req, res):
        self.get_logger().info(
            '{} | Empty service called'.format(self.get_name()))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SampleServiceButtons()
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
