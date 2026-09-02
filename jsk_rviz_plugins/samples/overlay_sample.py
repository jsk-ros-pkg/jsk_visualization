#!/usr/bin/env python3

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class OverlaySample(Node):
    """Publish the topics shown by config/overlay_sample.rviz."""

    def __init__(self):
        super().__init__('overlay_sample')
        self.string_pub = self.create_publisher(String, 'sample_string', 1)
        self.value_pub = self.create_publisher(Float32, 'value_sample', 1)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics_agg', 1)
        self.counter = 0
        self.rate = 100
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

    def _timer_cb(self):
        self.counter += 1

        text = String()
        text.data = (
            'This is jsk_rviz_plugin/String.\n'
            'The update rate is {} Hz.\n'
            'New line is supported and automatic text wrapping is also supported.\n'
            'Of course the text is not needed to be fixed, see the counter: {}.'
        ).format(self.rate, self.counter)
        self.string_pub.publish(text)

        value = Float32()
        value.data = math.sin(self.counter * math.pi * 2 / 100)
        self.value_pub.publish(value)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = '/sample_diagnostics'
        status.hardware_id = 'overlay_sample'
        phase = int(self.counter / 100) % 3
        if phase == 0:
            status.level = DiagnosticStatus.OK
            status.message = 'Everything is fine'
        elif phase == 1:
            status.level = DiagnosticStatus.WARN
            status.message = 'Something looks odd'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Something is broken'
        diagnostics.status.append(status)
        self.diagnostics_pub.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)
    node = OverlaySample()
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
