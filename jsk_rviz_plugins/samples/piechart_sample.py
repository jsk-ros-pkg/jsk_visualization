#!/usr/bin/env python

import math
import rospy

from std_msgs.msg import Float32


class PieChartSample():

    def __init__(self):
        self.pub = rospy.Publisher(
            '/sample_piechart', Float32, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(0.1), self._timer_cb)
        self.count = 0

    def _timer_cb(self, event):
        msg = Float32()
        msg.data = abs(math.sin(3.14 * self.count / 100.0))
        self.count = self.count + 1
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('piechart_sample')
    app = PieChartSample()
    rospy.spin()
