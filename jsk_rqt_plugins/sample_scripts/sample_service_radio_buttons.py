#!/usr/bin/env python

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class SampleServiceRadioButtons(object):
    def __init__(self):
        self.services = [
            rospy.Service('dummy/buttonA', Empty, self._empty_cb),
            rospy.Service('dummy/buttonB', Empty, self._empty_cb),
            rospy.Service('dummy/buttonC', Empty, self._empty_cb),
            rospy.Service('dummy/buttonD', Empty, self._empty_cb),
            rospy.Service('dummy/buttonE', Empty, self._empty_cb),
            rospy.Service('dummy/buttonF', Empty, self._empty_cb),
        ]

    def _empty_cb(self, req):
        rospy.loginfo('Empty service called')
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('sample_service_radio_buttons')
    sample = SampleServiceRadioButtons()
    rospy.spin()
