#!/usr/bin/env python

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class SampleServiceButtons(object):
    def __init__(self):
        self.services = [
            rospy.Service('dummy/buttonA', SetBool, self._set_bool_cb),
            rospy.Service('dummy/buttonB', SetBool, self._set_bool_cb),
            rospy.Service('dummy/buttonC', SetBool, self._set_bool_cb),
            rospy.Service('dummy/buttonD', Trigger, self._trigger_cb),
            rospy.Service('dummy/buttonE', Empty, self._empty_cb),
            rospy.Service('dummy/buttonF', Empty, self._empty_cb),
        ]
        self._name = rospy.get_name()

    def _set_bool_cb(self, req):
        rospy.loginfo('{} | SetBool service called: req.data={}'.format(self._name,req.data))
        return SetBoolResponse(success=True)

    def _trigger_cb(self, req):
        rospy.loginfo('{} | Trigger service called'.format(self._name))
        return TriggerResponse(success=True)

    def _empty_cb(self, req):
        rospy.loginfo('{} | Empty service called'.format(self._name))
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('sample_service_buttons')
    sample = SampleServiceButtons()
    rospy.spin()
