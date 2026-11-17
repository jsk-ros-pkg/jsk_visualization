#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat

from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


class SampleServiceButtons(object):
    def __init__(self):
        self.services = [
            ros_compat.create_service(node, 'dummy/buttonA', SetBool, self._set_bool_cb),
            ros_compat.create_service(node, 'dummy/buttonB', SetBool, self._set_bool_cb),
            ros_compat.create_service(node, 'dummy/buttonC', SetBool, self._set_bool_cb),
            ros_compat.create_service(node, 'dummy/buttonD', Trigger, self._trigger_cb),
            ros_compat.create_service(node, 'dummy/buttonE', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonF', Empty, self._empty_cb),
        ]
        self._name = ros_compat.get_name(node)

    def _set_bool_cb(self, req, res):
        ros_compat.loginfo(node, '{} | SetBool service called: req.data={}'.format(self._name,req.data))
        res.success = True
        return res

    def _trigger_cb(self, req, res):
        ros_compat.loginfo(node, '{} | Trigger service called'.format(self._name))
        res.success = True
        return res

    def _empty_cb(self, req, res):
        ros_compat.loginfo(node, '{} | Empty service called'.format(self._name))
        return res


if __name__ == '__main__':
    node = ros_compat.init_node('sample_service_buttons')
    sample = SampleServiceButtons()
    ros_compat.spin(node)
