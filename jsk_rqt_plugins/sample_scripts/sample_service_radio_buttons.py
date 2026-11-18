#!/usr/bin/env python

from jsk_rqt_plugins.compat import ros_compat

from std_srvs.srv import Empty


class SampleServiceRadioButtons(object):
    def __init__(self):
        self.services = [
            ros_compat.create_service(node, 'dummy/buttonA', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonB', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonC', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonD', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonE', Empty, self._empty_cb),
            ros_compat.create_service(node, 'dummy/buttonF', Empty, self._empty_cb),
        ]
        self._name = ros_compat.get_name(node)

    def _empty_cb(self, req, res):
        ros_compat.loginfo(node, '{} | Empty service called'.format(self._name))
        return res


if __name__ == '__main__':
    node = ros_compat.init_node('sample_service_radio_buttons')
    sample = SampleServiceRadioButtons()
    ros_compat.spin(node)
