#!/usr/bin/env python

import sys
import unittest

import rosgraph
import rospy
import rosunit
# from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
# from std_msgs.msg import String
from std_msgs.msg import Time
# from std_msgs.msg import UInt8

from jsk_recognition_msgs.msg import HistogramWithRange
from jsk_recognition_msgs.msg import PlotData

NAME = "test_rqt_plugins"
WAIT_TIMEOUT = 10.0  # sec


class TestRqtPlugins(unittest.TestCase):

    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        rospy.init_node(NAME)

    def is_subscribed(self, topic_name, topic_type, sub_node_name):
        try:
            rospy.wait_for_message(topic_name, topic_type, WAIT_TIMEOUT)
        except rospy.ROSException:
            self.assertTrue(
                False, 'Could not subscribe topic ({}) in {} seconds'.
                format(topic_name, WAIT_TIMEOUT))
        subs = []
        i = 0
        while subs == [] and i < 10:
            pubs, subs, _ = rosgraph.Master('/rostopic').getSystemState()
            subs = [x[1] for x in subs if x[0] == topic_name]
            rospy.sleep(1.0)
            i = i + 1
        if subs == []:
            self.assertTrue(
                False, 'No node subscribes topic ({})'.format(topic_name))
        if sub_node_name not in subs[0]:
            self.assertTrue(
                False, 'Node ({}) does not subscribe topic ({}), but {} does'.
                format(sub_node_name, topic_name, subs))
        return True

    def test_rqt_2d_plot(self):
        self.assertTrue(
            self.is_subscribed(
                '/pub_sample_2d_data/output', PlotData, '/rqt_2d_plot'))

    def test_rqt_3d_plot(self):
        self.assertTrue(
            self.is_subscribed(
                '/pub_sample_3d_data/output1', Float32, '/rqt_3d_plot'))

    def test_rqt_drc_mini_maxwell(self):
        self.assertTrue(
            self.is_subscribed('/drc_2015_environment/is_disabled', Bool,
                               '/rqt_drc_mini_maxwell'))
        self.assertTrue(
            self.is_subscribed('/drc_2015_environment/is_blackout', Bool,
                               '/rqt_drc_mini_maxwell'))
        self.assertTrue(
            self.is_subscribed(
                '/drc_2015_environment/next_whiteout_time', Time,
                '/rqt_drc_mini_maxwell'))

    def test_rqt_histogram_plot(self):
        self.assertTrue(
            self.is_subscribed('/range_array', HistogramWithRange,
                               '/rqt_histogram_plot'))

    def test_rqt_image_view2(self):
        # FIXME: Load settings to read topic: /pub_sample_image/image_raw
        #        in default.
        # self.assertTrue(
        #     self.is_subscribed('/pub_sample_image/image_raw/marked', Image,
        #                        '/rqt_image_view2'))
        pass

    def test_rqt_service_buttons(self):
        # FIXME: Add some test here
        pass

    def test_rqt_status_light(self):
        # FIXME: Load settings to read topic: /sample_status in default.
        # self.assertTrue(
        #     self.is_subscribed('/sample_status', UInt8, '/rqt_status_light'))
        pass

    def test_rqt_string_label(self):
        # FIXME: Load settings to read topic: /sample_string in default.
        # self.assertTrue(
        #     self.is_subscribed('/sample_string', String,
        #                        '/rqt_string_label'))
        pass

    def test_rqt_yn_btn(self):
        try:
            rospy.wait_for_service('rqt_yn_btn', WAIT_TIMEOUT)
            self.assertTrue(True)
        except rospy.ROSException:
            self.assertTrue(False, 'Could not find service (rqt_yn_btn)')


if __name__ == '__main__':
    rosunit.unitrun(NAME, sys.argv[0], TestRqtPlugins)
