#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from dynamic_reconfigure.server import Server
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_rviz_plugins.cfg import ClassificationResultVisualizerConfig
import message_filters
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ClassificationResultVisualizer(ConnectionBasedTransport):
    def __init__(self):
        super(ClassificationResultVisualizer, self).__init__()
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.srv = Server(ClassificationResultVisualizerConfig,
                          self.config_callback)
        self.pub_marker = self.advertise("~output", MarkerArray, queue_size=10)

    def subscribe(self):
        self.subscribers = [
            message_filters.Subscriber("~input/boxes", BoundingBoxArray),
            message_filters.Subscriber("~input/classes", ClassificationResult),
        ]
        self.sync = message_filters.TimeSynchronizer(self.subscribers, self.queue_size)
        self.sync.registerCallback(self.msg_callback)

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()

    def config_callback(self, config, level):
        self.text_color = {'r': config.text_color_red,
                           'g': config.text_color_green,
                           'b': config.text_color_blue,
                           'a': config.text_color_alpha}
        self.text_offset = [config.text_offset_x,
                            config.text_offset_y,
                            config.text_offset_z]
        self.text_size = config.text_size
        self.marker_lifetime = config.marker_lifetime
        return config

    def msg_callback(self, bboxes, classes):
        msg = MarkerArray()
        for i, data in enumerate(zip(bboxes.boxes, zip(classes.label_names, classes.label_proba))):
            bbox, cls = data
            text = "%s (%.3f)" % cls

            m = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.MODIFY,
                       header=bbox.header,
                       id=i,
                       pose=bbox.pose,
                       color=ColorRGBA(**self.text_color),
                       text=text,
                       ns=classes.classifier,
                       lifetime=rospy.Duration(self.marker_lifetime))
            m.scale.z = self.text_size
            m.pose.position.x += self.text_offset[0]
            m.pose.position.y += self.text_offset[1]
            m.pose.position.z += self.text_offset[2]
            msg.markers.append(m)

        self.pub_marker.publish(msg)


if __name__ == '__main__':
    rospy.init_node("classification_result_visualizer")
    viz = ClassificationResultVisualizer()
    rospy.spin()
