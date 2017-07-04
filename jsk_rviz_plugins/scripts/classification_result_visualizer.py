#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
import message_filters
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ClassificationResultVisualizer(ConnectionBasedTransport):
    def __init__(self):
        super(ClassificationResultVisualizer, self).__init__()
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.text_color = rospy.get_param("~text_color", [0.0, 0.0, 1.0, 1.0])
        self.text_offset = rospy.get_param("~text_offset", [0.0, 0.0, 0.07])
        self.text_size = rospy.get_param("~text_size", 0.05)
        self.pub_marker = self.advertise("~output", MarkerArray, queue_size=10)

    def subscribe(self):
        self.subscribers = [
            message_filters.Subscriber("~input/boxes", BoundingBoxArray),
            message_filters.Subscriber("~input/classes", ClassificationResult),
        ]
        self.sync = message_filters.TimeSynchronizer(self.subscribers, self.queue_size)
        self.sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()

    def callback(self, bbox, cls):
        msg = MarkerArray()
        for i, data in enumerate(zip(bbox.boxes, zip(cls.label_names, cls.label_proba))):
            b, c = data
            text = "%s (%.3f)" % c
            m = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.ADD,
                       header=bbox.header,
                       id=i,
                       pose=b.pose,
                       color=ColorRGBA(self.text_color[0], self.text_color[1],
                                       self.text_color[2], self.text_color[3]),
                       text=text,
                       ns=cls.classifier,
                       lifetime=rospy.Time(30))
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
