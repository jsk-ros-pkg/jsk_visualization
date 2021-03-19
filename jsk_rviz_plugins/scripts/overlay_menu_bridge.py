#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <me@furushchev.ru>


import rospy
from jsk_rviz_plugins.msg import OverlayMenu


class OverlayMenuBridge(object):
    def __init__(self):
        super(OverlayMenuBridge, self).__init__()

        if OverlayMenu._md5sum == 'fed3c7e9788f7ee37908107a2597b619':
            rospy.logwarn('This script is not necessary since md5sum of OverlayMenu is the same as old one.')

        self.queue_size = rospy.get_param('~queue_size', 10)
        self.remap_suffix = rospy.get_param('~remap_suffix', 'kinetic')
        self.publishers = {}
        self.subscribers = {}

        poll_rate = rospy.get_param('~poll_rate', 1.0)
        self.poll_timer = rospy.Timer(rospy.Duration(1.0 / poll_rate), self.timerCallback)

    def remap(self, topic):
        return topic + '/' + self.remap_suffix

    def messageCallback(self, msg, topic):
        try:
            self.publishers[self.remap(topic)].publish(msg)
        except Exception as exc:
            rospy.logerr('Error on publishing to {}: {}'.format(topic, exc))

    def timerCallback(self, event):
        topics = [i[0] for i in rospy.get_published_topics() if i[1] == OverlayMenu._type]
        subscribed_topics = self.subscribers.keys()
        managed_topics = subscribed_topics + self.publishers.keys()
        for topic in topics:
            if topic not in managed_topics:
                self.publishers[self.remap(topic)] = rospy.Publisher(
                    self.remap(topic), OverlayMenu, queue_size=self.queue_size)
                self.subscribers[topic] = rospy.Subscriber(
                    topic, rospy.AnyMsg, self.messageCallback, topic,
                    queue_size=self.queue_size)

                rospy.loginfo('Remapped {} -> {}'.format(topic, self.remap(topic)))

        for topic in subscribed_topics:
            if topic not in topics:
                sub = self.subscribers.pop(topic)
                sub.unregister()
                pub = self.publishers.pop(self.remap(topic))
                pub.unregister()

                rospy.loginfo('Stopped Remap {} -> {}'.format(topic, self.remap(topic)))


if __name__ == '__main__':
    rospy.init_node('overlay_menu_bridge')
    b = OverlayMenuBridge()
    rospy.spin()
