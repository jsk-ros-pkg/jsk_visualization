#!/usr/bin/env python

import re

from jsk_rviz_plugins.msg import OverlayText
from rosgraph_msgs.msg import Log
import rospy


def colored_message(msg):
    if msg.level == Log.DEBUG:
        return '<span style="color: rgb(120,120,120);">%s</span>' % (msg.msg)
    elif msg.level == Log.INFO:
        return '<span style="color: white;">%s</span>' % (msg.msg)
    elif msg.level == Log.WARN:
        return '<span style="color: yellow;">%s</span>' % (msg.msg)
    elif msg.level == Log.ERROR:
        return '<span style="color: red;">%s</span>' % (msg.msg)
    elif msg.level == Log.FATAL:
        return '<span style="color: red;">%s</span>' % (msg.msg)


def callback(msg):
    global lines
    if msg.name not in ignore_nodes:
        if msg.name in nodes or len(nodes) == 0:
            if len(nodes_regexp) == 0 or nodes_regexp_compiled.match(msg.name):
                lines = [colored_message(msg)] + lines
                if len(lines) > line_buffer_length:
                    lines = lines[0:line_buffer_length]
                text = OverlayText()
                text.left = 20
                text.top = 20
                text.width = 1200
                text.height = 1200
                text.fg_color.a = 1.0
                text.fg_color.r = 0.3
                text.text_size = 12
                text.text = "\n".join(lines)
                pub.publish(text)


if __name__ == "__main__":
    rospy.init_node("rosconsole_overlay_text")
    nodes = rospy.get_param("~nodes", [])
    nodes_regexp = rospy.get_param("~nodes_regexp", "")
    if nodes_regexp:
        nodes_regexp_compiled = re.compile(nodes_regexp)
    ignore_nodes = rospy.get_param("~ignore_nodes", [])
    line_buffer_length = rospy.get_param("~line_buffer_length", 100)
    lines = []
    sub = rospy.Subscriber("/rosout", Log, callback)
    pub = rospy.Publisher("~output", OverlayText, queue_size=1)
    rospy.spin()
