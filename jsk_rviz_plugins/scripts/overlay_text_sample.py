#!/usr/bin/env pythnon

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA
import rospy
import math
rospy.init_node("aaahoge")

pub = rospy.Publisher("text_sample", OverlayText)
counter = 0
r = rospy.Rate(20)
while not rospy.is_shutdown():
  text = OverlayText()
  text.header.frame_id = "base_link"
  text.header.stamp = rospy.Time.now()
  theta = counter % 255 / 255.0
  text.width = 0.5
  text.height = 0.2
  text.left = math.cos(theta) * 0.5
  text.top = math.sin(theta) * 0.5
  text.text_size = 0.05
  text.text = "Hello World %d          " % (counter)
  #text.fg_color = ColorRGBA((counter % 255) / 255.0, 0.0, 0.0, 0.5)
  text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
  text.bg_color = ColorRGBA(1.0, 1.0, (counter % 255) / 255.0, 0.0)
  pub.publish(text)
  counter = counter + 1
  r.sleep()

