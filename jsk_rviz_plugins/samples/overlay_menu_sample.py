#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayMenu

rospy.init_node("test_menu")
p = rospy.Publisher("test_menu", OverlayMenu, queue_size=1)
r = rospy.Rate(5)
counter = 0
while not rospy.is_shutdown():
  menu = OverlayMenu()
  menu.title = "The Beatles"
  menu.menus = ["John Lennon", "Paul McCartney", "George Harrison",
                "Ringo Starr"]
  menu.current_index = counter % len(menu.menus)
  if counter % 100 == 0:
    menu.action = OverlayMenu.ACTION_CLOSE
  menu.fg_color.r = 1.0
  menu.fg_color.g = 1.0
  menu.fg_color.b = 1.0
  menu.fg_color.a = 1.0
  menu.bg_color.r = 0.0
  menu.bg_color.g = 0.0
  menu.bg_color.b = 0.0
  menu.bg_color.a = 1.0
  p.publish(menu)
  counter = counter + 1
  r.sleep()
