#!/usr/bin/env python

try:
  from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
  # jsk_common < 3.0 has no jsk_ros1_ros2_compat (ROS 1 only)
  from jsk_rviz_plugins.rospy_compat import ros_compat
from jsk_rviz_plugins.msg import OverlayMenu
from sample_utils import exit_on_shutdown

node = ros_compat.init_node("test_menu")
p = ros_compat.create_publisher(node, "test_menu", OverlayMenu, queue_size=1)
r = ros_compat.Rate(node, 5)
counter = 0
with exit_on_shutdown():
  while not ros_compat.is_shutdown(node):
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
ros_compat.shutdown(node)
