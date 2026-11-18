#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32
try:
  from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
  from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
  # jsk_common < 3.0 has no jsk_ros1_ros2_compat (ROS 1 only)
  from jsk_rviz_plugins.rospy_compat import ros_compat, ROS_VERSION
import math
node = ros_compat.init_node("overlay_sample")

text_pub = ros_compat.create_publisher(node, "text_sample", OverlayText, queue_size=1)
value_pub = ros_compat.create_publisher(node, "value_sample", Float32, queue_size=1)
if ROS_VERSION == 2:
  # OverlayText is in rviz_2d_overlay_plugins on ROS 2, so the ROS 2
  # config/ros2/overlay_sample.rviz shows an OverlayDiagnostic instead,
  # whose diagnostics are published here, and the String display, whose
  # /sample_string overlay_sample.launch publishes with `rostopic pub`
  # (`ros2 topic pub` exits with an error on SIGINT).
  from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
  from std_msgs.msg import String
  diagnostics_pub = ros_compat.create_publisher(node, "diagnostics_agg", DiagnosticArray, queue_size=1)
  string_pub = ros_compat.create_publisher(node, "/sample_string", String, queue_size=1)
counter = 0
rate = 100
r = ros_compat.Rate(node, rate)
import random, math
while not ros_compat.is_shutdown(node):
  counter = counter + 1
  text = OverlayText()
  theta = counter % 255 / 255.0
  text.width = 400
  text.height = 600
  #text.height = 600
  text.left = 10
  text.top = 10
  text.text_size = 12.0  # float32, which rclpy does not fill from an int
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  text.text = """This is OverlayText plugin.
The update rate is %d Hz.
You can write several text to show to the operators.
New line is supported and automatical wrapping text is also supported.
And you can choose font, this text is now rendered by '%s'

You can specify background color and foreground color separatelly.

Of course, the text is not needed to be fixed, see the counter: %d.

You can change text color like <span style="color: red;">this</span>
by using <span style="font-style: italic;">css</style>.
  """ % (rate, text.font, counter)
  text.fg_color = ColorRGBA(r=25 / 255.0, g=1.0, b=240.0 / 255.0, a=1.0)
  text.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)
  text_pub.publish(text)
  value_pub.publish(Float32(data=math.sin(counter * math.pi * 2 / 100)))
  if ROS_VERSION == 2:
    status = DiagnosticStatus(name="/sample_diagnostics", hardware_id="overlay_sample")
    phase = int(counter / 100) % 3
    if phase == 0:
      status.level, status.message = DiagnosticStatus.OK, "Everything is fine"
    elif phase == 1:
      status.level, status.message = DiagnosticStatus.WARN, "Something looks odd"
    else:
      status.level, status.message = DiagnosticStatus.ERROR, "Something is broken"
    diagnostics_pub.publish(DiagnosticArray(status=[status]))
    if counter % rate == 0:
      string_pub.publish(String(data="This is a sample message of type std_msgs/String."))
  if int(counter % 500) == 0:
    ros_compat.logdebug(node, 'This is ROS_DEBUG.')
  elif int(counter % 500) == 100:
    ros_compat.loginfo(node, 'This is ROS_INFO.')
  elif int(counter % 500) == 200:
    ros_compat.logwarn(node, 'This is ROS_WARN.')
  elif int(counter % 500) == 300:
    ros_compat.logerr(node, 'This is ROS_ERROR.')
  elif int(counter % 500) == 400:
    ros_compat.logfatal(node, 'This is ROS_FATAL.')
  r.sleep()
ros_compat.shutdown(node)

