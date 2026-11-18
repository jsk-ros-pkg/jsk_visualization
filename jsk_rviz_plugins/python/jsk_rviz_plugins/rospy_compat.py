# -*- coding: utf-8 -*-
"""ROS 1 only: `ros_compat` and `ROS_VERSION` for the samples of this
package when jsk_ros1_ros2_compat can not be imported (jsk_common < 3.0):
the functions of jsk_ros1_ros2_compat's rospy_rclpy_compat they use,
forwarded to rospy as rospy_rclpy_compat does on ROS 1."""

import rospy

ROS_VERSION = 1


class ros_compat(object):
    get_param = staticmethod(lambda node, name, default=None: rospy.get_param(name, default))
    init_node = staticmethod(lambda name, anonymous=False, multi_threaded=False: rospy.init_node(name, anonymous=anonymous))
    spin = staticmethod(lambda node: rospy.spin())
    shutdown = staticmethod(lambda node: None)
    loginfo = staticmethod(lambda node, msg: rospy.loginfo(msg))
    logwarn = staticmethod(lambda node, msg: rospy.logwarn(msg))
    logerr = staticmethod(lambda node, msg: rospy.logerr(msg))
    logdebug = staticmethod(lambda node, msg: rospy.logdebug(msg))
    logfatal = staticmethod(lambda node, msg: rospy.logfatal(msg))
    create_publisher = staticmethod(lambda node, topic, msg_type, queue_size=1, latch=False: rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch))
    create_timer = staticmethod(lambda node, period, callback: rospy.Timer(rospy.Duration(period), callback))
    stamp_now = staticmethod(lambda node: rospy.Time.now())
    Rate = staticmethod(lambda node, hz: rospy.Rate(hz))
    is_shutdown = staticmethod(lambda node: rospy.is_shutdown())
