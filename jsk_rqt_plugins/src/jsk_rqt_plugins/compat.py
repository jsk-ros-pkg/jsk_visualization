# -*- coding: utf-8 -*-
"""`ros_compat` and `ROS_VERSION` for the plugins and sample scripts of this
package: jsk_ros1_ros2_compat's rospy_rclpy_compat (ROS1/ROS2), or, as
jsk_common < 3.0 has no jsk_ros1_ros2_compat, the same functions forwarded
to rospy (ROS1 only)."""

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat  # NOQA
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION  # NOQA
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import roslib.message
    import rospy
    ROS_VERSION = 1

    def _resolve_message_class(type_str):
        pkg, _, name = type_str.replace('/msg/', '/').rpartition('/')
        return roslib.message.get_message_class('{}/{}'.format(pkg, name))

    class ros_compat(object):
        init_node = staticmethod(lambda name, anonymous=False, multi_threaded=False: rospy.init_node(name, anonymous=anonymous))
        spin = staticmethod(lambda node: rospy.spin())
        get_name = staticmethod(lambda node: rospy.get_name())
        get_param = staticmethod(lambda node, name, default=None: rospy.get_param(name, default))
        loginfo = staticmethod(lambda node, msg: rospy.loginfo(msg))
        logwarn = staticmethod(lambda node, msg: rospy.logwarn(msg))
        logerr = staticmethod(lambda node, msg: rospy.logerr(msg))
        logdebug = staticmethod(lambda node, msg: rospy.logdebug(msg))
        create_publisher = staticmethod(lambda node, topic, msg_type, queue_size=1, latch=False: rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch))
        create_subscription = staticmethod(lambda node, topic, msg_type, callback, queue_size=None, latch=False: rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size))
        create_service = staticmethod(lambda node, name, srv_type, callback: rospy.Service(name, srv_type, lambda req: callback(req, srv_type._response_class())))
        now_sec = staticmethod(lambda node: rospy.Time.now().to_sec())
        stamp_now = staticmethod(lambda node: rospy.Time.now())
        stamp_to_sec = staticmethod(lambda stamp: stamp.to_sec())
        duration_from_sec = staticmethod(lambda seconds: rospy.Duration(seconds))
        Rate = staticmethod(lambda node, hz: rospy.Rate(hz))
        sleep = staticmethod(lambda node, seconds: rospy.sleep(seconds))
        is_shutdown = staticmethod(lambda node: rospy.is_shutdown())
        resolve_message_class = staticmethod(_resolve_message_class)
