#!/usr/bin/env python

import rospy
from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from std_msgs.msg import Bool
from std_msgs.msg import Time


def main():
    pub_is_disabled = ros_compat.create_publisher(node,
        '/drc_2015_environment/is_disabled', Bool, queue_size=1)
    pub_is_blackout = ros_compat.create_publisher(node,
        '/drc_2015_environment/is_blackout', Bool, queue_size=1)
    pub_next_whiteout_time = ros_compat.create_publisher(node,
        '/drc_2015_environment/next_whiteout_time', Time, queue_size=1)
    pub_is_disabled.publish(Bool(data=False))
    pub_is_blackout.publish(Bool(data=False))

    rate = ros_compat.Rate(node, 1 / 4.0)

    while not ros_compat.is_shutdown(node):
        pub_is_blackout.publish(Bool(data=False))
        ros_compat.sleep(node, 1.0)
        pub_is_disabled.publish(Bool(data=True))
        ros_compat.sleep(node, 1.0)
        pub_is_disabled.publish(Bool(data=False))
        pub_is_blackout.publish(Bool(data=True))
        pub_next_whiteout_time.publish(rospy.Time.now() + rospy.Duration(2.0))
        rate.sleep()


if __name__ == '__main__':
    node = ros_compat.init_node('sample_drc_mini_maxwell')
    main()
