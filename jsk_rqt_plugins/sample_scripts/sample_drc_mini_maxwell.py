#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
from std_msgs.msg import Bool
if ROS_VERSION == 2:
    # std_msgs/Time does not exist on ROS 2, builtin_interfaces/Time is the
    # time itself
    from builtin_interfaces.msg import Time
else:
    import rospy
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
        if ROS_VERSION == 2:
            pub_next_whiteout_time.publish(
                (node.get_clock().now()
                 + ros_compat.duration_from_sec(2.0)).to_msg())
        else:
            pub_next_whiteout_time.publish(
                rospy.Time.now() + rospy.Duration(2.0))
        rate.sleep()


if __name__ == '__main__':
    node = ros_compat.init_node('sample_drc_mini_maxwell')
    main()
