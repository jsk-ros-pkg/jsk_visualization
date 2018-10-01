#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Time


def main():
    pub_is_disabled = rospy.Publisher(
        '/drc_2015_environment/is_disabled', Bool, queue_size=1)
    pub_is_blackout = rospy.Publisher(
        '/drc_2015_environment/is_blackout', Bool, queue_size=1)
    pub_next_whiteout_time = rospy.Publisher(
        '/drc_2015_environment/next_whiteout_time', Time, queue_size=1)
    pub_is_disabled.publish(False)
    pub_is_blackout.publish(False)

    rate = rospy.Rate(1 / 4.0)

    while not rospy.is_shutdown():
        pub_is_blackout.publish(False)
        rospy.sleep(1.0)
        pub_is_disabled.publish(True)
        rospy.sleep(1.0)
        pub_is_disabled.publish(False)
        pub_is_blackout.publish(True)
        pub_next_whiteout_time.publish(rospy.Time.now() + rospy.Duration(2.0))
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sample_drc_mini_maxwell')
    main()
