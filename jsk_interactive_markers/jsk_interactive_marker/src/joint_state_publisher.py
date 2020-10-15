#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

joint_states = None
def callback(msg):
    global joint_states
    print(msg)
    joint_states = msg

def joint_state_publisher():
    global joint_states
    rospy.init_node('joint_states_latcher', anonymous=True)
    rospy.Subscriber("joint_states_sub", JointState, callback)
    pub = rospy.Publisher('joint_states_pub', JointState)
    while not rospy.is_shutdown():
        if joint_states:
            print(joint_states)
            joint_states.header.stamp = rospy.Time.now()
            pub.publish(joint_states)
        rospy.sleep(0.1)

if __name__ == "__main__":
    joint_state_publisher()



