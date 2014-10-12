#!/usr/bin/env python

from hrpsys_ros_bridge.msgs import MotorStates
#from sensor_msgs.msg import JointState as MotorStates
from std_msgs.msg import Float32
import rospy

g_publishers = []

def allocatePublishers(num):
    global g_publishers
    if num > len(g_publishers):
        for i in range(len(g_publishers), num):
            pub = rospy.Publisher('temperature_%02d' % (i), Float32)
            g_publishers.append(pub)
        
def motorStatesCallback(msg):
    global g_publishers
    #values = msg.position
    values = msg.temperature
    allocatePublishers(len(values))
    for val, pub in zip(values, g_publishers):
        pub.publish(Float32(val))

if __name__ == "__main__":
    rospy.init_node("motor_state_temperature_decomposer")
    s = rospy.Subscriber("motor_states", MotorStates, motorStatesCallback)
    #s = rospy.Subscriber("joint_states", MotorStates, motorStatesCallback)
    rospy.spin()
    
