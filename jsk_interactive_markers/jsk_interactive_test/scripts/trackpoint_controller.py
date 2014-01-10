#!/usr/bin/env python

import math
import numpy
import commands
import sys

import rospy
import roslib

roslib.load_manifest('jsk_joy')

from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import tf.transformations

sys.path.append(commands.getoutput('rospack find jsk_joy') + '/scripts')
from trackpoint_status import TrackpointStatus
from nanokontrol_status import NanoKONTROL2Status
from nanopad_status import NanoPAD2Status

INTERACTIVE_MARKER_TOPIC = '/goal_marker'
pre_pose = None

def nanokontrol_joyCB(msg):
    global nanokontrol_st
    nanokontrol_st = NanoKONTROL2Status(msg)
    
def nanopad_joyCB(msg):
    global nanopad_st
    nanopad_st = NanoPAD2Status(msg)

def trackpoint_joyCB(msg):
    global pre_view, g_seq_num, pre_pose
    global nanokontrol_st, nanopad_st
    if not pre_pose:
        pre_pose = PoseStamped()
    status = TrackpointStatus(msg)

    # rospy.loginfo(status)
    # move interactive marker
    new_pose = PoseStamped()
    new_pose.header.frame_id = '/map'
    new_pose.header.stamp = rospy.Time(0.0)
    # move in local
    local_xy_move = numpy.array((0.0,
                                 0.0,
                                 0.0,
                                 1.0))
    z_move = 0
    yaw_move = 0
    if ( nanopad_st and nanopad_st.buttonL1 ):
        scale_xy = 1500.0
        scale_z = 1500.0
        scale_yaw = 1000.0
    else:
        scale_xy = 500.0
        scale_z = 500.0
        scale_yaw = 300.0

    if not ( status.left or status.right or status.middle ):
        # if ( nanokontrol_st and nanokontrol_st.S8 ):
        #     z_move = status.y / scale
        # elif ( nanokontrol_st and nanokontrol_st.R8 ):
        #     yaw_move =  status.y / scale
        if ( nanopad_st and nanopad_st.buttonU1 ):
            z_move = status.y / scale_z
        elif ( nanopad_st and nanopad_st.buttonU2 ):
            yaw_move =  status.y / scale_yaw
        else:
            local_xy_move = numpy.array((- status.x / scale_xy,
                                         - status.y / scale_xy,
                                         0.0,
                                         1.0))
    q = numpy.array((pre_pose.pose.orientation.x,
                     pre_pose.pose.orientation.y,
                     pre_pose.pose.orientation.z,
                     pre_pose.pose.orientation.w))
    # xy_move = numpy.dot(tf.transformations.quaternion_matrix(q),
    #                     local_xy_move)
    xy_move = local_xy_move

    new_pose.pose.position.x = pre_pose.pose.position.x + xy_move[0]
    new_pose.pose.position.y = pre_pose.pose.position.y + xy_move[1]
    new_pose.pose.position.z = pre_pose.pose.position.z + z_move
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    yaw = yaw + yaw_move

    new_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    new_pose.pose.orientation.x = new_q[0]
    new_pose.pose.orientation.y = new_q[1]
    new_pose.pose.orientation.z = new_q[2]
    new_pose.pose.orientation.w = new_q[3]
    g_interactive_pub.publish(new_pose)
    pre_pose = new_pose
  
    # view update
    # view = ViewProperty()
    # view.focus = numpy.copy(pre_view.focus)
    # view.yaw = pre_view.yaw
    # view.pitch = pre_view.pitch
    # view.distance = pre_view.distance
    # if status.R3:
    #   view.distance = view.distance - status.left_analog_y * 0.1
    #   # calc camera orietation
    #   if status.left:
    #     view_x = 1.0
    #   elif status.right:
    #     view_x = -1.0
    #   else:
    #     view_x = 0.0
    #   if status.up:
    #     view_y = 1.0
    #   elif status.down:
    #     view_y = -1.0
    #   else:
    #     view_y = 0.0
    #   focus_diff = numpy.dot(view.cameraOrientation(),
    #                          numpy.array((view_x / 10.0 / view.distance,
    #                                       view_y / 10.0 / view.distance,
    #                                       0)))
    #   view.focus = view.focus + focus_diff
    #   if status.L2 and status.R2:           #align to marker
    #     view.distance = 1.0
    #     view.focus = numpy.array((new_pose.pose.position.x,
    #                               new_pose.pose.position.y,
    #                               new_pose.pose.position.z))
    #     #view.yaw = math.pi
    #     view.yaw = yaw + math.pi
    #     view.pitch = math.pi / 2.0 - 0.01
    # else:
    #   view.yaw = view.yaw - 0.5 * status.right_analog_x
    #   view.pitch = view.pitch + 0.5 * status.right_analog_y
    #   if view.pitch > math.pi / 2.0 - 0.01:
    #     view.pitch = math.pi / 2.0 - 0.01
    #   elif view.pitch < - math.pi / 2.0 + 0.01:
    #     view.pitch = - math.pi / 2.0 + 0.01
    # g_camera_pub.publish(view.cameraPlacement())
    # pre_view = view


def main():
    global g_camera_pub, g_interactive_pub
    global nanokontrol_st, nanopad_st
    rospy.sleep(1)
    rospy.init_node('trackpoint_controller')
    g_camera_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement)
    g_interactive_pub = rospy.Publisher(INTERACTIVE_MARKER_TOPIC + '/move_marker', PoseStamped)
    trackpoint_sub = rospy.Subscriber('/trackpoint/joy', Joy, trackpoint_joyCB)
    nanokontrol_sub= rospy.Subscriber('/nanokontrol/joy', Joy, nanokontrol_joyCB)
    nanopad_sub= rospy.Subscriber('/nanopad/joy', Joy, nanopad_joyCB)
    nanokontrol_st = None
    nanopad_st = None
        
    rospy.spin()
        

if __name__ == '__main__':
    main()
    
