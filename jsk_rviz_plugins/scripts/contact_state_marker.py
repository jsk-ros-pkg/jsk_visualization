#!/usr/bin/env python

"""
Colorize link according to hrpsys_ros_bridge/ContactState using
visualization_msgs/Marker
"""

import rospy
from hrpsys_ros_bridge.msg import ContactState, ContactStateStamped, ContactStatesStamped
from visualization_msgs.msg import Marker, MarkerArray
from xml.dom.minidom import parse, parseString

def find_mesh(link_name):
    "find mesh file from specified link_name"
    for link in g_links:
        if link_name == link.getAttribute('name'):
            visual_mesh = link.getElementsByTagName('visual').item(0).getElementsByTagName('mesh').item(0)
            return visual_mesh.getAttribute('filename')
    raise Exception("Cannot find link: {0}".format(link_name))

def callback(msgs):
    "msgs = ContactStatesStamped"
    marker_array = MarkerArray()
    for msg, i in zip(msgs.states, range(len(msgs.states))):
        marker = Marker()
        mesh_file = find_mesh(msg.header.frame_id)
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        marker.color.a = alpha
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.mesh_resource = mesh_file
        marker.frame_locked = True
        marker.id = i
        if msg.state.state == ContactState.OFF:
            marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node("contact_state_marker")
    rgb = rospy.get_param("~rgb", [1, 0, 0])
    alpha = rospy.get_param("~alpha", 0.5)
    scale = rospy.get_param("~scale", 1.02)
    robot_description = rospy.get_param("/robot_description")
    
    # Parse robot_description using minidom directly
    # because urdf_parser_py cannot read PR2 urdf
    doc = parseString(robot_description)
    g_links = doc.getElementsByTagName('link')
    
    pub = rospy.Publisher('~marker', MarkerArray)
    sub = rospy.Subscriber("~input", ContactStatesStamped, callback)
    rospy.spin()
