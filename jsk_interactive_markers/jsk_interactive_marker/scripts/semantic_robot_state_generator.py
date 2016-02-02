#!/usr/bin/env python

# simple script to generate srdf parameter on the fly
# with only world virtual joint
import rospy
from xml.dom.minidom import parseString
if __name__ == "__main__":
    rospy.init_node("semantic_robot_state_generator")
    root_link = rospy.get_param("~root_link", "BODY")
    global_frame = rospy.get_param("~global_frame", "odom")
    robot_description = rospy.get_param("/robot_description")
    robot_description_doc = parseString(robot_description)
    robot_name = robot_description_doc.getElementsByTagName("robot")[0].getAttribute("name")

    srdf_str = """<?xml version="1.0" ?>
    <robot name="%s">
    <virtual_joint name="world_joint" type="floating" parent_frame="%s" child_link="%s" />
    <passive_joint name="world_joint" />
    </robot>
    """ % (robot_name, global_frame, root_link)
    rospy.set_param("/robot_description_semantic", srdf_str)

