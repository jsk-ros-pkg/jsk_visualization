#!/usr/bin/env python


import roslib; roslib.load_manifest('jsk_interactive_test')
import rospy
import copy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf import transformations
import numpy

class InteractiveMarkerTest:
    poses = []
    colors = []
    status = []
    size = 2
    start_time = False

    def processFeedback(self, feedback):
        if not self.start_time:
            self.start_time = rospy.get_rostime()
        self.feedback = feedback
        size = self.size
        i = min(range(size*size*size),key=lambda i:numpy.linalg.norm(numpy.array([self.poses[i].position.x,self.poses[i].position.y,self.poses[i].position.z])-
                                                                     numpy.array([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z])))
        p = self.poses[i]
        pos = numpy.linalg.norm(numpy.array([p.position.x,p.position.y,p.position.z])-
                                numpy.array([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z]))
        rot = numpy.linalg.norm(numpy.array(transformations.euler_from_quaternion([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))-
                                numpy.array(transformations.euler_from_quaternion([feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w])))
        if pos < 0.010 and rot < numpy.deg2rad(2): # 1 [cm], 2 [deg]
            self.colors[i] = ColorRGBA(1.0,0.2,0.2,0.8)
            self.status[i] = rospy.get_rostime() - self.start_time
    def poseCB(self, msg):
        self.server.setPose('control', msg.pose, msg.header)
        self.server.applyChanges()
        self.processFeedback(msg)
    def __init__(self):
        pub_goal = rospy.Publisher('interactive_goal_marker', MarkerArray)
        rospy.init_node('publish_interactive_goal_marker')
        # setup
        size = self.size
        space = 0.5
        mesh = 'file://'+roslib.packages.get_pkg_dir('jsk_interactive_test')+'/scripts/RobotHand.dae'
        mesh_scale = Vector3(0.2, 0.2, 0.2)
        rospy.Subscriber('/goal_marker/move_marker', PoseStamped, self.poseCB)
        #
        for i in range(size*size*size):
            x = space * ((i%(size) - size/2) + numpy.random.normal(0,0.1))
            y = space * (((i/(size))%size - size/2) + numpy.random.normal(0,0.1))
            z = space * ((i/(size*size) - size/2) + numpy.random.normal(0,0.1))
            # q = transformations.quaternion_from_euler(*numpy.random.rand(3))
            # q = transformations.quaternion_from_euler(*numpy.random.normal(0,0.01,(1,3))[0])
            q = transformations.quaternion_from_euler(0,0,numpy.random.normal(0,0.5))
            self.poses.append(Pose(Point(x,y,z), Quaternion(*q)))
            self.colors.append(ColorRGBA(0.8,0.8,0.8,0.8))
            self.status.append(False)

        # interactive marker
        server = InteractiveMarkerServer("goal_marker")
        self.server = server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "control"
        int_marker.scale = 0.3

        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale = mesh_scale
        mesh_marker.color = ColorRGBA(0.2,0.2,1.0,0.8)
        mesh_marker.mesh_resource = mesh

        mesh_control = InteractiveMarkerControl()
        mesh_control.always_visible = True
        mesh_control.markers.append(mesh_marker)

        int_marker.controls.append(mesh_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        server.insert(int_marker, self.processFeedback)
        server.applyChanges()

        # start rosmain
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            array = MarkerArray()
            for i in range(size*size*size):
                msg = Marker()
                msg.header.stamp = rospy.get_rostime()
                msg.header.frame_id = '/map'
                msg.mesh_use_embedded_materials = False
                msg.mesh_resource = mesh
                msg.type = Marker.MESH_RESOURCE
                msg.scale = mesh_scale
                msg.color = self.colors[i]
                msg.pose = self.poses[i]
                msg.lifetime = rospy.Time(10)
                msg.id = i
                array.markers.append(msg)
            pub_goal.publish(array)
            r.sleep()
            if self.start_time:
                rospy.loginfo("%2d/%d %5.2f"%(len(filter(lambda x: x != False, self.status)),len(self.status),float((rospy.get_rostime()-self.start_time).to_sec())))
            if all(self.status):
                rospy.loginfo("Done..")
                for s in self.status:
                    rospy.loginfo("%5.2f"%s.to_sec())


if __name__ == '__main__':
    try:
        i = InteractiveMarkerTest()
    except rospy.ROSInterruptException:
        pass

