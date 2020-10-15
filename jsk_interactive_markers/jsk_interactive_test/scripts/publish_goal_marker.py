#!/usr/bin/env python


import roslib; roslib.load_manifest('jsk_interactive_test')
import rospy
import copy
import httplib, urllib2, urllib, json
from socket import gethostname
from getpass import getuser


from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf import transformations
import numpy, math

class InteractiveMarkerTest:
    poses = []
    colors = []
    status = []
    size = 2
    start_time = False
    done_time = False

    def recordResult(self, ui_type, time):
        #url = 'http://localhost:3000/interactive_marker_test'
        url = 'http://jsk-db.herokuapp.com/interactive_marker_test'
        params = {'ui_type': ui_type,
                  'user': '%s@%s' % (getuser(), gethostname()),
                  'time': time}
        data = urllib.urlencode(params)
        req = urllib2.Request(url, data)
        try:
            response = urllib2.urlopen(req)
        except:
            rospy.logfatal('failed to record result')

    def displayResult(self):
        url = 'http://jsk-db.herokuapp.com/interactive_marker_test/json'
        req = urllib2.Request(url)
        try:
            response = urllib2.urlopen(req)
            data = json.loads(response.read())
            rospy.loginfo("======================================================")
            rospy.loginfo("   http://jsk-db.herokuapp.com/interactive_marker_test")
            rospy.loginfo("                user /  ui_type /    time")
            for d in data:
                rospy.loginfo("%12s / %8s / %s sec", d['user'], d['ui_type'], d['time'])
            rospy.loginfo("======================================================")
        except:
            rospy.logfatal('failed to record result')

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
        #print transformations.quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        #print transformations.quaternion_matrix([feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w])
        #print transformations.quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])[0:3,2]
        #print transformations.quaternion_matrix([feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w])[0:3,2]
        rot = math.acos(numpy.dot(transformations.quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])[0:3,2],
                                  transformations.quaternion_matrix([feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w])[0:3,2]))
        #rot = numpy.linalg.norm(rvec)
        rospy.loginfo('error %6.3f / %6.3f'%(pos, numpy.rad2deg(rot)))
        if pos < self.position_threshold and rot < numpy.deg2rad(self.rotation_threshold):
            self.colors[i] = ColorRGBA(1.0,0.2,0.2,0.8)
            self.status[i] = rospy.get_rostime() - self.start_time

    def poseCB(self, msg):
        self.server.setPose('control', msg.pose, msg.header)
        self.server.applyChanges()
        self.processFeedback(msg)

    def __init__(self):
        pub_goal = rospy.Publisher('interactive_goal_marker', MarkerArray)
        rospy.init_node('publish_interactive_goal_marker')
        self.ui_type = rospy.get_param('~ui_type')
        #
        self.displayResult()
        # setup
        size = self.size
        space = 0.5
        hand_mesh = 'file://'+roslib.packages.get_pkg_dir('jsk_interactive_test')+'/scripts/RobotHand.dae'
        hand_mesh_scale = Vector3(0.2, 0.2, 0.2)
        goal_mesh = 'file://'+roslib.packages.get_pkg_dir('jsk_interactive_test')+'/scripts/Bottle.dae'
        goal_mesh_scale = Vector3(1.0, 1.0, 1.0)

        self.position_threshold = rospy.get_param('~position_threshold', 0.01) # 1 [cm]
        self.rotation_threshold = rospy.get_param('~rotation_threshold', 2)    # 2 [deg]

        rospy.Subscriber('/goal_marker/move_marker', PoseStamped, self.poseCB)
        #
        for i in range(size*size*size):
            x = space * ((i%(size) - size/2) + numpy.random.normal(0,0.1))
            y = space * (((i/(size))%size - size/2) + numpy.random.normal(0,0.1))
            z = space * ((i/(size*size) - size/2) + numpy.random.normal(0,0.1))
            q = transformations.quaternion_from_euler(*numpy.random.normal(0,0.7,3))
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
        mesh_marker.scale = hand_mesh_scale
        mesh_marker.color = ColorRGBA(0.2,0.2,1.0,0.8)
        mesh_marker.mesh_resource = hand_mesh

        target_marker = Marker()
        target_marker.type = Marker.CYLINDER
        target_marker.scale = Vector3(0.07,0.07,0.1)
        target_marker.color = ColorRGBA(0.4,0.4,1.0,0.6)

        mesh_control = InteractiveMarkerControl()
        mesh_control.always_visible = True
        mesh_control.markers.append(mesh_marker)
        mesh_control.markers.append(target_marker)

        int_marker.controls.append(mesh_control)

        if rospy.has_param(rospy.search_param('make_interactive_marker_arrow')) and rospy.get_param(rospy.search_param('make_interactive_marker_arrow')):
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
        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            array = MarkerArray()
            for i in range(size*size*size):
                msg = Marker()
                msg.header.stamp = rospy.get_rostime()
                msg.header.frame_id = '/map'
                msg.mesh_use_embedded_materials = False
                msg.mesh_resource = goal_mesh
                msg.type = Marker.MESH_RESOURCE
                msg.scale = goal_mesh_scale
                msg.color = self.colors[i]
                msg.pose = self.poses[i]
                msg.lifetime = rospy.Time(10)
                msg.id = i
                array.markers.append(msg)
            if self.done_time:
                msg = Marker()
                msg.header.stamp = rospy.get_rostime()
                msg.header.frame_id = '/map'
                msg.type = Marker.TEXT_VIEW_FACING
                msg.scale.z = 0.14
                msg.color.r = 1.0
                msg.color.g = 1.0
                msg.color.b = 1.0
                msg.color.a = 1.0
                msg.text = 'Done, time = %5.2f'%(self.done_time)
                msg.lifetime = rospy.Time(10)
                msg.id = size*size*size
                array.markers.append(msg)
            pub_goal.publish(array)
            r.sleep()
            if self.start_time:
                rospy.loginfo("%2d/%d %5.2f"%(len(filter(lambda x: x != False, self.status)),len(self.status),float((rospy.get_rostime()-self.start_time).to_sec())))
            if all(self.status):
                if self.done_time:
                    rospy.loginfo("Done.. %5.2f"%self.done_time)
                else:
                    self.done_time = (rospy.get_rostime()-self.start_time).to_sec()
                    self.recordResult(self.ui_type, self.done_time)
                for s in self.status:
                    rospy.loginfo("%5.2f"%s.to_sec())

if __name__ == '__main__':
    try:
        i = InteractiveMarkerTest()
    except rospy.ROSInterruptException:
        pass

