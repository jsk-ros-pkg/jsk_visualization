#!/usr/bin/env python

import os.path as osp
import sys

import yaml

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from jsk_rviz_plugins.srv import RequestMarkerOperate
from jsk_rviz_plugins.msg import TransformableMarkerOperate
from jsk_interactive_marker.msg import MarkerDimensions
from jsk_interactive_marker.msg import PoseStampedWithName
from jsk_interactive_marker.srv import GetTransformableMarkerFocus
from jsk_interactive_marker.srv import GetTransformableMarkerFocusRequest
from jsk_interactive_marker.srv import SetTransformableMarkerPose
from jsk_interactive_marker.srv import SetTransformableMarkerPoseRequest
from jsk_interactive_marker.srv import SetTransformableMarkerColor
from jsk_interactive_marker.srv import SetTransformableMarkerColorRequest
from jsk_interactive_marker.srv import SetMarkerDimensions
from jsk_interactive_marker.srv import SetMarkerDimensionsRequest
from jsk_recognition_utils.color import labelcolormap
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
import rospy


class TransformableMarkersClient(object):

    def __init__(self):
        self.server = rospy.resolve_name('~server')

        self.config_file = rospy.get_param('~config_file')
        if not osp.exists(self.config_file):
            rospy.logfatal("config_file '{}' does not exist"
                           .format(self.config_file))
            sys.exit(1)
        self.config = yaml.load(open(self.config_file))

        self.req_marker = rospy.ServiceProxy(
            osp.join(self.server, 'request_marker_operate'),
            RequestMarkerOperate)
        self.req_color = rospy.ServiceProxy(
            osp.join(self.server, 'set_color'), SetTransformableMarkerColor)
        self.req_pose = rospy.ServiceProxy(
            osp.join(self.server, 'set_pose'), SetTransformableMarkerPose)
        self.req_dim = rospy.ServiceProxy(
            osp.join(self.server, 'set_dimensions'), SetMarkerDimensions)
        self.req_focus = rospy.ServiceProxy(
            osp.join(self.server, 'get_focus'),
            GetTransformableMarkerFocus)
        rospy.wait_for_service(self.req_marker.resolved_name)
        rospy.wait_for_service(self.req_color.resolved_name)
        rospy.wait_for_service(self.req_pose.resolved_name)
        rospy.wait_for_service(self.req_dim.resolved_name)
        rospy.wait_for_service(self.req_focus.resolved_name)

        self.object_poses = {}
        self.object_dimensions = {}

        # TODO: support other than box: ex. torus, cylinder, mesh

        # insert box
        boxes = self.config['boxes']
        n_boxes = len(boxes)
        cmap = labelcolormap(n_boxes)
        for i in xrange(n_boxes):
            box = boxes[i]
            name = box['name']
            frame_id = box.get('frame_id', '/map')
            self.insert_marker(name, frame_id,
                               type=TransformableMarkerOperate.BOX)
            self.set_color(name, (cmap[i][0], cmap[i][1], cmap[i][2], 0.5))
            dim = box.get('dimensions', [1, 1, 1])
            self.set_dimensions(name, dim)
            pos = box.get('position', [0, 0, 0])
            ori = box.get('orientation', [0, 0, 0, 1])
            self.set_pose(box['name'], box['frame_id'], pos, ori)
            self.object_poses[box['name']] = Pose(
                position=Vector3(*pos),
                orientation=Quaternion(*ori),
            )
            self.object_dimensions[box['name']] = Vector3(*dim)
            rospy.loginfo("Inserted transformable box '{}'.".format(name))

        self.sub_dimensions = rospy.Subscriber(
            osp.join(self.server, 'marker_dimensions'),
            MarkerDimensions,
            self._dimensions_change_callback)
        self.sub_pose = rospy.Subscriber(
            osp.join(self.server, 'pose_with_name'),
            PoseStampedWithName,
            self._pose_change_callback)

        do_auto_save = rospy.get_param('~config_auto_save', True)
        if do_auto_save:
            self.timer_save = rospy.Timer(
                rospy.Duration(1), self._save_callback)

        self.pub_bboxes = rospy.Publisher(
            '~output/boxes', BoundingBoxArray, queue_size=1)
        self.timer_pub_bboxes = rospy.Timer(
            rospy.Duration(0.1), self._pub_bboxes_callback)

    def insert_marker(self, name, frame_id, type):
        msg = TransformableMarkerOperate(
            name=name,
            type=type,
            action=TransformableMarkerOperate.INSERT,
            frame_id=frame_id,
            description=name,
        )
        self.req_marker(msg)

    def set_color(self, name, rgba):
        req = SetTransformableMarkerColorRequest()
        req.target_name = name
        req.color.r = rgba[0]
        req.color.g = rgba[1]
        req.color.b = rgba[2]
        req.color.a = rgba[3]
        self.req_color(req)

    def set_dimensions(self, name, dimensions):
        req = SetMarkerDimensionsRequest()
        req.target_name = name
        req.dimensions.x = dimensions[0]
        req.dimensions.y = dimensions[1]
        req.dimensions.z = dimensions[2]
        self.req_dim(req)

    def set_pose(self, name, frame_id, position, orientation):
        req = SetTransformableMarkerPoseRequest()
        req.target_name = name
        req.pose_stamped.header.frame_id = frame_id
        req.pose_stamped.pose.position.x = position[0]
        req.pose_stamped.pose.position.y = position[1]
        req.pose_stamped.pose.position.z = position[2]
        req.pose_stamped.pose.orientation.x = orientation[0]
        req.pose_stamped.pose.orientation.y = orientation[1]
        req.pose_stamped.pose.orientation.z = orientation[2]
        req.pose_stamped.pose.orientation.w = orientation[3]
        self.req_pose(req)

    def get_focus_marker(self):
        req = GetTransformableMarkerFocusRequest()
        res = self.req_focus(req)
        return res.target_name

    def _dimensions_change_callback(self, msg):
        name = self.get_focus_marker()
        self.object_dimensions[name] = msg

    def _pose_change_callback(self, msg):
        self.object_poses[msg.name] = msg.pose.pose

    def _pub_bboxes_callback(self, event):
        bbox_array_msg = BoundingBoxArray()
        bbox_array_msg.header.frame_id = self.config['boxes'][0]['frame_id']
        bbox_array_msg.header.stamp = event.current_real
        for box in self.config['boxes']:
            bbox_msg = BoundingBox()
            pose = self.object_poses[box['name']]
            bbox_msg.header.frame_id = bbox_array_msg.header.frame_id
            bbox_msg.header.stamp = bbox_array_msg.header.stamp
            bbox_msg.pose = pose
            dimensions = self.object_dimensions[box['name']]
            bbox_msg.dimensions.x = dimensions.x
            bbox_msg.dimensions.y = dimensions.y
            bbox_msg.dimensions.z = dimensions.z
            bbox_array_msg.boxes.append(bbox_msg)
        self.pub_bboxes.publish(bbox_array_msg)

    def _save_callback(self, event):
        for i, box in enumerate(self.config['boxes']):
            box_cfg = self.config['boxes'][i]
            box_cfg['name'] = box['name']
            pose = self.object_poses[box['name']]
            dimensions = self.object_dimensions[box['name']]
            # TODO: support transformed bbox dimensions like pose
            box_cfg.update({
                'frame_id': box['frame_id'],
                'dimensions': [
                    dimensions.x,
                    dimensions.y,
                    dimensions.z,
                ],
                'position': [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                'orientation': [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            })
        yaml.dump(self.config, open(self.config_file, 'w'))


if __name__ == '__main__':
    rospy.init_node('transformable_markers_client')
    TransformableMarkersClient()
    rospy.spin()
