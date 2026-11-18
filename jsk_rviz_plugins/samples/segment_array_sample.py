#!/usr/bin/env python

import numpy as np
from scipy.spatial import ConvexHull

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat (ROS 1 only)
    from jsk_rviz_plugins.rospy_compat import ros_compat
from jsk_recognition_msgs.msg import Segment
from jsk_recognition_msgs.msg import SegmentArray
from geometry_msgs.msg import Point
from sample_utils import exit_on_shutdown


def main():
    node = ros_compat.init_node('segment_array_sample')

    pub = ros_compat.create_publisher(node, '~output', SegmentArray, queue_size=1)
    r = ros_compat.Rate(node, ros_compat.get_param(node, '~rate', 1.0))

    segment_array = SegmentArray()
    segment_array.header.frame_id = ros_compat.get_param(node, '~frame_id', 'map')

    N = 30
    while not ros_compat.is_shutdown(node):
        points = np.random.rand(N, 3)
        hull = ConvexHull(points)
        segment_array.header.stamp = ros_compat.stamp_now(node)
        segment_array.segments = []
        for i in hull.simplices:
            # the messages of ROS 2 (rclpy) take keyword arguments only
            a, b, c = [Point(x=p[0], y=p[1], z=p[2]) for p in points[i]]
            segment_array.segments.extend(
                [Segment(start_point=a, end_point=b),
                 Segment(start_point=b, end_point=c),
                 Segment(start_point=a, end_point=c),
                ])
        pub.publish(segment_array)
        r.sleep()
    ros_compat.shutdown(node)


if __name__ == '__main__':
    with exit_on_shutdown():
        main()
