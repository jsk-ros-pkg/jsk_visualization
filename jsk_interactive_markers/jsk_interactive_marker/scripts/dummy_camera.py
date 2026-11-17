#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
from sensor_msgs.msg import Image, CameraInfo


def set_matrices(camera_info, D, K, R, P):
    # D, K, R, P of ROS 1 are d, k, r, p on ROS 2
    if ROS_VERSION == 1:
        camera_info.D, camera_info.K, camera_info.R, camera_info.P = D, K, R, P
    else:
        camera_info.d, camera_info.k, camera_info.r, camera_info.p = D, K, R, P


if __name__ == '__main__':
    node = ros_compat.init_node("dummy_camera")

    pub = ros_compat.create_publisher(node, "~image", Image, queue_size=1)
    pub_info = ros_compat.create_publisher(
        node, "~camera_info", CameraInfo, queue_size=1)
    frame_id = ros_compat.get_param(node, '~frame_id', 'dummy_camera')
    width = ros_compat.get_param(node, '~width', 640)
    height = ros_compat.get_param(node, '~height', 480)

    def publish(event):
        # the stamps are zero, as rospy.Time(0) of the original
        dummy_camera_info = CameraInfo()
        dummy_camera_info.width = width
        dummy_camera_info.height = height
        dummy_camera_info.header.frame_id = frame_id
        set_matrices(
            dummy_camera_info,
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0,
             0.0, 0.0, 1.0, 0.0])
        dummy_camera_info.distortion_model = "plumb_bob"
        pub_info.publish(dummy_camera_info)

        dummy_img = Image()
        dummy_img.width = width
        dummy_img.height = height
        dummy_img.header.frame_id = frame_id
        dummy_img.step = width
        dummy_img.encoding = "mono8"
        dummy_img.data = [0] * width * height
        pub.publish(dummy_img)

    ros_compat.create_timer(node, 0.1, publish)
    ros_compat.spin(node)
