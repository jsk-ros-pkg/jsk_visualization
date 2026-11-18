#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# The ROS 2 version of test/test_util.py. The messages that test uses
# (jsk_rviz_plugins/OverlayText, jsk_recognition_msgs/Histogram) are not
# released for ROS 2, so messages of ros2/common_interfaces with the same
# kinds of fields are used.

from jsk_rqt_plugins.util import get_slot_type_field_names
from rosidl_runtime_py.utilities import get_message


def test_get_slot_type_field_names():
    # test for type as slot_type
    msg = get_message('visualization_msgs/msg/Marker')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/header/frame_id', '/ns', '/texture_resource',
                           '/texture/header/frame_id', '/texture/format',
                           '/text', '/mesh_resource', '/mesh_file/filename']
    # test for msg as slot_type, also in an array
    field_names = get_slot_type_field_names(
        msg, slot_type='std_msgs/ColorRGBA')
    assert field_names == ['/color', '/colors[]']
    # test for type array
    msg = get_message('std_msgs/msg/Float64MultiArray')
    field_names = get_slot_type_field_names(msg, slot_type='double')
    assert field_names == ['/data[]']
    # test for msg array
    msg = get_message('diagnostic_msgs/msg/DiagnosticArray')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/header/frame_id', '/status[]/name',
                           '/status[]/message', '/status[]/hardware_id',
                           '/status[]/values[]/key',
                           '/status[]/values[]/value']
