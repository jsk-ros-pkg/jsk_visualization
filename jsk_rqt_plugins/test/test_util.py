#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from jsk_rqt_plugins.util import get_slot_type_field_names
from rosidl_runtime_py.utilities import get_message


def test_get_slot_type_field_names():
    # test for type as slot_type
    msg = get_message('jsk_rviz_plugins/msg/OverlayText')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/font', '/text']
    # test for msg as slot_type
    field_names = get_slot_type_field_names(
        msg, slot_type='std_msgs/ColorRGBA')
    assert field_names == ['/bg_color', '/fg_color']
    # test for msg array
    msg = get_message('diagnostic_msgs/msg/DiagnosticArray')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert field_names == ['/header/frame_id', '/status[]/name',
                           '/status[]/message', '/status[]/hardware_id',
                           '/status[]/values[]/key',
                           '/status[]/values[]/value']


def test_get_slot_type_field_names_array_of_primitive():
    # ROS 1 used jsk_recognition_msgs/Histogram (float64[] histogram) here,
    # which is not migrated to ROS 2 yet, so a core message is used instead.
    msg = get_message('std_msgs/msg/Float64MultiArray')
    # asking for the array type itself matches the field as-is, the way the
    # ROS 1 test did with slot_type='float64[]'
    field_names = get_slot_type_field_names(
        msg, slot_type='sequence<double>')
    assert field_names == ['/data']
    # asking for the element type marks the field as an array
    field_names = get_slot_type_field_names(msg, slot_type='double')
    assert field_names == ['/data[]']
