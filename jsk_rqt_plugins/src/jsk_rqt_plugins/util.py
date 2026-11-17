#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
if ROS_VERSION == 2:
    from rosidl_runtime_py.utilities import get_message as get_message_class
else:
    from roslib.message import get_message_class
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException


def next_data(rosdata):
    """rosdata.next(): the buffered data of rqt_plot's ROSData, whose
    next() is renamed to next_data() in ROS 2 after jazzy"""
    if hasattr(rosdata, 'next_data'):
        return rosdata.next_data()
    return rosdata.next()


def _fields_and_types(msg):
    """(field name, field type) of msg: rospy's __slots__/_slot_types, or
    rclpy's get_fields_and_field_types() (e.g. 'sequence<string>' for an
    array, 'std_msgs/Header' for a message)"""
    if ROS_VERSION == 2:
        return msg.get_fields_and_field_types().items()
    return zip(msg.__slots__, msg._slot_types)


def _array_element_type(slot_t):
    """'string' for 'string[]' (ROS 1) or 'sequence<string>' (ROS 2),
    None if slot_t is not an array"""
    if slot_t.endswith('[]'):
        return slot_t[:-len('[]')]
    if slot_t.startswith('sequence<') and slot_t.endswith('>'):
        # bounded sequences are 'sequence<string, 3>'
        return slot_t[len('sequence<'):-len('>')].split(',')[0]
    return None


def get_slot_type_field_names(msg, slot_type, field_name=None, found=None):
    if field_name is None:
        field_name = ''
    if found is None:
        found = []
    if msg is None:
        return []

    for slot, slot_t in _fields_and_types(msg):
        deeper_field_name = field_name + '/' + slot
        if slot_t == slot_type:
            found.append(deeper_field_name)
        elif _array_element_type(slot_t) == slot_type:
            # supports array of type field like string[]
            deeper_field_name += '[]'
            found.append(deeper_field_name)
        try:
            if _array_element_type(slot_t) is not None:
                # supports array of ros message like std_msgs/Header[]
                deeper_field_name += '[]'
                slot_t = _array_element_type(slot_t)
            msg_impl = get_message_class(slot_t)
        except (ImportError, ValueError, AttributeError):
            # not a message type, e.g. float64 (rclpy raises)
            continue
        found = get_slot_type_field_names(msg_impl, slot_type,
                                          deeper_field_name, found)
    return found


class ROSData(_ROSData):
    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return val
            for f in self.field_evals:
                val = f(val)
            return val
        except IndexError:
            self.error = RosPlotException(
                "{0} index error for: {1}".format(
                    self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException(
                "{0} value was not numeric: {1}".format(
                    self.name, val))
