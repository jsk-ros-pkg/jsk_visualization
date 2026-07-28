#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rosidl_runtime_py.utilities import get_message
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException


def get_slot_type_field_names(msg, slot_type, field_name=None, found=None):
    if field_name is None:
        field_name = ''
    if found is None:
        found = []
    if msg is None:
        return []

    for slot, slot_t in zip(msg.get_fields_and_field_types().keys(),
                            msg.get_fields_and_field_types().values()):
        deeper_field_name = field_name + '/' + slot
        if slot_t == slot_type:
            found.append(deeper_field_name)
        elif slot_t == 'sequence<{}>'.format(slot_type):
            # supports array of type field like sequence<string>
            deeper_field_name += '[]'
            found.append(deeper_field_name)
        try:
            if slot_t.startswith('sequence<') and slot_t.endswith('>'):
                # supports array of ros message like sequence<std_msgs/Header>
                deeper_field_name += '[]'
                slot_t = slot_t[len('sequence<'):-1]
            msg_impl = get_message(slot_t)
        except (ImportError, ValueError, AttributeError, KeyError):
            # not a message type, e.g. a builtin like float64
            continue
        found = get_slot_type_field_names(msg_impl, slot_type,
                                          deeper_field_name, found)
    return found


class ROSData(_ROSData):
    """
    ROSData which returns the raw value instead of a numeric one.

    rqt_plot's ROSData assumes the field is numeric; this variant keeps
    non-numeric values (e.g. strings) so that StringLabel can use it.
    """

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
                '{0} index error for: {1}'.format(
                    self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException(
                '{0} value was not numeric: {1}'.format(self.name, val))
