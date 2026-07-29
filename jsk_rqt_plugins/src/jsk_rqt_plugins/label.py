#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import Lock

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from rosidl_runtime_py.utilities import get_message
from rqt_gui_py.plugin import Plugin
from rqt_plot.rosplot import RosPlotException
from std_msgs.msg import String

from .dialogs import LineEditDialog
from .util import get_slot_type_field_names
from .util import ROSData


class StringLabel(Plugin):
    """rqt plugin to provide simple label."""

    def __init__(self, context):
        super(StringLabel, self).__init__(context)
        self.setObjectName('StringLabel')
        self._widget = StringLabelWidget(context.node)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()

    def shutdown_plugin(self):
        self._widget.shutdown()


class StringLabelWidget(QWidget):

    def __init__(self, node):
        super(StringLabelWidget, self).__init__()
        self._node = node
        self.lock = Lock()
        self.string = ''
        vbox = QVBoxLayout(self)
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.label.setSizePolicy(
            QSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored))
        font = QFont('Helvetica', 14)
        self.label.setFont(font)
        self.label.setWordWrap(True)
        vbox.addWidget(self.label)
        self.string_sub = None
        self._string_topics = []
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)
        self._active_topic = None
        # to update label visualization
        self._dialog = LineEditDialog()
        self._rosdata = None
        self._start_time = self._node.get_clock().now().nanoseconds * 1e-9
        self._update_label_timer = QTimer(self)
        self._update_label_timer.timeout.connect(self.updateLabel)
        self._update_label_timer.start(40)

    def shutdown(self):
        # The Qt timers outlive the rclpy context otherwise, and querying the
        # graph from a destroyed context raises RCLError.
        self._update_topic_timer.stop()
        self._update_label_timer.stop()

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._dialog.value)

    def updateLabel(self):
        if not self._rosdata:
            return
        try:
            _, data_y = self._rosdata.next()
        except RosPlotException:
            self._rosdata = None
            return
        if len(data_y) == 0:
            return
        latest = data_y[-1]  # get latest data
        # supports std_msgs/String as well as string data nested in rosmsg
        if isinstance(latest, String):
            self.string = latest.data
        else:
            self.string = latest
        try:
            self.label.setText(self.string)
        except TypeError as e:
            self._node.get_logger().warning(str(e))

    def updateTopics(self):
        need_to_update = False
        for topic, topic_types in self._node.get_topic_names_and_types():
            for topic_type in topic_types:
                try:
                    msg = get_message(topic_type)
                except (ImportError, ValueError, AttributeError):
                    continue
                field_names = get_slot_type_field_names(
                    msg, slot_type='string')
                for field in field_names:
                    string_topic = topic + field
                    if string_topic not in self._string_topics:
                        self._string_topics.append(string_topic)
                        need_to_update = True
        if need_to_update:
            self._string_topics = sorted(self._string_topics)
            self._dialog.combo_box.clear()
            for topic in self._string_topics:
                self._dialog.combo_box.addItem(topic)
            if self._active_topic:
                if self._active_topic not in self._string_topics:
                    self._string_topics.append(self._active_topic)
                    self._dialog.combo_box.addItem(self._active_topic)
                self._dialog.combo_box.setCurrentIndex(
                    self._string_topics.index(self._active_topic))

    def setupSubscriber(self, topic):
        if not self._rosdata:
            self._rosdata = ROSData(self._node, topic, self._start_time)
        elif self._rosdata.name != topic:
            self._rosdata.close()
            self._rosdata = ROSData(self._node, topic, self._start_time)
        else:
            self._node.get_logger().warning('%s is already subscribed' % topic)
        self._active_topic = topic

    def onActivated(self, number):
        self.setupSubscriber(self._string_topics[number])

    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value('active_topic', self._active_topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value('active_topic'):
            topic = instance_settings.value('active_topic')
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)
