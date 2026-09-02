#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import Lock

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtGui import QPen
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import UInt8

from .dialogs import ComboBoxDialog


class StatusLight(Plugin):
    """rqt plugin to visualize a status as a colored light."""

    def __init__(self, context):
        super(StatusLight, self).__init__(context)
        self.setObjectName('StatusLight')
        self._widget = StatusLightWidget(context.node)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()

    def shutdown_plugin(self):
        self._widget.shutdown()


class StatusLightWidget(QWidget):
    _UNKNOWN_COLOR = QColor('#dddddd')
    _SUCCESS_COLOR = QColor('#18FFFF')
    _WARN_COLOR = QColor('#FFCA00')
    _ERROR_COLOR = QColor('#F44336')

    def __init__(self, node):
        super(StatusLightWidget, self).__init__()
        self._node = node
        self.lock = Lock()
        self.status_sub = None
        self.status = 0
        self._status_topics = []
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)
        self._active_topic = None
        self._dialog = ComboBoxDialog()
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(int(1000 / 15))

    def shutdown(self):
        # The Qt timers outlive the rclpy context otherwise, and querying the
        # graph from a destroyed context raises RCLError.
        self._update_topic_timer.stop()
        self._update_plot_timer.stop()

    def redraw(self):
        self.update()

    def paintEvent(self, event):
        with self.lock:
            if self.status == 1:
                color = self._SUCCESS_COLOR
            elif self.status == 2:
                color = self._WARN_COLOR
            else:
                color = self._UNKNOWN_COLOR
            rect = event.rect()
            qp = QPainter()
            qp.begin(self)
            radius = min(rect.width(), rect.height()) - 100
            qp.setFont(QFont('Helvetica', 100))
            qp.setPen(QPen(QBrush(color), 50))
            qp.setBrush(color)
            qp.drawEllipse(
                int((rect.width() - radius) / 2),
                int((rect.height() - radius) / 2),
                radius, radius)
            qp.end()
            return

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._status_topics[self._dialog.number])

    def updateTopics(self):
        need_to_update = False
        for topic, topic_types in self._node.get_topic_names_and_types():
            if 'std_msgs/msg/UInt8' in topic_types:
                if topic not in self._status_topics:
                    self._status_topics.append(topic)
                    need_to_update = True
        if need_to_update:
            self._status_topics = sorted(self._status_topics)
            self._dialog.combo_box.clear()
            for topic in self._status_topics:
                self._dialog.combo_box.addItem(topic)
            if self._active_topic:
                if self._active_topic not in self._status_topics:
                    self._status_topics.append(self._active_topic)
                    self._dialog.combo_box.addItem(self._active_topic)
                self._dialog.combo_box.setCurrentIndex(
                    self._status_topics.index(self._active_topic))

    def setupSubscriber(self, topic):
        if self.status_sub:
            self._node.destroy_subscription(self.status_sub)
        self.status_sub = self._node.create_subscription(
            UInt8, topic, self.statusCallback, 1)
        self._active_topic = topic

    def onActivated(self, number):
        self.setupSubscriber(self._status_topics[number])

    def statusCallback(self, msg):
        self.status = msg.data

    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value('active_topic', self._active_topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value('active_topic'):
            topic = instance_settings.value('active_topic')
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)
