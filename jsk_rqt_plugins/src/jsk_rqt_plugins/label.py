from distutils.version import LooseVersion
import math
import os
import sys
from threading import Lock

import python_qt_binding
import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtCore import QEvent
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import qWarning
from python_qt_binding.QtCore import Slot
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtGui import QPen
import yaml

from resource_retriever import get_filename
from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
if ROS_VERSION == 1:
    import rospy
from rqt_gui_py.plugin import Plugin
import rqt_plot
from std_msgs.msg import Bool
from std_msgs.msg import String

from .dialogs import LineEditDialog
from .util import get_slot_type_field_names
from .util import next_data
from .util import ROSData

if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QAction
    from python_qt_binding.QtWidgets import QComboBox
    from python_qt_binding.QtWidgets import QCompleter
    from python_qt_binding.QtWidgets import QDialog
    from python_qt_binding.QtWidgets import QLabel
    from python_qt_binding.QtWidgets import QLineEdit
    from python_qt_binding.QtWidgets import QMenu
    from python_qt_binding.QtWidgets import QMessageBox
    from python_qt_binding.QtWidgets import QPushButton
    from python_qt_binding.QtWidgets import QSizePolicy
    from python_qt_binding.QtWidgets import QVBoxLayout
    from python_qt_binding.QtWidgets import QWidget

else:
    from python_qt_binding.QtGui import QAction
    from python_qt_binding.QtGui import QComboBox
    from python_qt_binding.QtGui import QCompleter
    from python_qt_binding.QtGui import QDialog
    from python_qt_binding.QtGui import QLabel
    from python_qt_binding.QtGui import QLineEdit
    from python_qt_binding.QtGui import QMenu
    from python_qt_binding.QtGui import QMessageBox
    from python_qt_binding.QtGui import QPushButton
    from python_qt_binding.QtGui import QSizePolicy
    from python_qt_binding.QtGui import QVBoxLayout
    from python_qt_binding.QtGui import QWidget


class StringLabel(Plugin):
    """
    rqt plugin to provide simple label
    """
    def __init__(self, context):
        super(StringLabel, self).__init__(context)
        self.setObjectName("StringLabel")
        self._widget = StringLabelWidget(getattr(context, 'node', None))
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
    def __init__(self, node=None):
        super(StringLabelWidget, self).__init__()
        self._node = node
        self.lock = Lock()
        vbox = QVBoxLayout(self)
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignLeft)
        self.label.setSizePolicy(
            QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored))
        font = QFont("Helvetica", 14)
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
        # (ROS 2 has no parameter server to list the parameters of)
        self._dialog = LineEditDialog(
            candidates=rospy.get_param_names() if ROS_VERSION == 1 else None)
        self._rosdata = None
        self._start_time = ros_compat.now_sec(self._node)
        self._update_label_timer = QTimer(self)
        self._update_label_timer.timeout.connect(self.updateLabel)
        self._update_label_timer.start(40)

    def shutdown(self):
        # The Qt timers outlive the rclpy context otherwise, and querying
        # the graph from a destroyed context raises RCLError (ROS 2).
        self._update_topic_timer.stop()
        self._update_label_timer.stop()

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._dialog.value)

    def updateLabel(self):
        if not self._rosdata:
            return
        try:
            _, data_y = next_data(self._rosdata)
        except rqt_plot.rosplot.RosPlotException as e:
            self._rosdata = None
            return
        if len(data_y) == 0:
            return
        latest = data_y[-1]  # get latest data
        # supports std_msgs/String as well as string data nested in rosmsg
        if type(latest) == String:
            self.string = latest.data
        else:
            self.string = latest
        try:
            self.label.setText(self.string)
        except TypeError as e:
            ros_compat.logwarn(self._node, str(e))

    def updateTopics(self):
        need_to_update = False
        if ROS_VERSION == 2:
            published_topics = [
                (topic, topic_type) for topic, topic_types
                in self._node.get_topic_names_and_types()
                for topic_type in topic_types]
        else:
            published_topics = rospy.get_published_topics()
        for (topic, topic_type) in published_topics:
            try:
                msg = ros_compat.resolve_message_class(topic_type)
            except (ImportError, ValueError, AttributeError):
                # a type whose package is not installed here (rclpy raises)
                continue
            field_names = get_slot_type_field_names(msg, slot_type='string')
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

    def _ros_data(self, topic):
        # rqt_plot's ROSData subscribes through the node on ROS 2
        if ROS_VERSION == 2:
            return ROSData(self._node, topic, self._start_time)
        return ROSData(topic, self._start_time)

    def setupSubscriber(self, topic):
        if not self._rosdata:
            self._rosdata = self._ros_data(topic)
        else:
            if self._rosdata != topic:
                self._rosdata.close()
                self._rosdata = self._ros_data(topic)
            else:
                ros_compat.logwarn(self._node, "%s is already subscribed" % topic)
        self._active_topic = topic

    def onActivated(self, number):
        self.setupSubscriber(self._string_topics[number])

    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value("active_topic", self._active_topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("active_topic"):
            topic = instance_settings.value("active_topic")
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)
