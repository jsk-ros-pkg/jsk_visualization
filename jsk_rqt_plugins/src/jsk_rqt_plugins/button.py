from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget, \
QPainter, QColor, QFont, QBrush, QPen, QMessageBox, QSizePolicy
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent, QSize
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys
from std_srvs.srv import Empty

class ServiceButtons(Plugin):
    """
    rqt class to provide multiple buttons
    """
    def __init__(self, context):
        super(ServiceButtons, self).__init__(context)
        self.setObjectName("ServiceButtons")
        self._widget = ServiceButtonWidget()
        context.add_widget(self._widget)

class ServiceButtonWidget(QWidget):
    """
    Qt widget to visualize multiple buttons
    """
    def __init__(self):
        super(ServiceButtonWidget, self).__init__()
        self.lock = Lock()
        # Initialize layout of the buttons from yaml file
        # The yaml file can be specified by rosparam
        layout_yaml_file = rospy.get_param(
            "layout_file", 
            "package://jsk_rqt_plugins/resource/service_button_layout.yaml")
        resolved_layout_yaml_file = get_filename(layout_yaml_file)[len("file://"):]
        # check file exists
        if not os.path.exists(resolved_layout_yaml_file):
            self.showError("Cannot find %s (%s)" % (
                           layout_yaml_file, resolved_layout_yaml_file))
            sys.exit(1)
        self.setupButtons(resolved_layout_yaml_file)
        self.show()
    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)
    def setupButtons(self, yaml_file):
        """
        Parse yaml file and setup Buttons. Format of the yaml file should be:
        - name: 'button name' (required)
          image: 'path to image' (optional)
          service: 'service' (required)
          column: 'column index' (optional, defaults to 0)
        - name: 'button name' (required)
          image: 'path to image' (optional)
          service: 'service' (required)
          column: 'column index' (optional, defaults to 0)
        """
        self.buttons = []
        with open(yaml_file) as f:
            yaml_data = yaml.load(f)
            # first lookup column num
            column_indices = [d['column'] for d in yaml_data]
            max_column_index = max(*column_indices)
            self.layout = QtGui.QHBoxLayout()
            self.layout_boxes = [QtGui.QVBoxLayout()
                                 for i in range(max_column_index + 1)]
            
            for layout in self.layout_boxes:
                self.layout.addLayout(layout)
            for button_data in yaml_data:
                # check if all the field is available
                if not button_data.has_key("name"):
                    self.showError("name field is missed in yaml")
                    raise Exception("name field is missed in yaml")
                if not button_data.has_key("service"):
                    self.showError("service field is missed in yaml")
                    raise Exception("service field is missed in yaml")
                name = button_data['name']
                button = QtGui.QPushButton()
                button.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
                if button_data.has_key("image"):
                    image_file = get_filename(button_data["image"])[len("file://"):]
                    if os.path.exists(image_file):
                        icon = QtGui.QIcon(image_file)
                        button.setIcon(icon)
                        button.setIconSize(QSize(100, 100))
                else:
                    button.setText(name)
                button.clicked.connect(self.buttonCallback(button_data['service']))
                self.layout_boxes[button_data['column']].addWidget(button)
                self.buttons.append(button)
            self.setLayout(self.layout)
    def buttonCallback(self, service_name):
        """
        return function as callback
        """
        return lambda x: self.buttonCallbackImpl(service_name)
    def buttonCallbackImpl(self, service_name):
        srv = rospy.ServiceProxy(service_name, Empty)
        try:
            srv()
        except rospy.ServiceException, e:
            self.showError("Failed to call %s" % service_name)
