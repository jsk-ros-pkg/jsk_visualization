from distutils.version import LooseVersion
import math
import os
import sys

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
import rospy
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Bool
from std_msgs.msg import Time
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QAction
    from python_qt_binding.QtWidgets import QComboBox
    from python_qt_binding.QtWidgets import QCompleter
    from python_qt_binding.QtWidgets import QDialog
    from python_qt_binding.QtWidgets import QGroupBox
    from python_qt_binding.QtWidgets import QHBoxLayout
    from python_qt_binding.QtWidgets import QLineEdit
    from python_qt_binding.QtWidgets import QMenu
    from python_qt_binding.QtWidgets import QMessageBox
    from python_qt_binding.QtWidgets import QPushButton
    from python_qt_binding.QtWidgets import QRadioButton
    from python_qt_binding.QtWidgets import QSizePolicy
    from python_qt_binding.QtWidgets import QToolButton
    from python_qt_binding.QtWidgets import QVBoxLayout
    from python_qt_binding.QtWidgets import QWidget

else:
    from python_qt_binding.QtGui import QAction
    from python_qt_binding.QtGui import QComboBox
    from python_qt_binding.QtGui import QCompleter
    from python_qt_binding.QtGui import QDialog
    from python_qt_binding.QtGui import QGroupBox
    from python_qt_binding.QtGui import QHBoxLayout
    from python_qt_binding.QtGui import QLineEdit
    from python_qt_binding.QtGui import QMenu
    from python_qt_binding.QtGui import QMessageBox
    from python_qt_binding.QtGui import QPushButton
    from python_qt_binding.QtGui import QRadioButton
    from python_qt_binding.QtGui import QSizePolicy
    from python_qt_binding.QtGui import QToolButton
    from python_qt_binding.QtGui import QVBoxLayout
    from python_qt_binding.QtGui import QWidget


class LineEditDialog(QDialog):
    def __init__(self, parent=None):
        super(LineEditDialog, self).__init__()
        self.value = None
        vbox = QVBoxLayout(self)
        # combo box
        model = QtGui.QStandardItemModel(self)
        for elm in rospy.get_param_names():
            model.setItem(model.rowCount(), 0, QtGui.QStandardItem(elm))
        self.combo_box = QComboBox(self)
        self.line_edit = QLineEdit()
        self.combo_box.setLineEdit(self.line_edit)
        self.combo_box.setCompleter(QCompleter())
        self.combo_box.setModel(model)
        self.combo_box.completer().setModel(model)
        self.combo_box.lineEdit().setText('')
        vbox.addWidget(self.combo_box)
        # button
        button = QPushButton()
        button.setText("Done")
        button.clicked.connect(self.buttonCallback)
        vbox.addWidget(button)
        self.setLayout(vbox)

    def buttonCallback(self, event):
        self.value = self.line_edit.text()
        self.close()


class ServiceButtonGeneralWidget(QWidget):
    """
    Qt widget to visualize multiple buttons
    """
    def __init__(self, button_type="push"):
        super(ServiceButtonGeneralWidget, self).__init__()
        self.button_type = button_type
        self._layout_param = None
        self._dialog = LineEditDialog()

        if rospy.has_param("~layout_yaml_file"):
            self.loadLayoutYaml(None)

        self.show()

    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)

    def loadLayoutYaml(self, layout_param):
        # Initialize layout of the buttons from yaml file
        # The yaml file can be specified by rosparam
        layout_yaml_file = rospy.get_param("~layout_yaml_file", layout_param)
        resolved_layout_yaml_file = get_filename(
            layout_yaml_file)[len("file://"):]
        # check file exists
        if not os.path.exists(resolved_layout_yaml_file):
            self.showError("Cannot find %s (%s)" % (
                           layout_yaml_file, resolved_layout_yaml_file))
            sys.exit(1)
        self.setupButtons(resolved_layout_yaml_file)
        self.show()

    def setupButtons(self, yaml_file):
        """
        Parse yaml file and setup Buttons.
        """
        with open(yaml_file) as f:
            yaml_data = yaml.safe_load(f)
            self.setupButtons_with_yaml_data(yaml_data)

    def setupButtons_with_yaml_data(self, yaml_data, namespace=None):
        """
        Setup Buttons with yaml_data which is loaded in setupButtons. Format of the yaml file should be:
        - name: 'button name' (required)
          image: 'path to image for icon' (optional)
          image_size: 'width and height of icon' (optional)
          service: 'service' (required)
          column: 'column index' (optional, defaults to 0)
        """
        self.buttons = []
        if True:
            # lookup colum direction
            direction = 'vertical'
            for d in yaml_data:
                if 'direction' in d:
                    if d['direction'] == 'horizontal':
                        direction = 'horizontal'
                    else: # d['direction'] == 'vertical':
                        direction = 'vertical'
                    yaml_data.remove(d)
                    break
            # lookup column num
            column_indices = [d['column'] for d in yaml_data]
            if len(column_indices) > 1:
                max_column_index = max(*column_indices)
            else:
                max_column_index = column_indices[0]
            if direction == 'vertical':
                self.layout = QHBoxLayout()
                self.layout_boxes = [QVBoxLayout()
                                     for i in range(max_column_index + 1)]
            else:  # direction == 'horizontal'
                self.layout = QVBoxLayout()
                self.layout_boxes = [QHBoxLayout()
                                     for i in range(max_column_index + 1)]
            self.button_groups = [QGroupBox()
                                  for i in range(max_column_index + 1)]
            for button_data in yaml_data:
                # check if all the field is available
                if "name" not in button_data:
                    self.showError("name field is missed in yaml")
                    raise Exception("name field is missed in yaml")
                if "service" not in button_data:
                    self.showError("service field is missed in yaml")
                    raise Exception("service field is missed in yaml")
                if self.button_type == "push":
                    button = QToolButton()
                else:  # self.button_type == "radio":
                    button = QRadioButton()
                button.setSizePolicy(
                    QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
                if "image" in button_data:
                    image_file = get_filename(
                        button_data["image"])[len("file://"):]
                    if os.path.exists(image_file):
                        icon = QtGui.QIcon(image_file)
                        button.setIcon(icon)
                        if "image_size" in button_data:
                            button.setIconSize(QSize(
                                button_data["image_size"][0],
                                button_data["image_size"][1]))
                        else:
                            button.setIconSize(QSize(100, 100))
                if "name" in button_data:
                    name = button_data['name']
                    button.setText(name)
                if 'service_type' in button_data:
                    if button_data['service_type'] == 'Trigger':
                        service_type = Trigger
                    elif button_data['service_type'] == 'Empty':
                        service_type = Empty
                    elif button_data['service_type'] == 'SetBool':
                        service_type = SetBool
                    else:
                        raise Exception("Unsupported service type: {}".format(
                            button_data['service_type']))
                else:
                    service_type = Empty
                if service_type == SetBool:
                    button.setCheckable(True)
                if namespace:
                    sname = button_data['service']
                    if sname[0] == '/' or sname[0] == '~':
                        sname = sname
                    else:
                        sname = namespace + '/' + sname
                    button.clicked.connect(
                        self.buttonCallback(sname, service_type, button))
                else:
                    button.clicked.connect(
                        self.buttonCallback(button_data['service'], service_type, button))
                if self.button_type == "push":
                    button.setToolButtonStyle(
                        QtCore.Qt.ToolButtonTextUnderIcon)
                if ((self.button_type == "radio" or service_type == SetBool)
                        and ("default_value" in button_data
                             and button_data['default_value'])):
                    button.setChecked(True)
                self.layout_boxes[button_data['column']].addWidget(button)
                self.buttons.append(button)
            for i in range(len(self.button_groups)):
                self.button_groups[i].setLayout(self.layout_boxes[i])
            for group in self.button_groups:
                self.layout.addWidget(group)
            self.setLayout(self.layout)

    def buttonCallback(self, service_name, service_type, button):
        """
        return function as callback
        """
        return lambda checked: self.buttonCallbackImpl(checked, service_name, service_type, button)

    def buttonCallbackImpl(self, checked, service_name, service_type=Empty, button=None):
        srv = rospy.ServiceProxy(service_name, service_type)
        try:
            if service_type == SetBool:
                res = srv(checked)
            else:
                res = srv()
            if hasattr(res, 'success'):
                success = res.success
                if not success:
                    self.showError(
                        "Succeeded to call {}, but service response is res.success=False"
                        .format(service_name))
                    if service_type == SetBool and not button is None:
                        button.setChecked(not checked)
        except rospy.ServiceException as e:
            self.showError("Failed to call %s" % service_name)
            if service_type == SetBool and not button is None:
                button.setChecked(not checked)

    def save_settings(self, plugin_settings, instance_settings):
        if self._layout_param:
            instance_settings.set_value("layout_param", self._layout_param)
            rospy.loginfo("save setting is called. %s" % self._layout_param)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("layout_param"):
            self._layout_param = instance_settings.value("layout_param")
            self.loadLayoutYaml(self._layout_param)
            rospy.loginfo("restore setting is called. %s" % self._layout_param)

    def trigger_configuration(self):
        self._dialog.exec_()
        self._layout_param = self._dialog.value
        self.loadLayoutYaml(self._layout_param)
        rospy.loginfo(
            "trigger configuration is called. %s" % self._dialog.value)
