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
    from python_qt_binding.QtWidgets import QFileDialog
    from python_qt_binding.QtWidgets import QGroupBox
    from python_qt_binding.QtWidgets import QHBoxLayout
    from python_qt_binding.QtWidgets import QMenu
    from python_qt_binding.QtWidgets import QMessageBox
    from python_qt_binding.QtWidgets import QRadioButton
    from python_qt_binding.QtWidgets import QSizePolicy
    from python_qt_binding.QtWidgets import QToolButton
    from python_qt_binding.QtWidgets import QVBoxLayout
    from python_qt_binding.QtWidgets import QWidget
    from python_qt_binding.QtWidgets import QTabWidget
else:
    from python_qt_binding.QtGui import QAction
    from python_qt_binding.QtGui import QComboBox
    from python_qt_binding.QtGui import QCompleter
    from python_qt_binding.QtGui import QDialog
    from python_qt_binding.QtGui import QFileDialog
    from python_qt_binding.QtGui import QGroupBox
    from python_qt_binding.QtGui import QHBoxLayout
    from python_qt_binding.QtGui import QMenu
    from python_qt_binding.QtGui import QMessageBox
    from python_qt_binding.QtGui import QRadioButton
    from python_qt_binding.QtGui import QSizePolicy
    from python_qt_binding.QtGui import QToolButton
    from python_qt_binding.QtGui import QVBoxLayout
    from python_qt_binding.QtGui import QWidget
    from python_qt_binding.QtGui import QTabWidget

from jsk_rqt_plugins.button_general import ServiceButtonGeneralWidget


class ServiceTabbedButtonGeneralWidget(QWidget):
    def __init__(self):
        super(ServiceTabbedButtonGeneralWidget, self).__init__()
        self._tab_settings = None

        if rospy.has_param("~tabbed_layout"):
            tabbed_layout = rospy.get_param("~tabbed_layout")
            self._tab_list = []
            if not 'tab_list' in tabbed_layout:
                self.showError("Cannot find tab_list in %s"%(tabbed_layout))
                return
            tab_list = tabbed_layout['tab_list']
            for tb in tab_list:
                if tb in tabbed_layout:
                    param_settings = tabbed_layout[tb]
                    settings = {}
                    ##
                    if 'type' in param_settings:
                        settings['type'] = param_settings['type']
                    ##
                    if not 'name' in param_settings:
                        settings['name'] = tb
                    else:
                        settings['name'] = param_settings['name']
                    ##
                    if 'yaml_file' in param_settings:
                        settings['yaml_file'] =  param_settings['yaml_file']
                    else:
                        self.showError("Cannot find yaml_file in %s"%(tb))
                        settings = None
                    ##
                    if 'namespace' in param_settings:
                        settings['namespace'] =  param_settings['namespace']

                    if settings:
                        self._tab_list.append(settings)
                else:
                    self.showError("Cannot find key %s in %s"%(tb, tabbed_layout))
        else:
            self.showError("Cannot find rosparam ~tabbed_layout")
            return

        if len(self._tab_list) == 0:
            self.showError("there is no valid param in ~tabbed_layout")
            return

        qtab = QTabWidget()
        for tb in self._tab_list:
            wg = ServiceButtonGeneralWidget_in_tab(tb)
            qtab.addTab(wg, tb['name'] )

        #self.setWindowTitle('Tab Layout')
        hbox = QHBoxLayout()
        hbox.addWidget(qtab)
        self.setLayout(hbox)
        self.show()

    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)

    def save_settings(self, plugin_settings, instance_settings):
        ## ignore settings
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        ## ignore settings
        pass
    def trigger_configuration(self):
        ## ignore settings
        pass

class ServiceButtonGeneralWidget_in_tab(ServiceButtonGeneralWidget):
    """
    Qt widget to visualize multiple buttons
    """
    def __init__(self, settings):
        super(ServiceButtonGeneralWidget_in_tab, self).__init__()
        yaml_file = settings['yaml_file']
        namespace = None
        if 'type' in settings:
            self.button_type = settings['type']
        else:
            self.button_type = 'push'

        if 'namespace' in settings:
            namespace = settings['namespace']

        self._layout_param = None
        self._dialog = QFileDialog()
        self._dialog.setFileMode(QFileDialog.ExistingFile)
        self._dialog.setNameFilter(
            self._translator.tr("YAML files (*.yaml *.yml)"))

        resolved_yaml = get_filename(yaml_file)
        if "file://" == resolved_yaml[0:7]:
            resolved_yaml = resolved_yaml[len("file://"):]

        with open(resolved_yaml) as f:
            yaml_data = yaml.safe_load(f)
            self.setupButtons_with_yaml_data(yaml_data=yaml_data, namespace=namespace)

        self.show()
