#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from jsk_rqt_plugins.button_general import ServiceButtonGeneralWidget
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QTabWidget
from python_qt_binding.QtWidgets import QWidget
from resource_retriever import get_filename
import yaml

# Keys read out of the tabbed_layout parameter of each tab
_TAB_KEYS = ('type', 'name', 'yaml_file', 'namespace')


class ServiceTabbedButtonGeneralWidget(QWidget):

    def __init__(self, node):
        super(ServiceTabbedButtonGeneralWidget, self).__init__()
        self._node = node
        self._tab_settings = None
        self._tab_list = []

        # ROS 2 parameters cannot hold a nested structure, so the tabs are
        # described as a list of tab names in `tabbed_layout.tab_list` plus one
        # `tabbed_layout.<tab>.<key>` parameter per key.
        if not self._node.has_parameter('tabbed_layout.tab_list'):
            self._node.declare_parameter('tabbed_layout.tab_list', [''])
        tab_list = [
            t for t in
            self._node.get_parameter('tabbed_layout.tab_list').value or []
            if t]
        if not tab_list:
            self.showError('Cannot find parameter tabbed_layout.tab_list')
            return

        for tb in tab_list:
            base = 'tabbed_layout.{}.'.format(tb)
            for key in _TAB_KEYS:
                if not self._node.has_parameter(base + key):
                    self._node.declare_parameter(base + key, '')
            settings = {
                key: self._node.get_parameter(base + key).value
                for key in _TAB_KEYS
                if self._node.get_parameter(base + key).value
            }
            settings.setdefault('name', tb)
            if 'yaml_file' not in settings:
                self.showError('Cannot find yaml_file in %s' % tb)
                continue
            self._tab_list.append(settings)

        if len(self._tab_list) == 0:
            self.showError('there is no valid param in tabbed_layout')
            return

        qtab = QTabWidget()
        for tb in self._tab_list:
            wg = ServiceButtonGeneralWidgetInTab(self._node, tb)
            qtab.addTab(wg, tb['name'])

        hbox = QHBoxLayout()
        hbox.addWidget(qtab)
        self.setLayout(hbox)
        self.show()

    def showError(self, message):
        QMessageBox.about(self, 'ERROR', message)

    def save_settings(self, plugin_settings, instance_settings):
        # ignore settings
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # ignore settings
        pass

    def trigger_configuration(self):
        # ignore settings
        pass


class ServiceButtonGeneralWidgetInTab(ServiceButtonGeneralWidget):
    """Qt widget to visualize multiple buttons inside a tab."""

    def __init__(self, node, settings):
        super(ServiceButtonGeneralWidgetInTab, self).__init__(node)
        yaml_file = settings['yaml_file']
        self.button_type = settings.get('type', 'push')
        namespace = settings.get('namespace')

        self._layout_param = None
        self._dialog = QFileDialog()
        self._dialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        self._dialog.setNameFilter(
            self._translator.tr('YAML files (*.yaml *.yml)'))

        resolved_yaml = get_filename(yaml_file)
        if resolved_yaml.startswith('file://'):
            resolved_yaml = resolved_yaml[len('file://'):]

        with open(resolved_yaml) as f:
            yaml_data = yaml.safe_load(f)
            self.setupButtons_with_yaml_data(
                yaml_data=yaml_data, namespace=namespace)

        self.show()
