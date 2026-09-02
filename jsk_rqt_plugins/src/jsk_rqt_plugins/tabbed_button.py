#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from jsk_rqt_plugins.tabbed_button_general import \
    ServiceTabbedButtonGeneralWidget
from rqt_gui_py.plugin import Plugin


class ServiceTabbedButtons(Plugin):
    """rqt class to provide multiple buttons in tabs."""

    def __init__(self, context):
        super(ServiceTabbedButtons, self).__init__(context)
        self.setObjectName('ServiceTabbedButtons')
        self._widget = ServiceTabbedButtonWidget(context.node)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()


class ServiceTabbedButtonWidget(ServiceTabbedButtonGeneralWidget):
    """Qt widget to visualize multiple buttons in tabs."""

    def __init__(self, node):
        super(ServiceTabbedButtonWidget, self).__init__(node)
