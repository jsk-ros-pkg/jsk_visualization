#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from jsk_rqt_plugins.button_general import ServiceButtonGeneralWidget
from rqt_gui_py.plugin import Plugin


class ServiceButtons(Plugin):
    """rqt class to provide multiple buttons."""

    def __init__(self, context):
        super(ServiceButtons, self).__init__(context)
        self.setObjectName('ServiceButtons')
        self._widget = ServiceButtonWidget(context.node)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()


class ServiceButtonWidget(ServiceButtonGeneralWidget):
    """Qt widget to visualize multiple buttons."""

    def __init__(self, node):
        super(ServiceButtonWidget, self).__init__(node, button_type='push')
