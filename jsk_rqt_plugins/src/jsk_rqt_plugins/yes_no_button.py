#!/usr/bin/env python

from distutils.version import LooseVersion
import os

from jsk_rqt_plugins.compat import ros_compat, ROS_VERSION
if ROS_VERSION == 1:
    import rospkg
    import rospy
    import rosservice
else:
    from ament_index_python.packages import get_package_share_directory

import python_qt_binding
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin

from jsk_gui_msgs.srv import YesNo

# For both qt4 and qt5
if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QWidget
else:
    from python_qt_binding.QtGui import QWidget


class YesNoButtonWidget(QWidget):
    def __init__(self, node=None):
        super(YesNoButtonWidget, self).__init__()
        self._node = node
        if ROS_VERSION == 2:
            package_path = get_package_share_directory('jsk_rqt_plugins')
        else:
            rp = rospkg.RosPack()
            package_path = rp.get_path('jsk_rqt_plugins')
        ui_file = os.path.join(
            package_path, 'resource', 'yes_no_button.ui')
        loadUi(ui_file, self)
        self.setObjectName('YesNoButtonUi')
        self.yes_button.clicked[bool].connect(self._clicked_yes_button)
        self.no_button.clicked[bool].connect(self._clicked_no_button)
        self.yes_button.setEnabled(False)
        self.no_button.setEnabled(False)
        self.yes = None
        if ROS_VERSION == 2:
            service_name = node.get_namespace().rstrip('/') + '/rqt_yn_btn'
            service_list = [
                name for name, _ in node.get_service_names_and_types()]
        else:
            service_name = rospy.get_namespace() + 'rqt_yn_btn'
            service_list = rosservice.get_service_list()
        if service_name in service_list:
            ros_compat.logwarn(
                node, '{} is already advertised'.format(service_name))
            return
        self.srv = ros_compat.create_service(
            node, 'rqt_yn_btn', YesNo, self._handle_yn_btn)

    def _clicked_yes_button(self):
        """Handle events of being clicked yes button."""
        self.yes = True

    def _clicked_no_button(self):
        """Handle events of being clicked no button."""
        self.yes = False

    def _handle_yn_btn(self, req, res):
        """Callback function of service,

        and handle enable/disable of the buttons.
        """
        self.message.setText(req.message)
        self.yes = None  # initialize
        self.yes_button.setEnabled(True)
        self.no_button.setEnabled(True)
        while self.yes is None:  # wait for user input
            ros_compat.sleep(self._node, 1.)
        self.yes_button.setEnabled(False)
        self.no_button.setEnabled(False)
        res.yes = self.yes
        return res

    def __del__(self):
        if ROS_VERSION == 2:
            if hasattr(self, 'srv'):
                self._node.destroy_service(self.srv)
        else:
            self.srv.shutdown()


class YesNoButton(Plugin):
    def __init__(self, context):
        super(YesNoButton, self).__init__(context)
        self.setObjectName('YesNoButton')
        self._widget = YesNoButtonWidget(getattr(context, 'node', None))
        context.add_widget(self._widget)
