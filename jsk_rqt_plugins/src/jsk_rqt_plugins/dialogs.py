#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Small Qt dialogs shared by several of the plugins.

ComboBoxDialog used to live in image_view2_wrapper and LineEditDialog in
label, which made status_light pull in image_view2 (not released for
ROS 2) just for a dialog.
"""

from distutils.version import LooseVersion

import python_qt_binding
import python_qt_binding.QtGui as QtGui

if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QComboBox
    from python_qt_binding.QtWidgets import QCompleter
    from python_qt_binding.QtWidgets import QDialog
    from python_qt_binding.QtWidgets import QLineEdit
    from python_qt_binding.QtWidgets import QPushButton
    from python_qt_binding.QtWidgets import QVBoxLayout
else:
    from python_qt_binding.QtGui import QComboBox
    from python_qt_binding.QtGui import QCompleter
    from python_qt_binding.QtGui import QDialog
    from python_qt_binding.QtGui import QLineEdit
    from python_qt_binding.QtGui import QPushButton
    from python_qt_binding.QtGui import QVBoxLayout


class ComboBoxDialog(QDialog):
    def __init__(self, parent=None):
        super(ComboBoxDialog, self).__init__()
        self.number = 0
        vbox = QVBoxLayout(self)
        self.combo_box = QComboBox(self)
        self.combo_box.activated.connect(self.onActivated)
        vbox.addWidget(self.combo_box)
        button = QPushButton()
        button.setText("Done")
        button.clicked.connect(self.buttonCallback)
        vbox.addWidget(button)
        self.setLayout(vbox)

    def buttonCallback(self, event):
        self.close()

    def onActivated(self, number):
        self.number = number


class LineEditDialog(QDialog):
    def __init__(self, parent=None, candidates=None):
        super(LineEditDialog, self).__init__()
        self.value = None
        self.button_pressed = False
        vbox = QVBoxLayout(self)
        # combo box
        model = QtGui.QStandardItemModel(self)
        for elm in candidates or []:
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
        self.button_pressed = True
        self.close()
