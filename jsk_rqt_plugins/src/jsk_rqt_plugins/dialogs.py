#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Small Qt dialogs shared by several of the plugins.

On ROS 1 ``ComboBoxDialog`` lived in ``image_view2_wrapper`` and
``LineEditDialog`` in ``label``, which made every user of them pull in those
modules. They are collected here so that ``status_light`` does not have to
import the (not yet migrated) ``image_view2_wrapper``.
"""

from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QCompleter
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QVBoxLayout


class ComboBoxDialog(QDialog):

    def __init__(self, parent=None):
        super(ComboBoxDialog, self).__init__()
        self.number = 0
        vbox = QVBoxLayout(self)
        self.combo_box = QComboBox(self)
        self.combo_box.activated.connect(self.onActivated)
        vbox.addWidget(self.combo_box)
        button = QPushButton()
        button.setText('Done')
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
        button.setText('Done')
        button.clicked.connect(self.buttonCallback)
        vbox.addWidget(button)
        self.setLayout(vbox)

    def buttonCallback(self, event):
        self.value = self.line_edit.text()
        self.button_pressed = True
        self.close()
