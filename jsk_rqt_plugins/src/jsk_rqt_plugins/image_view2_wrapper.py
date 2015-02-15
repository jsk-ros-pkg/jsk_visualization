from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget, \
     QPainter, QColor, QFont, QBrush, QPen, QMessageBox, QSizePolicy, \
     QImage, QPixmap, qRgb
     
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, \
     QEvent, QSize
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys

import numpy as np
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
from image_view2.msg import MouseEvent
from sensor_msgs.msg import Image

class ImageView2Plugin(Plugin):
    """
    rqt wrapper for image_view2
    """
    def __init__(self, context):
        super(ImageView2Plugin, self).__init__(context)
        self.setObjectName("ImageView2Plugin")
        self._widget = ImageView2Widget()
        context.add_widget(self._widget)

class ScaledLabel(QtGui.QLabel):
    def __init__(self, *args, **kwargs):
        QtGui.QLabel.__init__(self)
        self._pixmap = QtGui.QPixmap(self.pixmap())
    def resizeEvent(self, event):
        self.setPixmap(self._pixmap.scaled(
            self.width(), self.height(),
            QtCore.Qt.KeepAspectRatio))

class ImageView2Widget(QWidget):
    """
    Qt widget to communicate with image_view2
    """
    cv_image = None
    pixmap = None
    def __init__(self):
        super(ImageView2Widget, self).__init__()
        self.left_button_clicked = False
        self.lock = Lock()
        self.event_pub = rospy.Publisher("event", MouseEvent)
        self.bridge = CvBridge()
        self.label = ScaledLabel()
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored))
        #self.label.installEventFilter(self)
        hbox = QtGui.QHBoxLayout(self)
        hbox.addWidget(self.label)
        self.setLayout(hbox)
        self.image_sub = rospy.Subscriber("image_marked", Image, 
                                          self.imageCallback)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(40)
        self.setMouseTracking(True)
        self.show()
    def imageCallback(self, msg):
        with self.lock:
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if msg.encoding == "bgr8":
                self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "rgb8":
                self.cv_image = cv_image
            self.numpy_image = np.asarray(self.cv_image)
    def redraw(self):
        with self.lock:
            if self.cv_image != None:
                size = self.cv_image.shape
                img = QImage(self.cv_image.data,
                             size[1], size[0], size[2] * size[1],
                             QImage.Format_RGB888)
                # convert to QPixmap
                self.pixmap = QPixmap(size[1], size[0])
                self.pixmap.convertFromImage(img)
                self.label.setPixmap(self.pixmap.scaled(
                    self.label.width(), self.label.height(),
                    QtCore.Qt.KeepAspectRatio))
                #self.label.setPixmap(self.pixmap)
    def mousePosition(self, e):
        label_x = self.label.x()
        label_y = self.label.y()
        label_width = self.label.width()
        label_height = self.label.height()
        pixmap_width = self.label.pixmap().width()
        pixmap_height = self.label.pixmap().height()
        x_offset = (label_width - pixmap_width) / 2.0 + label_x
        y_offset = (label_height - pixmap_height) / 2.0 + label_y
        return (e.x() - x_offset, e.y()- y_offset)
    def mouseMoveEvent(self, e):
        if self.left_button_clicked:
            msg = MouseEvent()
            msg.header.stamp = rospy.Time.now()
            msg.type = MouseEvent.MOUSE_MOVE
            msg.x, msg.y = self.mousePosition(e)
            msg.width = self.label.pixmap().width()
            msg.height = self.label.pixmap().height()
            self.event_pub.publish(msg)
    def mousePressEvent(self, e):
        msg = MouseEvent()
        msg.header.stamp = rospy.Time.now()
        if e.button() == Qt.LeftButton:
            msg.type = MouseEvent.MOUSE_LEFT_DOWN
            self.left_button_clicked = True
        elif msg.type == Qt.RightButton:
            msg.type = MouseEvent.MOUSE_RIGHT_DOWN
        msg.width = self.label.pixmap().width()
        msg.height = self.label.pixmap().height()
        msg.x, msg.y = self.mousePosition(e)
        self.event_pub.publish(msg)
    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.left_button_clicked = False
            msg = MouseEvent()
            msg.header.stamp = rospy.Time.now()
            msg.width = self.label.pixmap().width()
            msg.height = self.label.pixmap().height()
            msg.type = MouseEvent.MOUSE_LEFT_UP
            msg.x, msg.y = self.mousePosition(e)
            self.event_pub.publish(msg)
    def eventFilter(self, widget, event):
        if not self.pixmap:
            return QtGui.QMainWindow.eventFilter(self, widget, event)
        if (event.type() == QtCore.QEvent.Resize and
            widget is self.label):
            self.label.setPixmap(self.pixmap.scaled(
                self.label.width(), self.label.height(),
                QtCore.Qt.KeepAspectRatio))
            return True
        return QtGui.QMainWindow.eventFilter(self, widget, event)
