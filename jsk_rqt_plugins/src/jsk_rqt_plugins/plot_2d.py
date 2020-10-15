#!/usr/bin/env python

import argparse
import os
import sys

from cStringIO import StringIO
import cv2
from cv_bridge import CvBridge
from distutils.version import LooseVersion
from matplotlib.figure import Figure
import numpy as np
import python_qt_binding
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QIcon
import rospkg
import rospy
from rqt_gui_py.plugin import Plugin
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException
from rqt_py_common.topic_completer import TopicCompleter
from sensor_msgs.msg import Image
from sklearn import linear_model

from jsk_recognition_msgs.msg import PlotData
from jsk_recognition_msgs.msg import PlotDataArray

# qt5 in kinetic
if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QSizePolicy
    from python_qt_binding.QtWidgets import QVBoxLayout
    from python_qt_binding.QtWidgets import QWidget
    try:
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
            as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
            as FigureCanvas
    try:
        from matplotlib.backends.backend_qt5agg import NavigationToolbar2QTAgg \
            as NavigationToolbar
    except ImportError:
        from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT \
            as NavigationToolbar
else:
    from python_qt_binding.QtGui import QSizePolicy
    from python_qt_binding.QtGui import QVBoxLayout
    from python_qt_binding.QtGui import QWidget
    try:
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg \
            as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg \
            as FigureCanvas
    try:
        from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg \
            as NavigationToolbar
    except ImportError:
        from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT \
            as NavigationToolbar



class ROSData(_ROSData):
    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return val
            for f in self.field_evals:
                val = f(val)
            return val
        except IndexError:
            self.error = RosPlotException(
                "{0} index error for: {1}".format(
                    self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException(
                "{0} value was not numeric: {1}".format(
                    self.name, val))


def add_list(xss):

    "xss = [[0, 1, 2, ...], [0, 1, 2, ...], ...]"

    ret = []
    for xs in xss:
        ret.extend(xs)
    return ret


class Plot2D(Plugin):
    def __init__(self, context):
        super(Plot2D, self).__init__(context)
        self.setObjectName('Plot2D')
        self._args = self._parse_args(context.argv())
        self._widget = Plot2DWidget(self._args.topics)
        self._widget.is_line = self._args.line
        self._widget.fit_line = self._args.fit_line
        self._widget.fit_line_ransac = self._args.fit_line_ransac
        outlier = self._args.fit_line_ransac_outlier
        self._widget.fit_line_ransac_outlier = outlier
        self._widget.xtitle = self._args.xtitle
        self._widget.ytitle = self._args.ytitle
        self._widget.no_legend = self._args.no_legend
        self._widget.sort_x = self._args.sort_x
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(
            prog='rqt_histogram_plot', add_help=False)
        Plot2D.add_arguments(parser)
        args = parser.parse_args(argv)
        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group(
            'Options for rqt_histogram plugin')
        group.add_argument(
            'topics', nargs='?', default=[], help='Topics to plot')
        group.add_argument(
            '--line', action="store_true",
            help="Plot with lines instead of scatter")
        group.add_argument(
            '--fit-line', action="store_true",
            help="Plot line with least-square fitting")
        group.add_argument(
            '--fit-line-ransac', action="store_true",
            help="Plot line with RANSAC")
        group.add_argument(
            '--fit-line-ransac-outlier', type=float, default=0.1,
            help="Plot line with RANSAC")
        group.add_argument('--xtitle', help="Title in X axis")
        group.add_argument('--ytitle', help="Title in Y axis")
        group.add_argument('--no-legend', action="store_true")
        group.add_argument('--sort-x', action="store_true")


class Plot2DWidget(QWidget):
    _redraw_interval = 10

    def __init__(self, topics):
        super(Plot2DWidget, self).__init__()
        self.setObjectName('Plot2DWidget')
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('jsk_rqt_plugins'), 'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.cv_bridge = CvBridge()
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatPlot2D(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent
        self._start_time = rospy.get_time()
        self._rosdata = None
        if len(topics) != 0:
            self.subscribe_topic(topics)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(self._redraw_interval)

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.subscribe_topic(topic_name)

    @Slot()
    def on_topic_edit_returnPressed(self):
        "callback function when form is entered"
        if self.subscribe_topic_button.isEnabled():
            self.subscribe_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.subscribe_topic(str(self.topic_edit.text()))

    def subscribe_topic(self, topic_name):
        rospy.loginfo("subscribe topic")
        self.topic_with_field_name = topic_name
        self.pub_image = rospy.Publisher(
            topic_name + "/plot_image", Image, queue_size=1)
        if not self._rosdata:
            self._rosdata = ROSData(topic_name, self._start_time)
        else:
            if self._rosdata != topic_name:
                self._rosdata.close()
                self.data_plot.clear()
                self._rosdata = ROSData(topic_name, self._start_time)
            else:
                rospy.logwarn("%s is already subscribed", topic_name)

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()

    @Slot()
    def on_clear_button_clicked(self):
        self.data_plot.clear()

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    def plot_one(self, msg, axes):
        concatenated_data = zip(msg.xs, msg.ys)
        if self.sort_x:
            concatenated_data.sort(key=lambda x: x[0])
        xs = [d[0] for d in concatenated_data]
        ys = [d[1] for d in concatenated_data]
        if self.is_line or msg.type is PlotData.LINE:
            axes.plot(xs, ys,
                      label=msg.label or self.topic_with_field_name)
        else:
            axes.scatter(xs, ys,
                         label=msg.label or self.topic_with_field_name)
        if msg.fit_line or self.fit_line:
            X = np.array(msg.xs)
            Y = np.array(msg.ys)
            A = np.array([X, np.ones(len(X))])
            A = A.T
            a, b = np.linalg.lstsq(A, Y, rcond=-1)[0]
            axes.plot(X, (a*X+b), "g--", label="{0} x + {1}".format(a, b))
        if msg.fit_line_ransac or self.fit_line_ransac:
            model_ransac = linear_model.RANSACRegressor(
                linear_model.LinearRegression(), min_samples=2,
                residual_threshold=self.fit_line_ransac_outlier)
            X = np.array(msg.xs).reshape((len(msg.xs), 1))
            Y = np.array(msg.ys)
            model_ransac.fit(X, Y)
            line_X = X
            line_y_ransac = model_ransac.predict(line_X)
            if len(model_ransac.estimator_.coef_) == 1:
                coef = model_ransac.estimator_.coef_[0]
            else:
                coef = model_ransac.estimator_.coef_[0][0]
            if not isinstance(model_ransac.estimator_.intercept_, list):
                intercept = model_ransac.estimator_.intercept_
            else:
                intercept = model_ransac.estimator_.intercept_[0]
            axes.plot(
                line_X, line_y_ransac, "r--",
                label="{0} x + {1}".format(coef, intercept))

    def update_plot(self):
        if not self._rosdata:
            return
        try:
            data_x, data_y = self._rosdata.next()
        except RosPlotException as e:
            rospy.logerr("Exception in subscribing topic")
            rospy.logerr(e.message)
            return
        if len(data_y) == 0:
            return
        axes = self.data_plot._canvas.axes
        axes.cla()
        # matplotlib
        # concatenate x and y in order to sort
        latest_msg = data_y[-1]
        min_x = None
        max_x = None
        min_y = None
        max_y = None
        if isinstance(latest_msg, PlotData):
            data = [latest_msg]
            legend_size = 8
            no_legend = False
        elif isinstance(latest_msg, PlotDataArray):
            data = latest_msg.data
            legend_size = latest_msg.legend_font_size or 8
            no_legend = latest_msg.no_legend
            if latest_msg.min_x != latest_msg.max_x:
                min_x = latest_msg.min_x
                max_x = latest_msg.max_x
            if latest_msg.min_y != latest_msg.max_y:
                min_y = latest_msg.min_y
                max_y = latest_msg.max_y
        else:
            rospy.logerr(
                "Topic should be jsk_recognition_msgs/PlotData",
                "or jsk_recognition_msgs/PlotDataArray")
        for d in data:
            self.plot_one(d, axes)
        xs = add_list([d.xs for d in data])
        ys = add_list([d.ys for d in data])
        if min_x is None:
            min_x = min(xs)
        if max_x is None:
            max_x = max(xs)
        if min_y is None:
            min_y = min(ys)
        if max_y is None:
            max_y = max(ys)
        axes.set_xlim(min_x, max_x)
        axes.set_ylim(min_y, max_y)
        # line fitting
        if not no_legend and not self.no_legend:
            axes.legend(prop={'size': legend_size})
        axes.grid()
        if self.xtitle:
            axes.set_xlabel(self.xtitle)
        if self.ytitle:
            axes.set_ylabel(self.ytitle)
        self.data_plot._canvas.draw()
        buffer = StringIO()
        self.data_plot._canvas.figure.savefig(buffer, format="png")
        buffer.seek(0)
        img_array = np.asarray(bytearray(buffer.read()), dtype=np.uint8)
        if LooseVersion(cv2.__version__).version[0] < 2:
            iscolor = cv2.CV_LOAD_IMAGE_COLOR
        else:
            iscolor = cv2.IMREAD_COLOR
        img = cv2.imdecode(img_array, iscolor)
        self.pub_image.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


class MatPlot2D(QWidget):

    class Canvas(FigureCanvas):
        def __init__(self, parent=None):
            super(MatPlot2D.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def resizeEvent(self, event):
            super(MatPlot2D.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()

    def __init__(self, parent=None):
        super(MatPlot2D, self).__init__(parent)
        self._canvas = MatPlot2D.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

    def redraw(self):
        pass

    def clear(self):
        self._canvas.axes.cla()
        self._canvas.draw()
