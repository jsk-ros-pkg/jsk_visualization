#!/usr/bin/env python
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QSizePolicy, QColor
from rqt_py_common.topic_completer import TopicCompleter
from matplotlib.colors import colorConverter
from rqt_py_common.topic_helpers import is_slot_numeric
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException
from matplotlib.collections import (PolyCollection, 
                                    PathCollection, LineCollection)
import matplotlib

import rospkg
import rospy

import os, sys
import argparse

try:
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import sys
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

import numpy as np
import matplotlib.pyplot as plt

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
            self.error = RosPlotException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException("[%s] value was not numeric: %s" % (self.name, val))



class HistogramPlot(Plugin):
    def __init__(self, context):
        super(HistogramPlot, self).__init__(context)
        self.setObjectName('HistogramPlot')
        self._args = self._parse_args(context.argv())
        self._widget = HistogramPlotWidget(self._args.topics)
        context.add_widget(self._widget)
    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_histogram_plot', add_help=False)
        HistogramPlot.add_arguments(parser)
        args = parser.parse_args(argv)
        return args
    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_histogram plugin')
        # group.add_argument('-P', '--pause', action='store_true', dest='start_paused',
        #     help='Start in paused state')
        # group.add_argument('-L', '--line', action='store_true', dest='show_line',
        #     help='Show lines rather than polygon representation')
        # group.add_argument('--no-legend', action='store_true', dest='no_legend',
        #     help='do not show legend')
        # group.add_argument('-B', '--buffer', dest='buffer', action="store",
        #     help='the length of the buffer', default=100, type=int)
        group.add_argument('topics', nargs='?', default=[], help='Topics to plot')
        
class HistogramPlotWidget(QWidget):
    _redraw_interval = 40
    def __init__(self, topics):
        super(HistogramPlotWidget, self).__init__()
        self.setObjectName('HistogramPlotWidget')
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('jsk_rqt_plugins'), 
                               'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatHistogramPlot(self)
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
        print "hello"
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.subscribe_topic(topic_name)
    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.subscribe_topic(str(self.topic_edit.text()))
    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.subscribe_topic(str(self.topic_edit.text()))

    def subscribe_topic(self, topic_name):
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
    
    def update_plot(self):
        if not self._rosdata:
            return
        data_x, data_y = self._rosdata.next()
        if len(data_y) == 0:
            return
        xs = data_y[-1].histogram
        axes = self.data_plot._canvas.axes
        axes.cla()
        axes.set_xlim(xmin=0, xmax=len(xs))
        pos = np.arange(len(xs))
        #axes.xticks(range(5))
        for p, x in zip(pos, xs):
            axes.bar(p, x, color='r', align='center')
        self.data_plot._canvas.draw()
        
class MatHistogramPlot(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None):
            super(MatHistogramPlot.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()
        def resizeEvent(self, event):
            super(MatHistogramPlot.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()
    def __init__(self, parent=None):
        super(MatHistogramPlot, self).__init__(parent)
        self._canvas = MatHistogramPlot.Canvas()
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
