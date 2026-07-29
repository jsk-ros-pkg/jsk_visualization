#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT \
    as NavigationToolbar
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
# mpl_toolkits.mplot3d has to be imported for the '3d' projection to register
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import qWarning
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction
from python_qt_binding.QtWidgets import QMenu
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from rqt_plot.plot_widget import is_plottable
from rqt_plot.rosplot import ROSData
from rqt_plot.rosplot import RosPlotException
from rqt_py_common.topic_completer import TopicCompleter


class MatDataPlot3D(QWidget):

    class Canvas(FigureCanvas):
        """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

        def __init__(self, parent=None):
            super(MatDataPlot3D.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111, projection='3d')
            self.axes.set_xlabel('t')
            self.axes.set_xlim3d(0, 10)
            self.axes.set_ylabel('Y')
            self.axes.set_ylim3d(-1, 1)
            self.axes.set_zlabel('Z')
            self.axes.set_zlim3d(0, 1)

            self._tight_layout()
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            self.updateGeometry()

        def _tight_layout(self):
            # matplotlib >= 3.6 raises when the 3d axes would be laid out at a
            # non-positive size, which happens while Qt is still sizing the
            # canvas. There is nothing to lay out in that case.
            try:
                self.figure.tight_layout()
            except ValueError:
                pass

        def draw(self):
            # Qt also triggers draws on its own (backend_qt._draw_idle), so the
            # guard has to live here rather than only in MatDataPlot3D.redraw().
            if self.width() <= 0 or self.height() <= 0:
                return
            super(MatDataPlot3D.Canvas, self).draw()

        def resizeEvent(self, event):
            super(MatDataPlot3D.Canvas, self).resizeEvent(event)
            self._tight_layout()

    _colors = [QColor(c) for c in [
        Qt.GlobalColor.red, Qt.GlobalColor.blue, Qt.GlobalColor.magenta,
        Qt.GlobalColor.cyan, Qt.GlobalColor.green, Qt.GlobalColor.darkYellow,
        Qt.GlobalColor.black, Qt.GlobalColor.darkRed, Qt.GlobalColor.gray,
        Qt.GlobalColor.darkCyan]]

    def __init__(self, parent=None, buffer_length=100, use_poly=True,
                 no_legend=False):
        super(MatDataPlot3D, self).__init__(parent)
        self._canvas = MatDataPlot3D.Canvas()
        self._use_poly = use_poly
        self._buffer_length = buffer_length
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)
        self._curves_verts = {}
        self._color_index = 0
        self._curves = {}
        self._no_legend = no_legend
        self._autoscroll = False

    def autoscroll(self, enabled=True):
        self._autoscroll = enabled

    def add_curve(self, curve_id, curve_name, x, y):
        color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1
        line = None
        self._curves[curve_id] = [[], [], line, [None, None],
                                  (color.red() / 255.0,
                                   color.green() / 255.0,
                                   color.blue() / 255.0,
                                   0.6)]
        self.update_values(curve_id, x, y)
        self._update_legend()

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            del self._curves[curve_id]
            del self._curves_verts[curve_id]
            self._update_legend()

    def _update_legend(self):
        if self._no_legend:
            return
        labels = list(self._curves.keys())
        handles = [
            plt.Rectangle((0, 0), 1, 1, fc=self._curves[labels[i]][4])
            for i in range(len(labels))]
        self._canvas.axes.legend(handles, labels, loc='upper left')

    @Slot(str, list, list)
    def update_values(self, curve_id, x, y):
        data_x, data_y, line, range_y, c = self._curves[curve_id]
        data_x.extend(x)
        data_y.extend(y)
        if len(data_x) > self._buffer_length:
            data_x = data_x[-self._buffer_length:]
            data_y = data_y[-self._buffer_length:]
            self._curves[curve_id][0] = data_x
            self._curves[curve_id][1] = data_y
        self._curves_verts[curve_id] = (data_x, data_y)
        if y:
            ymin = min(y)
            if range_y[0]:
                ymin = min(ymin, range_y[0])
            range_y[0] = ymin
            ymax = max(y)
            if range_y[1]:
                ymax = max(ymax, range_y[1])
            range_y[1] = ymax

    def redraw(self):
        if self._canvas.width() <= 0 or self._canvas.height() <= 0:
            # Qt has not sized the canvas yet. matplotlib >= 3.6 raises
            # ValueError("'box_aspect' and 'fig_aspect' must be positive") when
            # laying out 3d axes at that point, and there is nothing to show anyway.
            return
        self._canvas.axes.grid(True, color='gray')
        # Set axis bounds
        ymin = ymax = None
        xmax = 0
        xmin = sys.maxsize
        for curve in self._curves.values():
            data_x, _, _, range_y, c = curve
            if len(data_x) == 0:
                continue
            xmax = max(xmax, data_x[-1])
            xmin = min(xmin, data_x[0])
            if ymin is None:
                ymin = range_y[0]
                ymax = range_y[1]
            else:
                ymin = min(range_y[0], ymin)
                ymax = max(range_y[1], ymax)

        if self._autoscroll and ymin is not None:
            self._canvas.axes.set_xbound(lower=xmin, upper=xmax)
            self._canvas.axes.set_zbound(lower=ymin, upper=ymax)
            self._canvas.axes.set_ybound(lower=0,
                                         upper=len(self._curves.keys()))
        # create poly object
        verts = []
        colors = []
        for curve_id in self._curves_verts.keys():
            (data_x, data_y) = self._curves_verts[curve_id]
            colors.append(self._curves[curve_id][4])
            if self._use_poly:
                verts.append([(xmin, ymin)] + list(zip(data_x, data_y))
                             + [(xmax, ymin)])
            else:
                verts.append(list(zip(data_x, data_y)))
        line_num = len(self._curves.keys())
        if self._use_poly:
            poly = PolyCollection(verts, facecolors=colors, closed=False)
        else:
            poly = LineCollection(verts, colors=colors)
        poly.set_alpha(0.7)
        self._canvas.axes.cla()
        self._canvas.axes.add_collection3d(
            poly, zs=range(line_num), zdir='y')
        self._update_legend()
        self._canvas.draw()


class Plot3D(Plugin):

    def __init__(self, context):
        super(Plot3D, self).__init__(context)
        self.setObjectName('Plot3D')
        self._node = context.node
        self._args = self._parse_args(context.argv())
        self._widget = Plot3DWidget(
            self._node,
            initial_topics=self._args.topics,
            start_paused=self._args.start_paused,
            buffer_length=self._args.buffer,
            use_poly=not self._args.show_line,
            no_legend=self._args.no_legend)
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def _resolve_topic_name(self, script_name, name):
        """
        Resolve a topic name given on the command line.

        rosgraph.names.script_resolve_name has no ROS 2 equivalent, so this
        mirrors what rqt_plot does in ROS 2.
        """
        sep = '/'
        priv_name = '~'

        # empty string resolves to namespace
        if not name:
            return self._node.get_namespace()
        # Check for global name: /foo/name resolves to /foo/name
        if name[0] == sep:
            return name
        # Check for private name: ~name resolves to /caller_id/name
        elif name[0] == priv_name:
            if script_name[-1] == sep:
                return script_name + name[1:]
            return script_name + sep + name[1:]
        return self._node.get_namespace() + name

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_3d_plot', add_help=False)
        Plot3D.add_arguments(parser)
        args = parser.parse_args(argv)
        topic_list = []
        for t in args.topics:
            # c_topics is the list of topics to plot
            c_topics = []
            # compute combined topic list, t == '/foo/bar1,/baz/bar2'
            for sub_t in [x for x in t.split(',') if x]:
                # check for shorthand '/foo/field1:field2:field3'
                if ':' in sub_t:
                    base = sub_t[:sub_t.find(':')]
                    # the first prefix includes a field name,
                    # so save then strip it off
                    c_topics.append(base)
                    if '/' not in base:
                        parser.error(
                            '%s must contain a topic and field name' % sub_t)
                    base = base[:base.rfind('/')]

                    # compute the rest of the field names
                    fields = sub_t.split(':')[1:]
                    c_topics.extend(['%s/%s' % (base, f) for f in fields if f])
                else:
                    c_topics.append(sub_t)
            # #1053: resolve command-line topic names
            c_topics = [self._resolve_topic_name('rqt_3d_plot', n)
                        for n in c_topics]
            topic_list.extend(c_topics)
        args.topics = topic_list

        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plot plugin')
        group.add_argument(
            '-P', '--pause', action='store_true', dest='start_paused',
            help='Start in paused state')
        group.add_argument(
            '-L', '--line', action='store_true', dest='show_line',
            help='Show lines rather than polygon representation')
        group.add_argument(
            '--no-legend', action='store_true', dest='no_legend',
            help='do not show legend')
        group.add_argument(
            '-B', '--buffer', dest='buffer', action='store',
            help='the length of the buffer', default=100, type=int)
        group.add_argument(
            'topics', nargs='*', default=[], help='Topics to plot')


class Plot3DWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, node, initial_topics=None, start_paused=False,
                 buffer_length=100, use_poly=True, no_legend=False):
        super(Plot3DWidget, self).__init__()
        self.setObjectName('Plot3DWidget')
        self._node = node
        self._buffer_length = buffer_length
        self._initial_topics = initial_topics

        ui_file = os.path.join(
            get_package_share_directory('jsk_rqt_plugins'),
            'resource', 'plot3d.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatDataPlot3D(self, self._buffer_length,
                                       use_poly, no_legend)
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        self.subscribe_topic_button.setEnabled(False)
        if start_paused:
            self.pause_button.setChecked(True)

        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics(self._node)
        self.topic_edit.setCompleter(self._topic_completer)

        self._start_time = self._node.get_clock().now().nanoseconds * 1e-9
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        # ROS 2's rqt_plot.rosplot.ROSData needs the topic type at subscribe
        # time, so a topic given on the command line cannot be added before its
        # publisher has been discovered. ROS 1 subscribed lazily; here the
        # initial topics are retried until they resolve.
        self._initial_topic_timer = QTimer(self)
        self._initial_topic_timer.timeout.connect(self._add_initial_topics)
        if self._initial_topics:
            self._add_initial_topics()
            if self._initial_topics:
                self._initial_topic_timer.start(1000)

    def _add_initial_topics(self):
        remaining = [topic_name for topic_name in self._initial_topics
                     if not self.add_topic(topic_name)]
        self._initial_topics = remaining
        if not remaining:
            self._initial_topic_timer.stop()

    def shutdown(self):
        # The Qt timers outlive the rclpy context otherwise
        self._update_plot_timer.stop()
        self._initial_topic_timer.stop()
        self.clean_up_subscribers()

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        # get topic name
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or \
               len(event.source().selectedItems()) == 0:
                qWarning(
                    'Plot.dragEnterEvent(): not hasattr(event.source(), '
                    'selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            topic_name = item.data(0, Qt.ItemDataRole.UserRole)
            if topic_name is None:
                qWarning(
                    'Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        else:
            topic_name = str(event.mimeData().text())

        # check for numeric field type
        plottable, message = is_plottable(self._node, topic_name)
        if plottable:
            event.acceptProposedAction()
        else:
            qWarning('Plot.dragEnterEvent(): rejecting: "%s"' % (message))

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.ItemDataRole.UserRole))
        self.add_topic(topic_name)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics(self._node)

        plottable, message = is_plottable(self._node, topic_name)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscroll_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)

    @Slot()
    def on_clear_button_clicked(self):
        self.clean_up_subscribers()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(
                            topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning(
                        'PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed,
            # enable timer based on subscribed topics
            self.enable_timer(self._rosdata)

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name):
        """Subscribe topic_name, returning whether it could be resolved."""
        if topic_name in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s'
                     % topic_name)
            return True

        self._rosdata[topic_name] = ROSData(
            self._node, topic_name, self._start_time)
        if self._rosdata[topic_name].error is not None:
            qWarning(str(self._rosdata[topic_name].error))
            del self._rosdata[topic_name]
            return False

        data_x, data_y = self._rosdata[topic_name].next()
        self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()
        return True

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
