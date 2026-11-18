^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rqt_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-11-19)
------------------
* Merge pull request `#908 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/908>`_ from k-okada/readme-gallery
  README: a gallery that links to how to run each sample
* package.xml: one line for the depends which are the same on ROS 1 and ROS 2
  The depends on jsk_recognition_msgs/utils, python3-scipy and
  python3-sklearn were written twice, with $ROS_VERSION == 1 and == 2;
  jsk_rqt_plugins listed rqt_gui, rqt_gui_py and rqt_plot twice. The
  evaluated dependencies of melodic, noetic, ROS One and jazzy are the
  same, except the version_gte of jsk_recognition_msgs, which now applies
  on ROS 2 too (1.2.19 there).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] exec_depend on jsk_recognition_msgs on ROS 1 as well
  HistogramPlot and Plot2D (hist.py, plot_2d.py) use its messages on ROS 1
  too, but it was not a dependency there.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] build_depend on jsk_recognition_msgs on ROS 2
  CMakeLists.txt installs rqt_histogram_plot and rqt_2d_plot on ROS 2 only
  when jsk_recognition_msgs is found at build time, so a clean package
  build needs it as a build dependency, not only as an exec dependency.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* README: alt text of the gallery images
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* depend on jsk_recognition_msgs/utils on ROS 2 without \*_FROM_SOURCE
  They are going to be released for ROS 2 soon, so the release of
  jsk_visualization depends on them as on ROS 1 (the ROS 2 binaries fail
  on the buildfarm until they are released). The ROS 2 CI checks out
  jsk_recognition_utils as well as jsk_recognition_msgs from source, and
  no longer sets JSK_RECOGNITION_MSGS_FROM_SOURCE.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* rqt_image_view2: drop the image topics which are gone from the list
  updateTopics() only added topics, so a topic whose publisher was gone
  was still offered (and subscribed by Done) instead of reaching the
  "No image topic is found" guard. Make the list from the topics
  published now on each refresh.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* jsk_rqt_plugins README: how to run sample_image_view2 on ROS 2
  rqt_image_view2 now lists the image topics when its configuration dialog
  opens, so it works on ROS 2: run the sample of image_view2 (jsk_common) and
  `ros2 run jsk_rqt_plugins rqt_image_view2`, then select /camera/image with
  the gear button. The note that it does not work on ROS 2 is replaced, and
  doc/jsk_rqt_plugins/plugins/rqt_image_view2.md gets the ROS 2 commands too.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* rqt_image_view2: list the image topics when the configuration dialog opens
  The image topics were listed only once, by a thread started with the
  widget. On ROS 2 discovery has not found any topic then, so the dialog
  stayed empty and Done raised an IndexError (rqt's reload re-created the
  plugin late enough, but `ros2 run jsk_rqt_plugins rqt_image_view2` is
  standalone and has no reload button); on ROS 1 the list was stale.
  trigger_configuration() now lists the topics again before the dialog, takes
  the selected row of the combo box (the active topic is preselected), and
  warns instead of raising when no image topic is found. The time.sleep(1) at
  the end of updateTopics() is dropped, so opening the dialog does not wait,
  and the active topic is preselected only when it is in the list.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* add README.md of jsk_rviz_plugins, jsk_rqt_plugins and jsk_interactive_marker
  Each README has a short intro, an index of the samples and one section per
  sample, named after it: the plugins it shows, links to their pages in doc/,
  the ROS 1 and ROS 2 captures of .readme/gallery/ side by side, and how to run
  it on ROS 1 (roslaunch / rosrun) and on ROS 2 (ros2 launch / ros2 run), with
  the topic publish and service call commands of the rqt scripts.
  On ROS 2, rqt_image_view2 cannot select an image yet (its topic list is
  filled once, before ROS 2 discovery finds any topic), which the README says.
  The existing usage of jsk_interactive_marker/README.md is kept below the
  samples.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request `#905 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/905>`_ from k-okada/rqt-plugins-ros1-ros2
  ROS 1 / ROS 2 for the whole repository from one source tree (jsk_ros1_ros2_compat)
* [jsk_rqt_plugins, jsk_rviz_plugins] samples: exit cleanly on SIGINT on ROS 2
  On ROS 2, SIGINT (e.g. from ros2 launch) shuts the rclpy context down from
  rclpy's signal handler and raises KeyboardInterrupt in the main thread, so
  the main loops of the sample publishers ended with a KeyboardInterrupt
  traceback (out of the Rate's sleep), or with "RCLError: Failed to publish:
  publisher's context is invalid" and exit code 1 when a publish() came after
  the context went down (sample_drc_mini_maxwell.py, every time).
  exit_on_shutdown(), a context manager around the main loops, takes both as
  the normal end on ROS 2 (any other exception while the context is still up
  is raised as before) and does nothing on ROS 1, where rospy handles SIGINT
  as before. It is in jsk_rqt_plugins.compat for sample_scripts/, and in
  samples/sample_utils.py for the samples of jsk_rviz_plugins, which import it
  from their own directory (jsk_rviz_plugins has no python module on ROS 2:
  the name is that of its message package); it is installed next to the
  samples on ROS 2, and by the samples/\*.py glob on ROS 1. In the ROS 1
  devel space the samples run through catkin's relays, and the relay of
  sample_utils.py runs it in a dict of its own, so sample_utils.py puts the
  helper into its module object itself.
  The loops of the samples run on ROS 2 use it: sample_2d_plot.py,
  sample_3d_plot.py, sample_drc_mini_maxwell.py, sample_hist_pub.py,
  bounding_box_sample.py, overlay_menu_sample.py, overlay_sample.py,
  pictogram.py, pictogram_all.py, polygon_array_sample.py and
  segment_array_sample.py. piechart_sample.py and the service samples spin
  through ros_compat.spin(), which already handles it.
  Ctrl-C of a terminal reaches the samples twice under ros2 launch (from
  the terminal and from launch), so the second SIGINT is ignored once the
  loop has ended, not to break the shutdown after it.
  overlay_sample.launch.py of ROS 2 no longer respawns the samples: on
  Ctrl-C launch restarted the samples that had just exited and did not end.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] shut the rqt plugins down cleanly on SIGINT on ROS 2
  Two tracebacks were printed when an rqt\_\* script got SIGINT on ROS 2 (e.g.
  from ros2 launch), both of shutdown races:
  - rqt_gui_py initializes rclpy with its signal handlers, so rclpy shut its
  context down at once on SIGINT, from another thread, while the rclpy
  spinner thread of rqt could be about to wait on it again:
  "RCLError: failed to initialize wait set: the given context is not
  valid" in rqt_gui_py/rclpy_spinner.py (half of the rqt_2d_plot windows
  of sample_2dplot.launch.py). The bin/rqt\_\* scripts initialize rclpy
  without its signal handlers before rqt does now (compat.init_rqt_ros(),
  nothing on ROS 1), so SIGINT is left to rqt's own handler, which shuts
  the plugins down, stops the spinner, destroys the node and then shuts
  rclpy down, in this order.
  - rqt's SIGINT handler is a Python handler, which runs between the
  bytecodes of the main thread, e.g. while update_plot() of Plot2D or
  HistogramPlot draws the plot. The whole shutdown, down to the
  destruction of the node, then happens inside update_plot(), which went
  on to publish ~plot_image with the destroyed publisher:
  "InvalidHandle: cannot use Destroyable because destruction was
  requested". update_plot() does not publish once the widget is shut
  down now.
  The other plugins do not use ROS after their timer callbacks may be
  interrupted (Plot3D, StringLabel, StatusLight, DRCEnvironmentViewer only
  query the graph or read buffered data before drawing).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] Plot2D: title the window Plot2D, not HistogramPlot
  Plot2DWidget loads resource/plot_histogram.ui of HistogramPlot, whose
  windowTitle is HistogramPlot, so every rqt_2d_plot window was titled
  HistogramPlot, on ROS 1 and ROS 2; it sets its own title after loadUi()
  now. The <label> of Plot2D in plugin.xml (the rqt plugin menu) was copied
  from HistogramPlot as well.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] sample_histogram, sample_2dplot: run on ROS 2
  sample_hist_pub.py and sample_2d_plot.py: rospy -> ros_compat of
  jsk_rqt_plugins.compat, as the other sample scripts, with python floats and
  ints instead of numpy arrays and integers in the messages (rclpy does not
  take them; the serialized ROS 1 messages are the same).
  sample_histogram.launch.py and sample_2dplot.launch.py are the ROS 2
  counterparts of the ROS 1 launch files. As rqt_histogram_plot and
  rqt_2d_plot, the sample scripts are installed, and
  test/ros2/test_rqt_plot_jsk_recognition_msgs.py (both plugins come up and
  subscribe the topics of the samples) is added, only when
  jsk_recognition_msgs is found.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* depend on jsk_ros1_ros2_compat only on ROS 2 and ROS One
  jsk_ros1_ros2_compat is in jsk_common >= 3.0, which is released for ROS 2
  and ROS One only; on indigo, kinetic, melodic and noetic rosdep could not
  resolve the exec_depend, and the packages now fall back to rospy without it.
  So the exec_depend of jsk_rqt_plugins, jsk_rviz_plugins and
  jsk_interactive_marker is conditioned on
  "$ROS_VERSION == 2 or $ROS_DISTRO == one" (checked with catkin_pkg 1.1.1
  and 0.4.16 of Ubuntu 20.04: it is evaluated true on jazzy and one, false on
  indigo and noetic). Their build dependencies on it were ROS 2 only already.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] compat.py: fall back to rospy without jsk_ros1_ros2_compat on ROS 1
  jsk_ros1_ros2_compat is in jsk_common >= 3.0, which is released for ROS 2
  and ROS One only, not for indigo, kinetic, melodic and noetic, where the
  plugins and the sample scripts have to work as before without it.
  src/jsk_rqt_plugins/compat.py gives ros_compat and ROS_VERSION:
  jsk_ros1_ros2_compat's rospy_rclpy_compat if it can be imported, else
  (ROS 1 only) a class of staticmethods forwarding the functions used in this
  package to rospy as rospy_rclpy_compat does on ROS 1, as
  ros_speech_recognition/compat.py of jsk_3rdparty. The plugins and the
  sample scripts import them from jsk_rqt_plugins.compat.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] HistogramPlot, Plot2D: run on ROS 2 when jsk_recognition_msgs is found
  hist.py (rqt_histogram_plot) and plot_2d.py (rqt_2d_plot) run on ROS 2 too,
  as Plot3D and StringLabel do: rqt_plot's ROSData and the TopicCompleter get
  the node rqt gives the plugin, the image publisher, the start time and the
  logs go through rospy_rclpy_compat, the .ui file is found with ament_index,
  ROSData.next() is util.next_data(). ROS 2's ROSData needs the type of the
  topic when it subscribes, so a topic given on the command line before its
  publisher is discovered is subscribed again until it resolves. rclpy gives
  arrays of numbers as numpy arrays / array.array, which HistogramPlot plots
  like a list. Their Qt timers stop in shutdown_plugin().
  Both skip drawing until Qt has sized the canvas: savefig() of a figure of
  zero size raises ("tile cannot extend outside image"), e.g. in the default
  standalone window under Xvfb, on ROS 1 as well.
  jsk_recognition_msgs is not released for ROS 2 yet, so on ROS 2
  rqt_histogram_plot and rqt_2d_plot are installed only when
  find_package(jsk_recognition_msgs QUIET) finds it, and package.xml has the
  ROS 2 dependencies (jsk_recognition_msgs, python3-sklearn) only with
  JSK_RECOGNITION_MSGS_FROM_SOURCE=1, as jsk_rviz_plugins does.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] ImageView2Plugin: run on ROS 2
  image_view2 is released for ROS 2 (jsk_common 3.x), so ImageView2Plugin
  (rqt_image_view2) runs on ROS 2 too: <topic>/marked is subscribed and
  <topic>/event published through rospy_rclpy_compat on the node rqt gives the
  plugin, the image topics are listed through the node (sensor_msgs/msg/Image),
  the stamps come from rospy_rclpy_compat.stamp_now(). The unused
  `from std_msgs.msg import Time` is dropped, std_msgs/Time does not exist on
  ROS 2. rqt_image_view2 is installed on ROS 2 and the ROS 2 test checks that it
  comes up.
  image_view2 3.0.5 of ROS 2 publishes marked at its node's namespace; the
  <image>/marked ROS 1 has (and this plugin subscribes) is on jsk_common master
  (jsk-ros-pkg/jsk_common@886c73e), i.e. the next jsk_common release.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] YesNoButton: run on ROS 2
  jsk_gui_msgs is released for ROS 2 (jsk_common_msgs 5.0.2), so YesNoButton
  (rqt_yn_btn) runs on ROS 2 too: the rqt_yn_btn service is created through
  rospy_rclpy_compat on the node rqt gives the plugin, the .ui file is found
  with ament_index on ROS 2 and the advertised services are listed through the
  node. The handler fills the response it is given (the (request, response)
  form of rospy_rclpy_compat.create_service()), which on ROS 1 is the same
  YesNoResponse as before. rqt_yn_btn is installed on ROS 2 and the ROS 2 test
  checks its service.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] util: list(zip()) of the fields of ROS 1, for check_python3
  check_python3 runs 2to3 with the zip fixer, which wants list(zip(...)) for the zip() returned
  by _fields_and_types() on ROS 1 (6547544). The callers iterate it once, so a list behaves the
  same.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] add the ROS 2 tests
  test/test_rqt_plugins.test (rostest) and test/test_util.py (nose) are
  ROS 1 tests. Their ROS 2 counterparts are in test/ros2/, where ROS 1's
  catkin_add_nosetests(test) does not look:
  - test_rqt_plugins.py, a launch_testing test taken from the ros2 branch
  (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_): it starts the plugins that run on ROS 2 with their sample
  publishers/services, checks that they come up, stay alive and
  subscribe the topics they should, and that none crashed on shutdown
  (SIGABRT/SIGSEGV are accepted there: under software OpenGL the
  processes can die from them while tearing down their Fast DDS
  subscriptions). It needs an X display and skips itself without one;
  CI runs it under Xvfb.
  - test_util.py, the pytest of get_slot_type_field_names(); the messages
  of the ROS 1 test (jsk_rviz_plugins/OverlayText,
  jsk_recognition_msgs/Histogram) are not released for ROS 2, so it
  uses visualization_msgs/Marker, std_msgs/Float64MultiArray and
  diagnostic_msgs/DiagnosticArray, which have the same kinds of fields.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] ServiceButtons: show a button without its icon when the image can not be found
  resource/service_button_layout.yaml, used by the samples and the
  tests, takes the icon of Button A from jsk_rviz_plugins
  (package://jsk_rviz_plugins/icons/hand-grasp-pose.jpg). Resolving a
  package:// URL of a package that is not installed raises (rospkg's
  ResourceNotFound, ament_index_python's PackageNotFoundError), which
  made the whole plugin fail to load; jsk_rviz_plugins is not ported to
  ROS 2 yet, so that is always the case there. The button is shown
  without the icon and a warning is logged instead, as for an image file
  that does not exist.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] add the ROS 2 launch files of the samples
  sample/\*.launch are ROS 1 launch files (roslaunch XML with rosparam,
  $(find), $(anon)); their ROS 2 counterparts are sample/\*.launch.py,
  taken from the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_), for the plugins that run on ROS 2:
  sample_3dplot, sample_drc_mini_maxwell, sample_service_buttons,
  sample_service_radio_buttons and sample_tabbed_buttons (whose
  ~tabbed_layout is given as tabbed_layout.\* parameters). The ROS 2 build
  installs them.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] follow the ROSData.next() rename of rqt_plot
  rqt_plot renamed ROSData.next() to next_data() in ROS 2 after jazzy,
  which leaves Plot3D and StringLabel without data on lyrical and
  rolling. util.next_data() calls whichever the ROSData has, as on the
  ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); ROS 1 and jazzy keep calling next().
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] stop the Qt timers of the plugins in shutdown_plugin()
  On ROS 2 the Qt timers of StringLabel, StatusLight, DRCEnvironmentViewer
  and Plot3D keep firing after rqt has shut the rclpy context down, and
  querying the graph or the clock from a destroyed context raises
  RCLError. shutdown_plugin() now stops them (and Plot3D closes its
  subscriptions), as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); that is also what
  shutdown_plugin() is meant for on ROS 1.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] Plot3D: do not draw the 3d axes before Qt sized the canvas
  matplotlib >= 3.6 (noble: 3.6.3, so jazzy and ROS-O) raises
  ValueError("'box_aspect' and 'fig_aspect' must be positive") when it
  lays out 3d axes on a canvas of zero size, which is what the canvas is
  until Qt lays the widget out; rqt_3d_plot printed the traceback on
  every start (bin/rqt_3d_plot's excepthook keeps it from aborting).
  There is nothing to show at that size, so Canvas.draw() and redraw()
  return early then, as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_).
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] Plot3D: retry the topics of the command line until they resolve
  On ROS 2, rqt_plot's ROSData looks the type of the topic up when it
  subscribes, so a topic given on the command line fails ("Can not
  resolve topic type") when rqt_3d_plot starts before its publisher has
  been discovered, which is the usual case; ROS 1 subscribed lazily.
  add_topic() now tells whether it subscribed, and the initial topics
  are retried every second until they do, as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_).
  On ROS 1 they all succeed the first time, as before.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] ServiceButtons, ServiceTabbedButtons: read ~layout_yaml_file and ~tabbed_layout on ROS 2
  A ROS 2 parameter exists only after the node declared it, so
  has_param() is false for a ~layout_yaml_file given on the command line
  or in a launch file; get_param() declares it (taking the given value)
  and is used to tell whether it is set instead.
  A ROS 2 parameter can not be a dictionary, so ~tabbed_layout of
  ServiceTabbedButtons is given as tabbed_layout.tab_list and
  tabbed_layout.<tab>.<type|name|yaml_file|namespace> (what a YAML
  parameter file makes of the nested dictionary of ROS 1), as on the ros2
  branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_), and put back together into the dictionary ROS 1 reads,
  so that the rest of the code is shared.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] ServiceButtons: call the services through the node on ROS 2
  ROS 2 has no rospy.ServiceProxy: the button creates a client on the
  node and calls the service asynchronously. rqt spins the node in its
  own thread, which completes the call, so the button waits for the
  future (up to SERVICE_TIMEOUT seconds, which rospy did not need since
  the call blocked until the reply) instead of spinning the node a second
  time, as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_). A missing service or a timeout
  raises ServiceException, which the button reports like rospy's.
  The positional arguments fill the request fields in order, as rospy's
  ServiceProxy does (SetBool's `data`).
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] Plot3D: ROS 2 counterparts of the rqt_plot/rqt_py_common/rosgraph/rospkg helpers
  Plot3D uses helpers whose ROS 2 version differs or does not exist; on
  ROS 2 (ROS 1 is unchanged), as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_):
  - rqt_py_common.topic_helpers.is_slot_numeric() is gone; rqt_plot's
  plot_widget.is_plottable(node, topic) tells whether a topic can be
  plotted
  - TopicCompleter.update_topics() and rqt_plot's ROSData list and
  subscribe the topics through the node, which they take as an argument
  - rosgraph.names.script_resolve_name() is gone; the topics of the
  command line are resolved in the namespace of the node, like rqt_plot
  does on ROS 2
  - rospkg is gone; the .ui file is found with ament_index_python
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] StringLabel: give rqt_plot's ROSData the node on ROS 2
  rqt_plot's ROSData, which StringLabel's ROSData derives from, subscribes
  the topic through the node it takes as its first argument on ROS 2
  (ROSData(node, topic, start_time)), as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_).
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] DRCEnvironmentViewer: builtin_interfaces/Time for the whiteout time on ROS 2
  std_msgs/Time does not exist on ROS 2, so mini_maxwell.py and
  sample_drc_mini_maxwell.py use builtin_interfaces/Time, the time itself
  (no `data` field), for /drc_2015_environment/next_whiteout_time there,
  as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); ROS 1 keeps std_msgs/Time. The other
  plugins imported std_msgs/Time without using it, which failed on ROS 2;
  those imports are dropped.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] StatusLight: unsubscribe the previous topic through the node on ROS 2
  An rclpy subscription has no unregister(); the node destroys it, as on
  the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_).
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] StringLabel, StatusLight: list the topics through the node on ROS 2
  StringLabel and StatusLight offer the published topics of the right
  type in their configuration dialog, which they got from the ROS master
  (rospy.get_published_topics()). ROS 2 has no master: the node lists the
  topics it discovered (get_topic_names_and_types(), with the types as
  'pkg/msg/Type'), as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_). The message class is
  resolved with ros_compat.resolve_message_class(), which takes both
  spellings; a type whose package is not installed is skipped (rclpy
  raises where roslib returned None).
  StringLabel also offered the parameter names (rospy.get_param_names())
  as candidates; ROS 2 has no parameter server to list, so there are none.
  rospy is imported on ROS 1 only.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] util.get_slot_type_field_names: read the fields of ROS 2 messages
  get_slot_type_field_names() (used by StringLabel to list the string
  fields of the published topics) walked rospy's __slots_\_/_slot_types
  and resolved nested types with roslib, which does not exist on ROS 2.
  There it reads get_fields_and_field_types(), whose arrays are
  'sequence<type>' (or 'sequence<type, N>') instead of 'type[]', and
  resolves the nested types with rosidl_runtime_py's get_message(), which
  raises for the builtin types; the ROS 1 path is unchanged. As on
  the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_).
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] move ROSData to util.py and the dialogs to dialogs.py
  label.py (StringLabel) took ROSData from hist.py, and status_light.py
  (StatusLight) took ComboBoxDialog from image_view2_wrapper.py, so these
  plugins imported jsk_recognition_msgs, cv_bridge and image_view2 just
  for those helpers. jsk_recognition_msgs and image_view2 are not
  released for ROS 2, which kept StringLabel and StatusLight from loading
  there. ROSData now lives in util.py and the dialogs in a new dialogs.py,
  as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); hist.py and image_view2_wrapper.py import
  them from there, so their names stay available.
  LineEditDialog takes its completion candidates as an argument instead
  of reading rospy.get_param_names() itself; StringLabel passes them.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] build on ROS 1 (catkin) and ROS 2 (ament) from one package.xml/CMakeLists.txt
  package.xml: the dependencies only ROS 1 has (catkin, rosbuild, mk,
  message_generation/runtime, rostest, ...) and those of the plugins that
  stay ROS 1 only get condition="$ROS_VERSION == 1"; the ament build tools
  and the dependencies rqt_plot and friends pull in implicitly on ROS 1
  only get condition="$ROS_VERSION == 2"; build_type follows ROS_VERSION.
  CMakeLists.txt: the ROS 1 part is unchanged; the ROS 2 (ament) build is
  prepended in `if("$ENV{ROS_VERSION}" EQUAL 2) ... else()`, and the file
  ends with `endif()`. The ROS_VERSION guess for builds without
  ros_environment is the one of jsk_common's packages. The ROS 2 build
  installs the python package, the plugin.xml, the resources and the
  entry points of the plugins that run on ROS 2, the scripts through
  jsk_ros1_ros2_compat_install_python() for a python3 shebang.
  HistogramPlot, Plot2D, ImageView2Plugin and YesNoButton need
  jsk_recognition_msgs, image_view2 or jsk_gui_msgs, which are not
  released for ROS 2, so they are not installed on ROS 2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rqt_plugins] rospy -> jsk_ros1_ros2_compat
  The plugins and the sample scripts that are going to run on ROS 2 call
  the ROS API through jsk_ros1_ros2_compat's rospy_rclpy_compat, which is
  rospy on ROS 1 and rclpy on ROS 2, so that one source serves both.
  `node` is None on ROS 1 and, in the plugins, the rclpy node rqt creates
  for them on ROS 2 (the plugin context's `node`); the widgets take it as
  a new trailing `node=None` keyword argument, which keeps their ROS 1
  signature.
  The mechanical part was done by
  cd jsk_rqt_plugins
  SRC="src/jsk_rqt_plugins/button_general.py src/jsk_rqt_plugins/tabbed_button_general.py src/jsk_rqt_plugins/label.py src/jsk_rqt_plugins/mini_maxwell.py src/jsk_rqt_plugins/status_light.py src/jsk_rqt_plugins/plot.py"
  SAMPLES="sample_scripts/sample_3d_plot.py sample_scripts/sample_drc_mini_maxwell.py sample_scripts/sample_service_buttons.py sample_scripts/sample_service_radio_buttons.py"
  sed -i 's/^import rospy$/import rospy\nfrom jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat/' $SRC $SAMPLES
  sed -i -E 's/rospy\.(loginfo|logwarn|logerr|get_param|has_param)\(/ros_compat.\1(self._node, /; s/rospy\.(get_time|Time\.now)\(\)/ros_compat.now_sec(self._node)/; s/rospy\.Subscriber\(/ros_compat.create_subscription(self._node, /' $SRC
  sed -i -E 's/rospy\.init_node\(/node = ros_compat.init_node(/; s/rospy\.Publisher\(/ros_compat.create_publisher(node, /; s/rospy\.Service\(/ros_compat.create_service(node, /; s/rospy\.(\w+)\(\)/ros_compat.\1(node)/g; s/rospy\.(Rate|sleep|loginfo)\(/ros_compat.\1(node, /' $SAMPLES
  sed -i -E 's/(ros_compat\.\w+\((self\._)?node,) $/\1/' $SRC $SAMPLES
  for f in $SRC $SAMPLES; do grep -q 'rospy\.' $f || sed -i '/^import rospy$/d' $f; done
  and by hand:
  - pass the plugin context's node to the widgets (button.py, radio_button.py,
  tabbed_button.py, label.py, status_light.py, mini_maxwell.py, plot.py)
  and keep it as self._node
  - log\*(): one str message (label.py)
  - time: mini_maxwell.py keeps the times as seconds (now_sec(),
  stamp_to_sec()) instead of rospy.Time
  - samples: publish messages instead of bare values, which rclpy does not
  wrap, and services take (req, res) and fill the given response
  (std_srvs' \*Response classes do not exist on ROS 2)
  - package.xml: exec_depend on jsk_ros1_ros2_compat
  What has no rospy_rclpy_compat counterpart (the ROS graph queries, the
  service client, ROSData, the parameter dictionary of the tabbed buttons,
  std_msgs/Time, ...) is left on rospy here and handled one by one in the
  following commits. hist.py, plot_2d.py, image_view2_wrapper.py and
  yes_no_button.py depend on packages not released for ROS 2
  (jsk_recognition_msgs, image_view2, jsk_gui_msgs) and stay ROS 1 only.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request `#904 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/904>`_ from k-okada/24_04_test
  add ROS-O 24.04 build test
* PyQt5/6 aborts the whole process on an unhandled exception raised inside a Qt-invoked slot (e.g. a QTimer-driven redraw). Since this node runs under roslaunch with required=true, such an abort tears down the whole launch. Log instead of aborting.
* add more try/except to pass ROS-O test
* use int() for python2/3
* fix hist.py, mini_maxwell.py, status_light.py for python2/python3 compatible
* Contributors: Kei, Kei Okada

2.1.10 (2024-12-11)
-------------------

2.1.9 (2024-12-11)
------------------
* add more visualization options for BoundingBox and BoundingBoxArray (`#844 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/844>`_)
* [ROS-O] patches (`#891 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/891>`_)

  * drop ROS_DISTRO STRGREATER tests
    As the Debian packages use "Debian"
    https://salsa.debian.org/science-team/ros-ros-environment/-/blame/master/debian/rules?ref_type=heads#L6
    which is incompatible, does not trigger them
    c++14 is the default from melodic onward, so the additional statements
    there are not necessary

* face_detector is released, so remove .travis.rosinstall.noetic . add test to check with latest pyyaml from pip (`#884 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/884>`_)

  * [jsk_rqt_plugins] Fix yaml load in tabbed_button_general.py

* update perspective for noetic (`#873 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/873>`_)
* drop obsolete (and broken urlgrabber dependency) (`#867 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/867>`_)
* add test to check (ServiceButtonGeneralWidget_in_tab (`#880 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/880>`_)

  * Fix super function argument in tabbed_button_general
  * add test to check https://github.com/jsk-ros-pkg/jsk_visualization/pull/879

* use QFileDialog for file select (`#876 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/876>`_)
* fix plugin category (`#877 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/877>`_)
* return int in mouse event (`#874 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/874>`_)

* Contributors: Naoaki Kanazawa, Kei Okada, Shingo Kitagawa, Yoshiki Obinata, v4hn

2.1.8 (2022-01-11)
------------------
* [jsk_interactive_markers][jsk_rqt_plugins] fix yaml load (`#818 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/818>`_)
* fix with '2to3 -w -f zip .' (`#817 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/817>`_)

  * In python3, sys.maxint changed to sys.maxsize
  * fix "TypeError: 'dict_keys' object is not subscriptable" error
    we can fix this by '2to3 -w -f dict', but it will change too many lines, so we do not use this for now....
  * fix with '2to3 -w -f import .'
  * fix cStringIO.StringIO -> io.BytesIO, see https://stackoverflow.com/questions/11914472/stringio-in-python3 / https://stackoverflow.com/questions/50797043/string-argument-expected-got-bytes-in-buffer-write
  * [jsk_rqt_plugins][jsk_rviz_plugins] Fix has_key

* add tabbed_buttuns to rqt_plugin (`#813 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/813>`_)
* update .travis to 0.5.21 (`#814 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/814>`_)
* Updating the URLs of the jsk_rviz_plugins and jsk_rqt_plugins so the generated README points to working links (closes #805 ). (`#806 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/806>`_)
* Support single button setup in ServiceButtons plugin (`#798 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/798>`_)

  * `*column_indices` expects to be an array of integers and the length
  should be equal or longer than 2.
  * If column_indices is just an array of a single integer (length is
  1), do not apply sum with expanding a list argument.

* fix for using button_general.py without perspective file (`#796 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/796>`_)
* [jsk_rqt_plugins/button_general.py] reset button if SetBool fail (`#794 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/794>`_)
* sample_service_button.py support SetBool (`#792 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/792>`_)
* add sample_service_radio_buttons script (`#789 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/789>`_)
* add sample_service_buttons scriptf (`#790 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/790>`_)
* add sample_service_buttons.launch (`#791 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/791>`_)

* Contributors: Adi Vardi, Kei Okada, Naoki Hiraoka, Ryohei Ueda, Sam Pfeiffer, Shingo Kitagawa, Yohei Kakiuchi

2.1.7 (2020-10-17)
------------------
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * run 2to3 -f except
  * support noetic, conver to package format 3

* Contributors: Kei Okada

2.1.6 (2020-04-13)
------------------
* fix test failure in jsk_rqt_plugins (`#766 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/766>`_)

  * disable jsk_rqt_plugins test in indigo
    this is because indigo does not support matplotlib
    liner_model.RANSACRegressor.

* [jsk_rqt_plugins] load rosparam correctly in button_general.py (`#746 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/746>`_)
* Replace image_pipeline with image_publisher (`#729 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/729>`_)

  * Replace image_pipeline with image_publisher
    * image_pipeline is a meta package and normal ros packages are not
    intended to depend on meta packages.

* Contributors: Ryohei Ueda, Shingo Kitagawa

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Add comment for install destination
  * Install missing test/ as well
  * Use source permission when installing executables
  * Fix installation destination
* [jsk_rqt_plugins] Fix for working correctly on kinetic + qt5 (`#708 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/708>`_)
  * Add sample for rqt_image_view2
  * Update test for rqt_drc_mini_maxwell
  * Add sample for rqt_drc_mini_maxwell
  * Update test for rqt_3d_plot
  * Add sample for rqt_3d_plot
  * Fix for substituting to 'intercept'
  * Specify version of image_pipeline
  * Support matplotlib backend for qt5
  * Support qt>=5 in rqt plugins
  * Add dependency for test
  * Set retry=3 for test
  * Install all programs in jsk_rqt_plugins/bin/
  * Add test for program in jsk_rqt_plugins
  * use matplotlib.backends.backend_qt5agg in Qt5
  * add queue_size in publishers
  * check opencv version for kinetic
  * follow new sklearn ranac linear regression
  * use correct module for qt5
  * refactor plot_2d.py

* [jsk_rqt_plugins] fix hist.py to work on kinetic (`#707 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/707>`_)
* Revert "[jsk_rqt_plugins/HistogramPlot] modify to correctly run on kinetic (`#688 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/688>`_)" (`#704 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/704>`_)
  This reverts commit 79dcedf7fd761fc638f1cffc1cfaa46a7bb4f1a2.
* [jsk_rqt_plugins/HistogramPlot] modify to correctly run on kinetic (`#688 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/688>`_)
  * refactor codes in hist.py
  * use correct module of qt5 for kinetic
  * check opencv version for kinetic
  * add queue_size in publisher
* [jsk_rqt_plugins] Add dependency of sklearn (`#701 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/701>`_)
* Add <url> to package.xml to add link to README (`#681 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/681>`_)
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Shingo Kitagawa, Yuto Uchimi, Iory Yanokura

2.1.3 (2017-10-26)
------------------
* PR `#672 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/672>`_ needs jsk_gui_msgs 4.3.0 (https://github.com/jsk-ros-pkg/jsk_common_msgs/pull/18) (`#673 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/673>`_)
* Display message on rqt_yn_btn (`#672 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/672>`_)
* Contributors: Kei Okada, Kentaro Wada

2.1.2 (2017-07-07)
------------------

2.1.1 (2017-02-15)
------------------

2.1.0 (2017-02-13)
------------------

2.0.1 (2016-12-15)
------------------

2.0.0 (2016-12-14)
------------------

1.0.34 (2016-09-29)
-------------------

1.0.33 (2016-09-13)
-------------------

1.0.32 (2016-07-20)
-------------------

1.0.31 (2016-05-19)
-------------------
* Warn about unsupported topic type
* Check class type of data instead of subscribed topic type in rqt_histgram_plot to support HistgramWithRangeArray
* Contributors: Kentaro Wada, Iori Kumagai

1.0.30 (2016-03-25)
-------------------

1.0.29 (2016-03-20)
-------------------

1.0.28 (2016-02-03)
-------------------
* Except NavigationToolbar ImportError from matplotlib
  this is caused with upgraded matplotlib.
  reported at: https://github.com/semiautomaticgit/SemiAutomaticClassificationPlugin/issues/2
  Modified:
  - jsk_rqt_plugins/src/jsk_rqt_plugins/hist.py
  - jsk_rqt_plugins/src/jsk_rqt_plugins/plot.py
  - jsk_rqt_plugins/src/jsk_rqt_plugins/plot_2d.py
* [jsk_rqt_plugins] Support min-max fields of PlotDataArray in
  rqt_2d_plot
* [jsk_rqt_plugins/plot_2d] Update to support new
  jsk_recognition_msgs/PlotData fields
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.27 (2015-12-08)
-------------------
* [jsk_rqt_plugins] Avoid already advertised error for rqt_yn_btn
* Contributors: Kentaro Wada

1.0.26 (2015-12-03)
-------------------
* [jsk_rqt_plugins] Advertise service after initialized
* Contributors: Kentaro Wada

1.0.25 (2015-10-10)
-------------------
* [jsk_rqt_plugins] Fit line to date by ransac
* [jsk_rqt_plugins] Move README to sphinx + readthedocs
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.24 (2015-09-08)
-------------------
* Fix get_slot_type_field_names for None msg
* [jsk_rqt_plugins] Add more plot options for rqt_2d_plot
* [jsk_rqt_plugins/rqt_2d_plot] Add sample
* [jsk_rqt_plugins/plot_2d] Show grid
* [jsk_rqt_plugins] Add script to plot data as scatter
* [jsk_rqt_plugins] Add util to get slot field
* [jsk_rqt_plugins] Depends on rqt_image_view
* [jsk_rqt_plugins/rqt_histogram_plot] Publish rendered image from
  rqt_histogram_plot.
  In order to overlay the image on rviz
* [jsk_rqt_plugins] Update README about rqt_histogram_plot
* [jsk_rqt_plugins/rqt_histogram_plot] Support legend
* [jsk_rqt_plugins/histogram_plot] Support
  jsk_recognition_msgs/HistogramWithRange message to specify
  x-axis values
* [jsk_rqt_plugins] Remove unused comments
* [jsk_rqt_plugins][rqt_string_label] Display more candidates which has string field
* [jsk_rqt_plugins] Catch error for unexpected message type
* [jsk_rqt_plugins] rqt_string_label supports nested string as well as std_msgs/String
* [jsk_rqt_plugins][button_general.py] Make combo_box shared
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------

1.0.22 (2015-06-24)
-------------------

1.0.21 (2015-06-11)
-------------------
* [jsk_rqt_plugins] Add sign for configuration button in the README image of rqt_service_button
* [jsk_rqt_plugins] catch import error and use roslib in rqt_yn_btn
* [jsk_rqt_plugins] Add README for rqt_service_button
* [jsk_rqt_plugins] Add rqt_yn_btn
* [jsk_rqt_plugins] generate button groups
* Contributors: Kentaro Wada, Masaki Murooka

1.0.20 (2015-05-04)
-------------------
* [jsk_rqt_plugins/iamge_view2_wrapper] Use thread to update image topic
  list isntead of QTimer not to hung up rqt_gui
* use button general class for push button and radio button. enable to set parameter name to set layout
* add radio button plugin
* display label and icon in button
* [jsk_rqt_plugins] Add python-urlgrabber dependency
* Contributors: Masaki Murooka, Ryohei Ueda

1.0.19 (2015-04-09)
-------------------
* [jsk_rqt_plugins] Use parens to import a lot of symbols from modules
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics and redraw
  in StatusLightWidget
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics in DRCEnvironmentViewerWidget
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics in StringLabelWidget
* [jsk_rqt_plugins] Optimize image_view2_wrapper:
  1. Use signal to tell redraw event from subscription callback
  2. Fix periodic duration to call updateTopics
* [jsk_rqt_plugins] Add image_view2 to build depend
* [jsk_rqt_plugins] Support move event without clicking mouse in
  image_view2 rqt wrapper
* [jsk_rqt_plugins] Fix for handling right click in rqt_image_view2 wrapper
* [jsk_rqt_plugins] Lower frequency to update StatusLight
* [jsk_rqt_plugins] Do not redraw image if no needed in image_view2 wrapper
* [jsk_rqt_plugins] Change message type to uint8 from int32 in Status plugin
* [jsk_rqt_plugins] Fix typo
* [jsk_rqt_plugins] Add simple widget to visualize status
* [jsk_rqt_plugins] Add settings button and remove combo box from top view
* [jsk_rqt_plugins] Do not convert invalid ROS images to OpenCV images in rqt_image_view2
* [jsk_rqt_plugins] Update label setting of StringLabel plugin.
  * Use larger font. font size is 14
  * Align text to left
  * Enable word wraping
* [jsk_rqt_plugins] Use topic stored in perspective file in StringLabel plugin
* [jsk_rqt_plugins] Simple widget to display std_msgs/String
* [jsk_rqt_plugins] Add ComboBox and store setting in rqt_image_view2
* [jsk_rqt_plugins] Fix drawing of rqt_image_view2 when size is not usual
* [jsk_rqt_plugins] Read configuration of buttons from private parameter space
* [jsk_rqt_plugins] Add perspective to combine rqt_image_view2 and rqt_service_button
* [jsk_rqt_plugins] image_view2 rqt wrapper
* [jsk_rqt_plugins] Support image for button icons in rqt_service_button
* [jsk_rqt_plugins] Add simple rqt plugin to list buttons to call empty
  services and configurable by yaml file
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.18 (2015-01-30)
-------------------

1.0.17 (2015-01-29)
-------------------
* [jsk_rqt_plugins] Add simple viewer to visualize mini maxwell status.
* Contributors: Ryohei Ueda

1.0.16 (2015-01-04)
-------------------

1.0.15 (2014-12-13)
-------------------

1.0.14 (2014-12-09)
-------------------

1.0.13 (2014-10-10)
-------------------

1.0.12 (2014-09-23)
-------------------

1.0.11 (2014-09-22)
-------------------

1.0.10 (2014-09-13)
-------------------

1.0.9 (2014-09-07)
------------------

1.0.8 (2014-09-04)
------------------

1.0.7 (2014-08-06)
------------------

1.0.6 (2014-07-14)
------------------

1.0.5 (2014-06-29)
------------------
* fix jsk_rqt_plugins for groovy users
* only one topic should be taken into account. the argument of the topics
  cannot be an array
* add rqt plugin to visualize histogram
* Contributors: Ryohei Ueda

1.0.4 (2014-05-31)
------------------
* jsk_rqt_plugins: add mk/rosbuild to build_depend
* jsk_rqt_plugins) install missing .ui file
* Contributors: Kei Okada, Isaac IY Saito

1.0.3 (2014-05-22)
------------------

1.0.2 (2014-05-21)
------------------

1.0.1 (2014-05-20)
------------------

1.0.0 (2014-05-17)
------------------

0.0.3 (2014-05-15)
------------------

0.0.2 (2014-05-15)
------------------
* add --no-legend option to disable legend
* support polygon mode. if you want to plot in line mode, please add -L option
* implement 3d plotter
* add jsk_rqt_plugins directory
* Contributors: Ryohei Ueda
