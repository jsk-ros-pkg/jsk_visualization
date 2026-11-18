^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* README: alt text of the gallery images
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* depend on jsk_recognition_msgs/utils on ROS 2 without \*_FROM_SOURCE
  They are going to be released for ROS 2 soon, so the release of
  jsk_visualization depends on them as on ROS 1 (the ROS 2 binaries fail
  on the buildfarm until they are released). The ROS 2 CI checks out
  jsk_recognition_utils as well as jsk_recognition_msgs from source, and
  no longer sets JSK_RECOGNITION_MSGS_FROM_SOURCE.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Improve README clarity on sample configurations
  Clarify the description of sample launches and their behavior regarding rviz and data publishing.
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
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
* [jsk_rviz_plugins] PolygonArray: use jsk_recognition_utils on ROS 2 as well
  jsk_recognition_utils builds on ROS 2 now (jsk-ros-pkg/jsk_recognition,
  recognition-utils-ros2), so PolygonArrayDisplay includes
  <jsk_recognition_utils/geo/polygon.h> on ROS 2 as on ROS 1, and the copy of
  its Polygon and ear clipping, src/geo_polygon_compat.h, is removed.
  - CMakeLists.txt (ROS 2 part only): find_package(jsk_recognition_utils
  QUIET) when jsk_recognition_msgs is found; PolygonArray, its sample and
  test are built/installed only when it is found, otherwise skipped with a
  STATUS message. The ROS 1 part is unchanged.
  - PolygonArray moves from rviz_common_plugins_jsk_recognition_msgs.xml to
  rviz_common_plugins_jsk_recognition_utils.xml, exported only then.
  - package.xml: jsk_recognition_utils is a ROS 2 dependency only with
  JSK_RECOGNITION_UTILS_FROM_SOURCE=1, as jsk_recognition_msgs is with
  JSK_RECOGNITION_MSGS_FROM_SOURCE=1, since it is not released for ROS 2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
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
* [jsk_rviz_plugins] EmptyServiceCallInterface: show an empty panel without ~rviz_service_call/buttons
  ROS 1: XmlRpcValue::size() of the missing ~rviz_service_call/buttons (an
  invalid value) throws XmlRpcException, which aborted rviz whenever the panel
  was loaded without the parameter (e.g. from a saved config); so did a value
  of another type than a list of {icon, service_name, text} structs. The
  parameter is checked first now, and a missing or malformed one is warned
  about and leaves the panel empty.
  ROS 2: a parameter of another type (e.g. an integer for
  rviz_service_call.buttons) made declare_parameter()/get_value() throw, and
  rviz2 dropped the whole display config; a missing icon package threw as
  well. They are caught, warned about and leave the panel empty; a missing
  parameter is warned about.
  The panel is the same as before when the parameters are set.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] polygon_array_sample: run on ROS 2
  polygon_array_sample.py: rospy -> jsk_ros1_ros2_compat (rospy fallback of
  jsk_rviz_plugins.rospy_compat on ROS 1). Its rate of 10 Hz is slept on ROS 2
  only: on ROS 1 the rate is never slept and the polygons are published as
  fast as possible, which is left as it is.
  polygon_array_sample.launch.py with config/ros2/polygon_array_sample.rviz
  (the display and view of the ROS 1 config) is its ROS 2 counterpart,
  installed and tested (test/ros2/test_polygon_array.py) when
  jsk_recognition_msgs is found, as the other jsk_recognition_msgs samples.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] PolygonArray: build on ROS 2 when jsk_recognition_msgs is found
  PolygonArrayDisplay uses jsk_recognition_utils::Polygon (geo/polygon.h) for
  fromROSMsg(), getVertices(), getNormal() and decomposeToTriangles(); the
  latter triangulates with the ear clipping of jsk_recognition_utils'
  pcl::EarClippingPatched. jsk_recognition_utils is not ported to ROS 2, so
  src/geo_polygon_compat.h provides that part of jsk_recognition_utils::Polygon
  on ROS 2 (header only, on Eigen, the ear clipping written out as in
  ear_clipping_patched.cpp) and includes jsk_recognition_utils/geo/polygon.h on
  ROS 1, as color_utils_compat.h does for jsk_topic_tools/color_utils.h; the
  ROS 1 build is unchanged.
  The triangles and normals are the same: for the 4 polygons of
  polygon_array_sample.py and 200 random (also non planar) ones, in both
  orientations, jsk_recognition_utils of ROS One and geo_polygon_compat.h of
  the ROS 2 build give the same output.
  The display joins the optional jsk_recognition_msgs block of the ROS 2 build
  and rviz_common_plugins_jsk_recognition_msgs.xml: rviz_common/rviz_rendering
  headers, ConstSharedPtr on ROS 2. SimpleOccupancyGridArray stays ROS 1 only
  (planes, PCL conversions of jsk_recognition_utils).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] boundingbox_sample, segment_array_sample: run on ROS 2
  bounding_box_sample.py and segment_array_sample.py: rospy ->
  jsk_ros1_ros2_compat, with the rospy fallback of jsk_rviz_plugins.rospy_compat
  (which gets get_param()) on ROS 1 as the other samples. Floats for the float
  fields and keyword arguments for the messages (rclpy), and
  quaternion_about_axis() of tf.transformations (ROS 1 only) for the z axis
  written out in bounding_box_sample.py.
  boundingbox_sample.launch.py and segment_array_sample.launch.py with
  config/ros2/\*.rviz (the displays and views of the ROS 1 configs) are their
  ROS 2 counterparts. As the displays, the samples are installed and their
  tests (test/ros2/test_bounding_box.py, test_segment_array.py) added only
  when jsk_recognition_msgs is found. python3-numpy and python3-scipy
  (segment_array_sample.py) are ROS 2 exec_depends as well.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] overlay_sample, piechart_sample: show OverlayText and PieChart on ROS 2
  config/ros2/overlay_sample.rviz shows the OverlayText (/text_sample) and the
  PieChart (/value_sample) of overlay_sample.py, with the properties of the
  ROS 1 config (the OverlayText at the top left, where ROS 1 shows
  rosconsole_overlay_text.py, a ROS 1 only script), and the Plotter2D at its
  ROS 1 position. piechart_sample.launch.py and config/ros2/piechart_sample.rviz
  are the ROS 2 counterparts of piechart_sample.launch, tested by
  test/ros2/test_piechart.py as the other samples.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] OverlayText, PieChart: build the overlay displays on ROS 2
  overlay_text_display and pie_chart_display were left to
  rviz_2d_overlay_plugins on ROS 2, which is a copy of them released by
  others. They are built from the same sources now, as the other overlay
  displays: rviz_common/rviz_rendering headers on ROS 2, the OverlayText
  message of jsk_rviz_plugins/msg (already generated on ROS 2) and
  std_msgs/msg/Float32, a subscription on the node of rviz,
  prepareOverlays() and the RosTopicProperty initialized in onInitialize().
  The ROS 2 includes of overlay_text_display.cpp come after the Qt headers,
  as X11/Xlib.h of rviz_rendering/render_system.hpp defines None.
  They are added to rviz_common_plugins.xml, and OverlayPicker picks them on
  ROS 2 as well (its ROS 1 code is back to that of master).
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
* [jsk_rviz_plugins] samples: fall back to rospy without jsk_ros1_ros2_compat on ROS 1
  jsk_ros1_ros2_compat is in jsk_common >= 3.0, which is released for ROS 2
  and ROS One only, not for indigo, kinetic, melodic and noetic, where the
  samples have to work as before without it.
  The samples using it (overlay_sample.py, overlay_menu_sample.py,
  pictogram.py, pictogram_all.py, piechart_sample.py) import ros_compat (and
  ROS_VERSION) from jsk_ros1_ros2_compat's rospy_rclpy_compat, or, if it can
  not be imported (ROS 1 only), from python/jsk_rviz_plugins/rospy_compat.py:
  the functions they use forwarded to rospy, as rospy_rclpy_compat does on
  ROS 1. It is in the python module of ROS 1 (catkin_python_setup()), not
  next to the samples: catkin installs samples/\*.py with
  catkin_install_python(), whose devel space relays would hide the names of a
  module imported from there.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] build_depend and exec_depend of jsk_recognition_msgs on ROS 2, not depend
  check_dependency failed: catkin_pkg reported the generic dependency of ROS 2 on
  jsk_recognition_msgs as redundant with its build_depend and exec_depend of ROS 1, as it checks
  them without evaluating the conditions, as for jsk_interactive_marker (dbeb34b).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] TorusArray, SegmentArray, HumanSkeletonArray: build on ROS 2 when jsk_recognition_msgs is found
  The displays of TorusArray, SegmentArray and HumanSkeletonArray join the
  optional jsk_recognition_msgs block of the ROS 2 build and
  rviz_common_plugins_jsk_recognition_msgs.xml: rviz_common/rviz_rendering
  headers (rviz::MeshShape is added to the aliases of rviz_compat.h), the
  messages of jsk_recognition_msgs/msg, ConstSharedPtr on ROS 2,
  colorCategory20() of color_utils_compat.h.
  PolygonArray and SimpleOccupancyGridArray stay ROS 1 only: besides
  jsk_recognition_msgs they need jsk_recognition_utils (polygon triangulation,
  planes, PCL conversions), which is not ported to ROS 2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] BoundingBox, BoundingBoxArray: build on ROS 2 when jsk_recognition_msgs is found
  jsk_recognition_msgs is not released for ROS 2 yet (its ROS 2 port is
  `jsk-ros-pkg/jsk_recognition#2903 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2903>`_), so on ROS 2 the displays of its messages
  are optional: CMakeLists.txt builds them only when
  find_package(jsk_recognition_msgs QUIET) finds it, and exports their own
  plugin description file, rviz_common_plugins_jsk_recognition_msgs.xml, then;
  otherwise it says they are skipped. ROS 1 builds them as before.
  package.xml must not make rosdep resolve the unreleased key, so the ROS 2
  dependency is conditional on JSK_RECOGNITION_MSGS_FROM_SOURCE=1, which a
  workspace building jsk_recognition_msgs from source (as the ROS 2 CI will)
  sets for rosdep to take it from the workspace and colcon to build it first.
  The sources (bounding_box_display_common.h, bounding_box_display,
  bounding_box_array_display):
  - rviz_common/rviz_rendering headers, the messages of jsk_recognition_msgs/msg
  through JSK_ROS1_ROS2_COMPAT_MSG_ALIAS, ConstSharedPtr on ROS 2;
  - colorCategory20()/heatColor() of color_utils_compat.h;
  - ROS_WARN_THROTTLE on the logger of jsk_rviz_plugins on ROS 2;
  - rviz_rendering::BillboardLine calls newLine() finishLine(), so on ROS 2 the
  edges are a BillboardLine subclass providing newLine().
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] YesNoButton: build the panel on ROS 2
  jsk_gui_msgs is released for ROS 2 (jsk_common_msgs 5.0.2), so the YesNoButton
  panel is built on ROS 2 too. Its /rviz/yes_no_button service is created on the
  rviz node. rviz_common spins that node in the GUI thread, where the ROS 1 way
  of waiting for a button in the service callback (processEvents() until
  clicked) would re-enter the executor, so on ROS 2 the response is deferred:
  the request enables the buttons, and Yes/No send the response
  (rclcpp::Service::send_response()). A new request while one is pending answers
  the pending one with "no".
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] Footstep: build the display on ROS 2
  jsk_footstep_msgs is released for ROS 2 (jsk_common_msgs 5.0.2), so the
  Footstep display is built on ROS 2 too: rviz_common/rviz_rendering headers
  (rviz::Shape is added to the aliases of rviz_compat.h), the messages of
  jsk_footstep_msgs/msg, processMessage() with ConstSharedPtr.
  jsk_topic_tools/color_utils.h is ROS 1 only in jsk_topic_tools 3.x, so
  src/color_utils_compat.h provides jsk_topic_tools::colorCategory20() and
  heatColor() on ROS 2 (the same colors), and includes
  jsk_topic_tools/color_utils.h on ROS 1; the display uses colorCategory20() for
  "Use Group Coloring".
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] AmbientSound: build the display on ROS 2
  jsk_hark_msgs is released for ROS 2 (jsk_common_msgs 5.0.2), so the
  AmbientSound display (ambient_sound_display_groovy, ambient_sound_visual) is
  built on ROS 2 too: rviz_common/rviz_rendering headers, the HarkPower message
  of jsk_hark_msgs/msg, processMessage()/setMessage() with ConstSharedPtr. The
  forward declarations of the rviz:: properties and the unused
  ROSTopicStringProperty member stay ROS 1 only (aliases of rviz_compat.h on
  ROS 2). ambient_sound_display.{h,cpp} are built on neither.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] add the ROS 2 tests
  test/ros2/test\_{overlay,linear_gauge,pictogram,service_call_panel,
  robot_command_interface}.py run the ROS 2 sample launch files with
  launch_testing and check that rviz2 stays alive, subscribes the topics of the
  config and exits cleanly, from the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_) (there they replaced the
  rostests, here they sit next to them in test/ros2/ with their
  rviz_config_check.py). They need an X display and skip themselves without one.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] add the ROS 2 launch files of the samples
  launch/{overlay,linear_gauge,pictogram}_sample.launch.py,
  robot_command_interface_sample.launch.py and service_call_panel.launch.py, the
  ROS 2 counterparts of the .launch files, with the rviz2 configs and the ROS 2
  parameter files of the panels in config/ros2/ (rviz2 configs and
  ros__parameters files can not be shared with ROS 1), from the ros2 branch
  (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_). The ROS 2 build installs the samples they run with a python3 shebang.
  - overlay_sample.launch.py remaps image_raw of image_publisher, which is
  ~image_raw on ROS 1, to /image_publisher/image_raw the config shows.
  - pictogram_sample.launch.py runs pictogram.py as well, for the Pictogram
  display of its config.
  - ROS 2 parameters can not hold an array of structs, so
  config/ros2/command_samples.yaml and default_robot_command.yaml describe the
  buttons as a list of ids plus one parameter group per id.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] samples: rospy -> jsk_ros1_ros2_compat
  overlay_sample.py, overlay_menu_sample.py, piechart_sample.py, pictogram.py and
  pictogram_all.py, the publishers of the samples which run on ROS 2 as well,
  use jsk_ros1_ros2_compat.rospy_rclpy_compat instead of rospy:
  - rclpy messages take neither positional arguments nor ints for float fields,
  and publish() no message fields, so ColorRGBA(r=, g=, b=, a=), Float32(data=)
  and float values are spelled out; the values are the same on ROS 1.
  - frame_id "base_link" instead of "/base_link": tf2 of ROS 2 rejects the
  leading slash, which ROS 1 strips.
  - the publishers of pictogram(_all).py get queue_size=1 (they had none).
  - overlay_sample.py also publishes /sample_string and /diagnostics_agg on ROS 2
  for the String and OverlayDiagnostic displays of its ROS 2 rviz config, as on
  the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_): OverlayText is in rviz_2d_overlay_plugins there, and
  `ros2 topic pub`, the counterpart of the `rostopic pub` of
  overlay_sample.launch, exits with an error on SIGINT.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] Normal, TFTrajectory, QuietInteractiveMarker, RvizScenePublisher, VideoCapture: build the displays on ROS 2
  - Normal: processMessage() is a slot, so rviz_compat.h is included outside
  Q_MOC_RUN for moc to see the ROS 2 signature; findChannelIndex/validateFloats
  come from rviz_default_plugins/rviz_common.
  - TFTrajectory: rclcpp::Time/Duration.
  - QuietInteractiveMarker: rviz_default_plugins' InteractiveMarkerDisplay.
  - RvizScenePublisher, VideoCapture: the render window is grabbed through its
  winId(); image_transport publisher on the rviz node; std::filesystem instead
  of boost::filesystem on ROS 2.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] TwistStamped, FacingVisualizer, TargetVisualizer, Pictogram(Array), Diagnostics: build the displays on ROS 2
  rviz_common::MessageFilterDisplay calls processMessage() with ConstSharedPtr on
  ROS 2; the Ogre objects come from rviz_rendering. FacingVisualizer takes the
  camera from the current view controller. Diagnostics subscribes with rclcpp on
  the rviz node.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] OverlayImage, OverlayMenu, OverlayDiagnostic, Plotter2D, LinearGauge, String: build the overlay displays on ROS 2
  The overlays are prepared with prepareOverlays() of rviz_rendering in
  onInitialize(), and the displays subscribe with rclcpp on the rviz node (the
  topic property is initialized with it, its message type is the rosidl name);
  processMessage() takes ConstSharedPtr on ROS 2. OverlayImage subscribes through
  image_transport::create_subscription with the transport of
  ImageTransportHintsProperty. overlay_utils only needs rviz_compat.h for its
  logging.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] ObjectFitOperator, RobotCommandInterface, SelectPointCloudPublish, TabletController: build the panels on ROS 2
  As the other panels, the rviz node is taken in onInitialize() on ROS 2, where
  the part of the constructor needing it continues.
  - ObjectFitOperator: the icon parameters are declared on the rviz node with
  their defaults from the package share directory.
  - RobotCommandInterface: robot_command_buttons is a list of ids with
  robot_command.<id>.{name,icon,type,command,srv}, as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_);
  the services are waited for (1 s) and called on the rviz node.
  - SelectPointCloudPublish: the selection manager comes from the display
  context; rclcpp publisher.
  - TabletController: rclcpp publishers/subscription, std::mutex.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] PublishTopic, EmptyServiceCallInterface, CancelAction, RecordAction: build the panels on ROS 2
  The panels get the rviz node in onInitialize() on ROS 2 (rviz_common has no
  node before that), and create their publishers / service clients there.
  - PublishTopic: rclcpp publisher; nothing is published while no topic is set.
  - EmptyServiceCallInterface: the buttons are built in onInitialize(); ROS 2
  parameters can not hold an array of structs, so rviz_service_call.buttons is
  a list of ids with rviz_service_call.<id>.{icon,service_name,text}, as on the
  ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); the services are called asynchronously, as the executor of
  rviz already spins its node.
  - CancelAction: the actions are found by their _action/cancel_goal services and
  cancelled through action_msgs/srv/CancelGoal.
  - RecordAction: rclcpp publisher of /record_command.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] CloseAll, OpenAll, OverlayPicker, ScreenshotListener: build the tools on ROS 2
  The headers and the pluginlib header differ (rviz_common, class_list_macros.hpp);
  the rest is shared through the rviz:: aliases of rviz_compat.h.
  - ScreenshotListener: the screenshot service is created on the rviz node and
  the render window is grabbed through QScreen::grabWindow() of its winId().
  - OverlayPicker: OverlayText and PieChart are not built on ROS 2, so String and
  LinearGauge are pickable instead, as on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_); the target
  property starts as NULL on ROS 2.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_rviz_plugins] build on ROS 1 (catkin) and ROS 2 (ament) from one package.xml/CMakeLists.txt
  package.xml: the dependencies only ROS 1 has (catkin, rviz, dynamic_reconfigure,
  message_generation, jsk\_\*_msgs, people_msgs, view_controller_msgs, ...) and the
  test dependencies of the rostests get condition="$ROS_VERSION == 1"; the ament
  build tools, rviz_common, rviz_rendering, rviz_default_plugins and the other
  dependencies of the ROS 2 build get condition="$ROS_VERSION == 2"; build_type
  follows ROS_VERSION.
  CMakeLists.txt: the ROS 1 part is unchanged; the ROS 2 (ament) build is
  prepended in `if("$ENV{ROS_VERSION}" EQUAL 2) ... else()`, and the file ends with
  `endif()`. It generates the messages with rosidl, builds the plugins of
  rviz_common_plugins.xml from the same src/ as ROS 1 and exports
  rviz_common_plugins.xml (the ROS 2 plugin description, from the ros2 branch,
  `#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_) to pluginlib; plugin_description.xml stays the ROS 1 one. Which plugins
  are not built on ROS 2 and why is listed there, as decided on the ros2 branch:
  OverlayText/PieChart (rviz_2d_overlay_plugins), CameraInfo/PoseArray
  (rviz_default_plugins), those waiting for jsk_hark_msgs, jsk_footstep_msgs,
  jsk_recognition_msgs, people_msgs, view_controller_msgs or jsk_gui_msgs, and
  OverlayCamera (rviz_rendering hides the Ogre render target).
  msg/srv: `std_msgs/Header` and a package-relative TransformableMarkerOperate,
  which rosidl requires and catkin accepts as well.
  src/rviz_compat.h: the sources tell ROS 1 and ROS 2 apart with
  `#if ROS_VERSION_MAJOR != 1` as jsk_common does
  (jsk_ros1_ros2_compat/ros_version.h, which the ROS 1 build of this package does
  not depend on, so the detection is spelled out). On ROS 2 it reopens the rviz::
  names used here as aliases of their rviz_common / rviz_common::properties /
  rviz_rendering / rviz_default_plugins counterparts, as
  JSK_ROS1_ROS2_COMPAT_MSG_ALIAS() does for the messages, so that the pure renames
  need no #if, and provides ROS_DEBUG..ROS_FATAL(_STREAM), ROS_VERSION_MINIMUM and
  ros::WallTime/WallDuration for the shared code.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request `#899 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/899>`_ from jsk-ros-pkg/obese-devel
  Remove gdown key
* gdown is not used anymore
* Contributors: Kei Okada, Yoshiki Obinata

2.1.10 (2024-12-11)
-------------------

2.1.9 (2024-12-11)
------------------
* enable to jsk_rviz_plugins, without rviz/ogre (`#848 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/848>`_)
  * jsk_rviz_plugins: find_package(Qt.. only when USE_VISUALIZATION
  * enable to compile without jsk_recognition_utils, if 'pcl/visualization' not found
  * enable to jsk_rviz_plugins, without rviz/ogre, i.e. use jsk_rviz_plugins message within embedded environment, compile with 'catkin bt -vi --cmake-args -DUSE_VISUALIZATION=ON'

* add more visualization options for BoundingBox and BoundingBoxArray `#844 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/844>`_
  * keep index to set box color
  * add value threshold filtering and fix unintended state behaviour for boxes violating size constraints
  * property naming consistency
  * hide min max properties in flat mode
  * add alpha min max value properties
  * add alpha = value option

* [ROS-O] patches (`#891 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/891>`_)

  * support Ogre 1.12
    OgreTechnique and Overlay are only forward declared in some headers now.
    `SimpleRenderable::setMaterial` requires a MaterialPtr from 1.12.
    The std::string overload was deprecated in 1.10.
  * drop ROS_DISTRO STRGREATER tests
    As the Debian packages use "Debian"
    https://salsa.debian.org/science-team/ros-ros-environment/-/blame/master/debian/rules?ref_type=heads#L6
    which is incompatible, does not trigger them
    c++14 is the default from melodic onward, so the additional statements
    there are not necessary

* [jsk_rviz_plugins] fix typo in test: boundigbox -> boundingbox (`#888 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/888>`_)
* [jsk_rviz_plugins][Plotter2D] add option to control text size in plot (`#878 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/878>`_)
* use qt key namespace (`#870 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/870>`_)
* Fix QPainterPath path has incomplete type and cannot be defined (`#869 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/869>`_)
* Fix simultaneous display of OverlayCamera and OverlayImage (`#863 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/863>`_)
* [jsk_rviz_plugins] allocate only valid bboxes (`#861 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/861>`_)
* Fixed linear gague display (`#860 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/860>`_)

* add test to check 'genpy.message.SerializationError: field width must be an integer type' (`#864 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/864>`_)
  * use int_t for width/height/top/left to be compatible with OverlayText.msg
  * add test to check 'genpy.message.SerializationError: field width must be an integer type'
  ```
  File "/home/k-okada/catkin_ws/jsk_recognition_ws/devel/lib/python3/dist-packages/jsk_rviz_plugins/msg/_OverlayText.py", line 127, in serialize
  except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
  File "/opt/ros/noetic/lib/python3/dist-packages/genpy/message.py", line 393, in _check_types
  check_type(n, t, getattr(self, n))
  File "/opt/ros/noetic/lib/python3/dist-packages/genpy/message.py", line 261, in check_type
  raise SerializationError('field %s must be an integer type' % field_name)
  genpy.message.SerializationError: field width must be an integer type
  ```

* Fixed typo of Software License Agreement. and/o2r to and/or (`#853 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/853>`_)
* Fixed typo of Software License Agreement. and/o2r to and/or
* [jsk_rviz_plugins] Fixed Camera Info visualization (`#851 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/851>`_)

  * [jsk_rviz_plugins/CameraInfo] Fixed scale depending on encodings
  * [jsk_rviz_plugins/CameraInfo] Fixed duplicated BGR encogind
  * [jsk_rviz_plugins/CameraInfo] Enable ROI size iamge input
  * [jsk_rviz_plugins/CameraInfo] Supprt for other image encodings (`#1 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/1>`_)
  * [jsk_rviz_plugins/CameraInfo] Avoid invalid image size case for visualization image
  * [jsk_rviz_plugins] Fixed Camera Info visualization

* integrate all config file to one, to make it easy to restart failed job (`#850 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/850>`_)

  * location? file? of 2017-06-20-12-00-00_people_images_in_lab.bag.tgz has changed ? but md5sum is same... strange

* Contributors: Aoi Nakane, Kei Okada, Martin Pecka, Shingo Kitagawa, Yoshiki Obinata, iory, joachim-p, ozzdemir, v4hn

2.1.8 (2022-01-11)
------------------
* Fix font name for pictgram (`#835 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/835>`_)

  * add launch/pictogram_sample.launch config/pictogram_sample.rviz
  * Add FontAwesome5: `#759 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/759>`_ need to update Font Name too
    PR `#579 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/579>`_ reported fa-mendeley is not displayed correctly. This because of the plugin uses system font, which is installed by fonts-font-awesome deb package, not font defined in fontawesome.dat. So if you remove fonts-font-awesome, you can not display most of font correctly. This PR update getFont function to return the correct font family

* Change pie chart color at your custom threshold (`#840 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/840>`_)
* [jsk_rviz_plugins] added new Linear Gauge rviz plugin (`#831 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/831>`_)
* add dependency.yml to check resolve circular dependency in jsk_rviz_plugins/package.xml (`#830 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/830>`_)

* add test code for jsk_rviz_plugins (`#825 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/825>`_)

  * fix `#799 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/799>`_
  * move normal.test and face_detecotr.test to xml because of missing bag files
  * skip if it failed to download bag file, due to 'could not download file. maximum download trial exceeded.'
  * add more info on rviz_config_check.py
    - fix bugs when waiting rviz service takes more than 5 sec
    - rviz_config_check.py: check subscribing topic of rviz while rosnode ping rviz
  * increase time-limit to 30
  * skip install test on indigo
  * jsk_rviz_plugins : enable add_rostest for all DISTRO
  * normal.test requires jsk_pcl/NormalConcatenater whcih needs jsk_pcl_ros on kinetic
  * on <meloidc, rviz/SendFilePath is not found, use std_srvs/Empty for rviz/reload_shaders
  * install gdown / jsk_pcl_ros_utils for install_sample_data
  * install_sample_data requries jsk_data
  * enable normal.test, face_detector.test
  * fix normal_sample.launch to suport launch_openni arg and use async
  * fix face_detector_sample.launch to work with 2017-06-20-12-00-00_people_images_in_lab.bag
  * fix typo t_start -> self.t_start
  * add install_sample_data.py to download 2017-06-20-12-00-00_people_images_in_lab.bag for test
  * add joint_state_publisher and robot_state_publisher to test_depend
  * add robot_command_interface_sample.rviz and service_call_panel.rviz
  * [test/segmen_array.test, config/segment_array_sample.rviz] fix wrong Topic /polygon_array_sample/output -> /segment_array_sample/output
  * jsk_rviz_plugins: add test codes
  * link_marker_publisher_sample.launch, contact_state_marker_sample.launch requires pr2_description
  * add rostest to package.xml, run test code from catkin
  * rviz_config_check.py : node to check if rviz arives

* [jsk_rviz_plugins] move rviz config from cfg/ to config/ and fix typo (`#823 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/823>`_)
* [jsk_rviz_plugins] add piechart sample (`#824 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/824>`_)
* Resize and repositioning of PieChart wasn't working, corrected. (`#822 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/822>`_)
* fix with '2to3 -w -f zip .' (`#817 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/817>`_)
* Use multiple billboard lines when exceeding max elements per line (`#808 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/808>`_)
* Updating the URLs of the jsk_rviz_plugins and jsk_rqt_plugins so the generated README points to working links (closes #805 ). (`#806 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/806>`_)
* added a regex for "color: XXX;" pattern to have a properly colored shadow (`#802 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/802>`_)

  * fix
    [jsk_rviz_plugins:make] /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:274:81: error: no matching function for call to ‘regex_replace(std::string&, std::regex&, const char [1])’
    [jsk_rviz_plugins:make]          std::string formatted_text\_ = std::regex_replace(text\_, color_tag_re, );
    [jsk_rviz_plugins:make]                                                                                  ^
    [jsk_rviz_plugins:make] /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:274:81: note: candidates are:
    [jsk_rviz_plugins:make] In file included from /usr/include/c++/4.8/regex:62:0,
    [jsk_rviz_plugins:make]                  from /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:48:
    [jsk_rviz_plugins:make] /usr/include/c++/4.8/bits/regex.h:2162:5: note: template<class _Out_iter, class _Bi_iter, class _Rx_traits, class _Ch_type> _Out_iter std::regex_replace(_Out_iter, _Bi_iter, _Bi_iter, const std::basic_regex<_Ch_type, _Rx_traits>&, const std::basic_string<
    _Ch_type>&, std::regex_constants::match_flag_type)
    error, for compile on ubuntu 12.04

* [jsk_rviz_plugins] add warn for deprecated jsk_rviz_plugins/PoseArrayDisplay (`#786 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/786>`_)

* Contributors: Adi Vardi, Andres Kushnir, Francois Teyssere, Francois Teyssere, Jan Krieglstein, Kei Okada, Sam Pfeiffer, Shingo Kitagawa, Shumpei Wakabayashi

2.1.7 (2020-10-17)
------------------
* Fix programming issues where functions were not getting return values, and variables were not being declared for types (`#783 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/783>`_)
* Support custom color for OverlayMenu (`#775 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/775>`_)
* add human skeleton rviz visualization(`#740 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/740>`_)

  * jsk_recognition_msgs < 1.2.15 does not support human_skeleton_array_display.cpp
  * meloidc needs to include OGRE/OgreSceneManager.h
  * human_skeleton_array_display supports indigo build
  * add sphere at all edge ends
  * fix typo: skelton -> skeleton
  * add human skelton rviz visualization

* Add fg_color/bg_color to OverlayMenu.msg (`#776 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/776>`_)
* [jsk_rviz_plugins] Add StringDisplay as a new display plugin (`#728 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/728>`_)
* set property for ccache if cmake version < 3.4 (`#780 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/780>`_)
* [jsk_rviz_plugin/PieChart] add clock wise rotate option for pie chart (`#782 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/782>`_)
* Remove meaningless lock (`#750 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/750>`_)
* call processNormal when polygon points has more than 3 point (`#771 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/771>`_)
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * fix typo CV_VERSION_MAJOR -> CV_MAJOR_VERSION
  * run 2to3 -f except
  * run 2to3 -f print
  * support noetic, use c++14, convert to package format 3
  * Merge remote-tracking branch 'ruvu/fix/noetic' into add_noetic

* Add Rviz scene publisher (`#773 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/773>`_)
* Improve Overlay Visibility (`#769 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/769>`_)
* Contributors: Kei Okada, Patrick Beeson, Ramon Wijnands, Ryohei Ueda, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi, Iory Yanokura, Taichi Hagashide

2.1.6 (2020-04-13)
------------------
* [jsk_rviz_plugins] Add FontAwesome 5 (`#759 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/759>`_)

  * Display FontAwesome 5 icons from pictogram.py and pictogram_all.py
  * Add FontAwesome 5
  * Add property to set position of overlay menu

* Add option to specify width and height in VideoCapture plugin (`#748 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/748>`_)
* Add a script that convert String to OverlayText (`#753 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/753>`_)
* fix build failure of OgreSceneManager this block latest Melodic builds (`#766 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/766>`_)
* Add property to set position of overlay menu (`#758 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/758>`_)
* [jsk_rviz_plugins/OverlayImage] Add property to ignore alpha channel of the image (`#752 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/752>`_)
* [motor_states_temperature_decomposer.py] add queue_size (`#756 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/756>`_)
* support jsk_rviz_plugin to be loaded in indigo (`#739 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/739>`_)
* [jsk_rviz_plugins/OverlayImageDisplay] Use memcpy to copy image data (`#737 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/737>`_)

  * Use memcpy to copy image data from cv::Mat to QImagee instead of use QImage::setPixel() many times for optimization.

* Transport hint for camera info (`#736 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/736>`_)

  * Add field to select transport hint of CameraInfo display
    * Use ImageTransport to create subscriber to subscribe image topic in CameraInfoDisplay.
    * Use ImageTransportHintsProperty to choose image transport hints when subscribing image topic to visualize sensor_msgs/CameraInfo.
  * Use ImageTransportHintsProperty in OverlayImageDisplay class
  * Add ImageTransportHintsProperty class
    * ImageTransportHintsProperty is an rviz property class specialized for image_transport::TransportHints.

* Add transport hint field to OverlayImage display (`#730 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/730>`_)

  * Add transport hint field to OverlayImage display
    * Add an editable enum field to specify transport hint on OverlayImage display.
    * raw, compressed and theora are listed as pre-defined transport  hints.

* Unsubscribe image topic when "use image" is unchecked in CameraInfo display (`#732 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/732>`_)
* Fix format specifier (`#731 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/731>`_)

  * Use %u instead of %lu to print Ogre::Texture::getWidth() and Ogre::Texture::getHeight() because they return uint32.

* Do not subscribe image topic when rviz startups in OverlayImage display (`#733 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/733>`_)
  * Do not subscribe image topic when rviz startups in OverlayImage display
    * In order not to subscribe image topic when rviz startups with OverlayImage display disabled, always verify if the display is enabled before the display subscribes topic.
  * Unsubscribe image topic when "use image" is unchecked in CameraInfo display

* Support classification result visualization with approximate sync (`#725 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/725>`_)

  * classification_result_visualizer: add option to use approximate synchronizer

* Contributors: Yuki Furuta, Iki Yo, Naoki Mizuno, Naoki Hiraoka, Ryohei Ueda, Shingo Kitagawa, Yuto Uchimi, Iory Yanokura

2.1.5 (2019-02-18)
------------------
* [jsk_rviz_plugins] Add "Align Bottom" option to OverlayText (`#723 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/723>`_ )

  * Update config for easily understanding the effect of AlignBottom
  * Update overlay_sample.launch
  * Add rosparam to enable/disable reversing lines
  * Add "Align Bottom" option to overlay_text plugin

* Contributors: Yuto Uchimi

2.1.4 (2018-11-01)
------------------
* [jsk_rviz_plugins/target_visualize] Add visualizer\_ initilized flags (`#720 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/720>`_)
* replace boost::shared_ptr by std::shared_ptr (`#710 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/710>`_)
  * enable C++11
  * replace boost pointers by std pointers

* add error message to status (`#715 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/715>`_)
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Add comment for install destination
  * Fix path to headers for installation
  * Install missing test/ as well
  * Use source permission when installing executables
  * Fix installation destination

* [jsk_rviz_plugins/camera_info_display] Check fx and fy are not equal to zero. (`#1 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/1>`_)
* [jsk_rviz_plugins] Optimize camera info displaying (`#709 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/709>`_)
  * Split and merge image matrix channels instead of slow pixel-by-pixel copying while renderind camera info.

* [jsk_rviz_plugins] add segment_array_display (`#666 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/666>`_)
  * Add doc and sample of segment_array
  * add segment_array_display rviz plugin.

* [jsk_rviz_plugins] use QScreen::grabWindow() instead of QPixmap::grabWindow (`#700 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/700>`_)
  * [jsk_rviz_plugins] use QScreen::grabWindow() instead of QPixmap::grabWindow

* add enable lighitng property in polygon_array_display (`#686 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/686>`_)
* add jsk_rviz_plugins library to catkin_package LIBRARIES, use  instea… (`#696 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/696>`_)
  * add jsk_rviz_plugins library to catkin_package LIBRARIES
* Add #include <boost/format.hpp> (`#695 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/695>`_)

* jsk_rviz_plugins: warn on missing frame_id (`#698 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/698>`_)
* Suppress warnings of jsk_rviz_plugins for non-existent targets (`#693 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/693>`_)
  Support `catkin_make` also.
  Ref: https://github.com/jsk-ros-pkg/jsk_visualization/pull/692#issuecomment-390873758

* [jsk_rviz_plugins] fix std::isnan to make it compile under Ubuntu 16.04 / gcc 5 (`#687 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/687>`_)
  * fix std::isnan to make it compile under Ubuntu 16.04 / gcc 5
  * revert whitespace changes (adding trailing whitespace again)

* add enable lighitng property in polygon_array_display
* jsk_rviz_plugins: class_result_vis: add more types to vislalize (`#684 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/684>`_)
* jsk_rviz_plugins: add missing deps (`#683 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/683>`_)
* Add <url> to package.xml to add link to README (`#681 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/681>`_)
* Contributors: Aleksandr Rozhdestvenskii, Christian Rauch, Daniel Neumann, Yuki Furuta, Jan Carius, Kei Okada, Kentaro Wada, Laurenz, Masaki Murooka, Tamaki Nishino, Yuto Uchimi, Iori Yanokura

2.1.3 (2017-10-26)
------------------
* [jsk_rviz_plugins] Rviz default font is changed from Arial to LiberationSans (See: https://github.com/ros-visualization/rviz/pull/1141) (`#676 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/676>`_)
* Add exclude regex in rosconsole_overlay (`#675 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/675>`_)
* Contributors: Iori Kumagai, Kentaro Wada

2.1.2 (2017-07-07)
------------------
* [jsk_rviz_plugins][classification_result_visualizer] minor bugfix (`#669 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/669>`_ )
* [jsk_rviz_plugins] add marker publisher for classification result (`#667 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/667>`_)
  * [jsk_rviz_plugins] add visualizer for classification result

* Contributors: Yuki Furuta

2.1.1 (2017-02-15)
------------------
* remove depends to wxwidgets https://github.com/ros/rosdistro/pull/13886#issuecomment-279832181
* Contributors: Kei Okada

2.1.0 (2017-02-13)
------------------
* migration to kinetic, which uses qt5 wehre as indig/jade uses qt4 (`#662 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/662>`_ )
* Feature to transform markers in rviz (`#661 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/661>`_ )
  * Not to build transformable_marker_operator in jsk_rviz_plugins
  * Move TransformableMarkerOperatorAction to jsk_interactive_marker
  * Add feature to transform marker to rviz plugin
  * Add server_name for TransformableMarkerOperatorAction
* Contributors: Kentaro Wada, Hiroto Mizohana

2.0.1 (2016-12-15)
------------------

2.0.0 (2016-12-14)
------------------
* Stop using deprecated jsk_topic_tools/log_utils.h
  see
  - https://github.com/jsk-ros-pkg/jsk_common/pull/1462
  - https://github.com/jsk-ros-pkg/jsk_common/issues/1461
* [jsk_rviz_plugins/src/empty_service_call_interface.cpp] remove unused variables.
* Contributors: Kentaro Wada, MasakiMurooka

1.0.34 (2016-09-29)
-------------------
* Fix for Ogre >= 1.9, which build fail on Jade on 14.10/15.04
* [jsk_rviz_plugins] add offset to footstep_display.h
* [jsk_rviz_plugin] Add rviz button interface for yes/no service request
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi

1.0.33 (2016-09-13)
-------------------
* [doc/jsk_rviz_plugins/plugins/pie_chart.md, plotter_2d.md] add doc to how to change caption of overray text (Fix https://github.com/jsk-ros-pkg/jsk_visualization/issues/634)
* [jsk_rviz_plugins/CMakeLists.txt] Install samples dir that was missing for jsk_rviz_plugins (https://github.com/jsk-ros-pkg/jsk_visualization/issues/632)
* [jsk_rviz_plugins/samples/overlay_sample.py] Add queue_size arg for deprecated warning in overlay_sample.py (https://github.com/jsk-ros-pkg/jsk_visualization/issues/631)
* [jsk_rviz_plugins/src/overlay_text_display.cpp] Show available fonts using enum property (https://github.com/jsk-ros-pkg/jsk_visualization/issues/630)
* [jsk_rviz_plugins/src/overlay_picker_tool.cpp] handleDisplayClick was not going past first group  as after processing a group with no overlay item, it was still  returning true by default. It needed to return false to continue the
  seach (https://github.com/jsk-ros-pkg/jsk_visualization/issues/627)
* New rviz plugin to visualize jsk_recognition_msgs::BoundingBox (https://github.com/jsk-ros-pkg/jsk_visualization/issues/616)

  * [jsk_rviz_plugins/src/bounding_box_array_display.cpp] Show valid boxes even if invalid box is included

* Contributors: Jit Ray Chowdhury, Kei Okada, Kentaro Wada

1.0.32 (2016-07-20)
-------------------
* Show colorized ros logging on rviz overlay text
* Fix style of code of rosconsole_overlay_text.py
* Convert RGB to BGR precisely in video capturing
* Support multi legs in footstep_display
* Use small sized icons for faster adding display properties
  Fix https://github.com/jsk-ros-pkg/jsk_visualization/issues/603
* Cleanup jsk_rviz_plugins package.xml
* Fix moc generation errors with boost >= 1.57 (for OS X currently)
  Please refer to https://github.com/ros-visualization/rviz/pull/826
* Keep aspect ratio with only specified width for OverlayImage
* Contributors: Kentaro Wada, Eisoku Kuroiwa

1.0.31 (2016-05-19)
-------------------
* Stop passing -z flag to ld with Clang
* Contributors: Kentaro Wada

1.0.30 (2016-03-25)
-------------------
* use jsk_rviz_plugins::StringStamped instead of roseus::StringStamped, to remove roseus depends
* add rviz_DEFAULT_PLUGIN_LIBRARIES:  see https://github.com/ros-visualization/rviz/pull/979
* Contributors: Kei Okada

1.0.29 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_rviz_plugins] Do not show unnecessary properties of CameraInfo
* [jsk_rviz_plugins] Delete property in OverlayDiagnosticDisplay
* [jsk_rviz_plugins/OverlayDiagnostics] Add new style
* [jsk_rviz_plugins/OverlayPicker] Align to grid in pressing shift key
* Contributors: Kei Okada, Ryohei Ueda

1.0.28 (2016-02-03)
-------------------
* [jsk_rviz_plugins] Fix OverlayPicker for plugins gdouped by DisplayGroup.
* [jsk_rviz_plugins] Add keep aspect ratio option to OverlayImageDisplay.
  And ass overlay image sample to overlay_sample.launch
* [jsk_rviz_plugins] Add new tool OverlayPicker to move overlay plugin
  by mouse dragging
* [jsk_rviz_plugins] Add script to visualize static OverlayText
* [jsk_rviz_plugins] Support multiple Float32 in float32_to_overlay_text.py
* [jsk_rviz_plugins] Utility script to draw float32 as overlay text
  Added:
  - jsk_rviz_plugins/scripts/float32_to_overlay_text.py
* [jsk_rviz_plugins] Add utility python class to publish OverlayText
  Modified:
  - jsk_rviz_plugins/CMakeLists.txt
  Added:
  - jsk_rviz_plugins/cfg/OverlayTextInterface.cfg
  - jsk_rviz_plugins/python/jsk_rviz_plugins/__init_\_.py
  - jsk_rviz_plugins/python/jsk_rviz_plugins/overlay_text_interface.py
  - jsk_rviz_plugins/setup.py
* [jsk_rviz_plugins] Add regular expression interface to specify
  target node in rosconsole_overlay_text.py
  Modified:
  - jsk_rviz_plugins/scripts/rosconsole_overlay_text.py
* [jsk_rviz_plugins] Fix allocateShapes API of TorusArrayDisplay
  Modified:
  - jsk_rviz_plugins/src/torus_array_display.cpp
  - jsk_rviz_plugins/src/torus_array_display.h
* [jsk_rviz_plugins] Add script to visualize /rosout on rviz
  Added:
  - jsk_rviz_plugins/scripts/rosconsole_overlay_text.py
* Merge remote-tracking branch 'refs/remotes/garaemon/clear-torus-arrow' into many-prs
* [jsk_rviz_plugins] Support css to change text color and so on in OverlayText display.
* [jsk_rviz_plugins] A script to take screenshot of rviz when a topic is
  published: relay_screenshot.py
  It depends on ScreenshotListener tool of jsk_rviz_plugins.
  For example, `rosrun jsk_rviz_plugins relay_screenshot.py ~input:=/region_growing_multi_plane_segmentation/output/polygons`
* [jsk_rviz_plugins] Clear arrow of torus when it is disabled
  Modified:
  jsk_rviz_plugins/src/torus_array_display.cpp
* [jsk_rviz_plugins] Disable `show coords` in default in BoundingBoxDisplay
  Modified:
  jsk_rviz_plugins/src/bounding_box_array_display.cpp
* [jsk_rviz_plugins] Do not visualize failure=true toruses.
  It requires https://github.com/jsk-ros-pkg/jsk_recognition/pull/1379
* Contributors: Ryohei Ueda

1.0.27 (2015-12-08)
-------------------
* [jsk_rviz_plugins/BoundingBoxArray] Fix coords orientation.
  closes `#528 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/528>`_
* Use ccache to make it faster to generate object file
* [jsk_rviz_plugins] Empty function implementation for undefined methods
* [jsk_rviz_plugins] Use set_target_properties to set linker flags only
  for libjsk_rviz_plugins.so
* Use gcc -z defs to check undefined symbols in shared objects
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.26 (2015-12-03)
-------------------
* [jsk_rviz_plugins] Install icons
* [jsk_rviz_plugins] add landing_time_detector to display early landing/taking-off
* [jsk_rviz_plugins/motor_states_temparature_decomposer] Decrease cpu load
  by queue_size=1.
  Fix for joints which does not have limit attribute.
* [jsk_rviz_plugins] Add ~parent_link parameter for contact_state_publisher
* [jsk_rviz_plugins] Add dynamic_reconfigure API to ContactStateMarker
* [jsk_rviz_plugins] Check size of likelihood and labels of PolygonArray
* [jsk_rviz_plugins/contact_state_marker.py] Support origin attribute of
  visual tag
* [jsk_rviz_plugins] update ambient sound visual paramter
* [jsk_rviz_plugins] contact_state_marker.py to visualize hrpsys_ros_bridge/ContactStatesStamped
* [jsk_rviz_plugins] Add script to publish marker of a robot link with
  specified color
* Contributors: Eisoku Kuroiwa, Kentaro Wada, Ryohei Ueda, Yuto Inagaki

1.0.25 (2015-10-10)
-------------------
* [jsk_rviz_plugins] Fix font size of PeoplePositionMeasurementArray
* [jsk_rviz_plugins] Add script for diagnostics sample
* [jsk_rviz_plugins] Compile PeoplePositionMeasurementArrayDisplay
* [jsk_rviz_plugins/VideoCapture] Check file permission to write correctly
* [jsk_rviz_plugins] Use readthedocs to document
* [jsk_rviz_plugins] Add index page for sphinx + readthedocs
* [jsk_rviz_plugins] Use jsk_recognition_utils instead of jsk_pcl_ros to
  speed up compilation
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.24 (2015-09-08)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Fix compilation error because of
  the latest jsk_recongition_utils changes
* [jsk_rqt_plugins/TwistStamped] Fix duplicated delete
* [jsk_rviz_plugins] Allow width/height 0 image (fix segfault)
* [jsk_rviz_plugins/PolygonArray] Coloring by labels and likelihood fields
  of jsk_recognition_msgs/PolygonArray
* [jsk_rviz_plugins/TwistStamped] Decide circle thickness according to
  radius of circle
* [jsk_rviz_plugins/BoundingBoxArray] Normalize value color gradation
* [jsk_rviz_plugins/BoundingBoxArray] Update coloring method to support
  coloring by values and labels.
* [jsk_rviz_plugins] Remove footstep texts from rviz when reset the plugin
* [jsk_rqt_plugins] Add sample launch for PolygonArray
* [jsk_rviz_plugins/PolygonArray] Use enum property to choose coloring method
* [jsk_rviz_plugins/TfTrajectory] Use status property to show error rather than
  ROS_ERROR
* [jsk_rviz_plugins/RobotCommandInterface] Use smaller icon size
* [jsk_rviz_plugins] Use ~robot_command_buttons parameter to configure RobotCommandInterfaceAction
* [jsk_rviz_plugins/TFTrajectory] Initialize line width
* [jsk_rviz_plugins/TFTrajectory] Add movie link to README
* [jsk_rviz_plugins] A rviz plugin to visualize tf trajectory as path
* [jsk_rviz_plugins][OverlayImage] Automatically setup size with negative val
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------
* [jsk_rviz_plugins/PoseArray] Clear pose array if checkbox is unchecked
* fix coords bug
* Contributors: Ryohei Ueda, Yu Ohara

1.0.22 (2015-06-24)
-------------------
* [jsk_rviz_plugins/OverlayImage] Support alpha channel if image_encoding
  is BGRA8 or RGBA8
* Contributors: Ryohei Ueda

1.0.21 (2015-06-11)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Cleanup codes to be within 80 columns
* [jsk_rviz_plugins/BoundingBoxArray] Immediately apply change of attributes
* [jsk_rviz_plugins/BoundingBoxArray] Refactor codes by splitting processMessages into several functions
* [jsk_rviz_plugins/BoundingBoxArray] Use symmetrical radius for coordinates arrow
* [jsk_rviz_plugins/BoundingBoxArray] Fix coding style around if/else/for
* [jsk_rviz_plugins/BoundingBoxArray] Check if the size of box is nan
* [jsk_rviz_plugins/BoundingBoxArray] Fix indent to be within 80 columns
* Contributors: Ryohei Ueda

1.0.20 (2015-05-04)
-------------------
* [jsk_rviz_plugins] add rotate speed to pictogram
* [jsk_rviz_plugins] add String PopupMode for Pictogram
* [jsk_rviz_plugins] Make arrow nodes invisible as default in PolygonArrayDisplay not to show normal if no needed
* [jsk_rviz_plugins] Check size of BoundingBox
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.19 (2015-04-09)
-------------------
* [jsk_rviz_plugins] Fix initialization order in Plotter2DDisplay in order  to avoid call std::vector::resize with uninitialized length
* [jsk_rviz_plugins] Obsolate SparseOccupancyGridArray, it's replaced by SimpleOccupancyGridArray
* [jsk_rviz_plugins] Use jsk_pcl_ros/geo_util to reconstruct 3d
  information in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add image of SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support auto coloring in SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support 4th parameter of plane coefficients in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] add tmp pose array display
* [jsk_rviz_plugins] Change plotter color from 30%
* add_mesh_model_in_transformable_marker
* [jsk_rviz_plugins] Do not update min/max value when re-enabling Plotter2D
* [jsk_rviz_plugins] Change color of plotter from 50 percent of max value
* [jsk_rviz_plugins] add showing coords option for bounding box array display
* [jsk_rviz_plugins] Add utility script to visualize difference between to tf frame on rviz
* [jsk_rviz_plugins] Check direction vector is non-nan in PolygonArrayDisplay
* [jsk_pcl_ros] Fix license: WillowGarage -> JSK Lab
* [jsk_pcl_ros] Fix install path and install headers
* [jsk_rviz_plugins] Do not show disabled properties of OverlayText, Plotter2D and PieChart
* [jsk_pcl_ros] Make overlay sample more faster
* [jsk_rviz_plugins] Change color from 60 percent of maximum value in PieChartDisplay and Plotter2DDisplay
* [jsk_rviz_plugins] Draw PieChart at the first time
* Remove rosbuild files
* [jsk_rviz_plugins] Update PieChartDisplay only if value changed
* [jsk_rviz_plugins] Do not change texture size and position in processMessage
* [jsk_rviz_plugins] Optimize PieChartDisplay, draw image in update() method instead of processMessage
* Contributors: Ryohei Ueda, Yu Ohara, Yuto Inagaki

1.0.18 (2015-01-30)
-------------------
* add depends to cv_bridge instaed of opencv2

1.0.17 (2015-01-29)
-------------------
* [jsk_rviz_plugins] Add TwistStampedDisplay
* [jsk_rviz_plugins] Use jsk_recognition_msgs
* update README file for mainly panels
* [jsk_rviz_plugins] Add document of PolygonArray display
* add publishing pointcloud information as overlay text
* add record action panel
* remove unused QLineEdit variable
* add normal option for torus display
* [jsk_rviz_plugins] Refactor PolygonArrayDisplay class
* [jsk_rviz_plugins] Add "Show Normal" to PolygonArrayDisplay
* add object fit operator panel
* Make torus more smooth and add beatiful parameter
* add torus array display
* Contributors: Ryohei Ueda, JSK Lab member, Yuto Inagaki

1.0.16 (2015-01-04)
-------------------
* [jsk_rviz_plugins] Fix namespace of TabletViewController
* [jsk_rviz_plugins] Fix namespace jsk_rviz_plugin -> jsk_rviz_plugins
* [jsk_rviz_plugins] Utility script to draw the number of samples during
  capturing data
* [jsk_rviz_plugins] Remove invalid codes of ScreenshotListenerTool
* [jsk_rviz_plugins] VideoCaptureDisplay Display to capture rviz as movie
* [jsk_rviz_plugins] ScreenshotListenerTool: A simple tool to listen to
  a service and save screenshot to specified file
* [jsk_rviz_plugins] Avoid Segmentation Fault when size 0 texture is
  specified

1.0.15 (2014-12-13)
-------------------
* Add new plugin and message to display array of pictograms
* Remove pictogram when the display is disabled
* Fix policy to move head using rviz: Do not consider movement of mouse,
  just use the position of the mouse. Because we cannot ignore
  network latency
* Fix several parameters suitable for surface
* Add panel for tablet demonstration
* Add view_controller_msgs
* Compute difference to mouse position
* Add TabletViewController to control robot from tablet using rviz
* Check texture is available or not when initializing CameraInfo
* Paster image on the bottom of the camera parameter pyramid
* Contributors: Ryohei Ueda

1.0.14 (2014-12-09)
-------------------
* Add more action to pictogram
* Add documentation about pictogram
* Do not rewrite texture if no need
* Add sample to visualize all the pictograms
* Add FontAwesome fonts and several improvements about font drawing:
  1) decide size of font according to font metrics
  2) do not re-write pictogram texture if no need
* Support deletion of pictogram
* Add color field to Pictogram.msg
* Add sample script for pictogram
* Add display to visualize pictogram
* fixed parameter namespace mismatch.
* set the components to align left
* added button for start_impedance_for_drill
* added service to check marker existence. added copy to marker operation.
* fix quatation signiture for function name in robot_command_interface.cpp
* refact and delete some unneeded includes
* add empty_service_call_interface
* add robot_command_interface
* Change the size of menu according to the change of title and fix
  position of the popup window if the window is larger than the rviz
* Use name for decomposed topic of motor_states_temperature_decomposer.py
* Change color of text according to the foreground color of PieChart
* Show value as string on Plotter2DDisplay
* Decompose joint_state's effort value and read the max value from robot_description
* Fix motor_state_decomposer.py
* Take title into account to decide the size of OverlayMenu
* compacting the panel with using tab
* move msg to jsk_rviz_plugins
* add depend on jsk_interactive_marker
* add transformable marker operator panel
* Coloring footstep by jsk_footstep_msgs::Footstep::footstep_group
* Show text on footstep to display left or right
* Separate 'OvertakeProperties' into 'Overtake Color Properties' and
  'Overtake Position Properties'
* Script to decompose MotorStates/temperature into std_msgs/Float32
* Contributors: Ryohei Ueda, Masaki Murooka, Yuto Inagaki

1.0.13 (2014-10-10)
-------------------
* Add "overtake properties" property to OverlayTextDisplay
* Call queueRender after opening/closing properties in Open/CloseAllTool
* Contributors: Ryohei Ueda

1.0.12 (2014-09-23)
-------------------

1.0.11 (2014-09-22)
-------------------
* Do not ues deprecated PLUGINLIB_DECLARE_CLASS
* Draw polygon as 'face' on PolygonArrayDisplay
* Use jsk_topic_tools::colorCategory20 to colorize automatically
* Add tool plugin to close/open all the displays on rviz
* Contributors: Ryohei Ueda

1.0.10 (2014-09-13)
-------------------
* Fix color of people visualizer by initializing color to sky blue
* Fix texture color of camera info by filling color value of texture image
* Fix caching of overlay textures of OverlayMenuDisplay to support
  changing menus
* add relay camera info node
* Add new plugin to visualize sensor_msgs/CameraInfo
* Ignore first message means CLOSE in OverlayMenuDisplay
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.9 (2014-09-07)
------------------

1.0.8 (2014-09-04)
------------------
* add enum menu to TargetVisualizer and PeoplePositionMeasurementDisplay
  to select the style of the visualizer
* do not depends on people_msgs on groovy
* add SimpeCircleFacingVisualizer class
* separate a code to draw visualizer into facing_visualizer.cpp
* add rviz plugin for face_detector
* cleanup package.xml of jsk_rviz_plugins
* Contributors: Ryohei Ueda

1.0.7 (2014-08-06)
------------------
* show "stalled" if no diagnostic message received in OverlayDiagnosticDisplay
* add utility class for Overlay: OverlayObject and ScopedPixelBuffer in overlay_utils.cpp
* spcify max/min values for the properties of Plotter2D
* fix color error when changing the size of the window of Plotter2D
* add offset to compute the absolute position of the grid
* Remove non-used color property in OverlayDiagnosticsDisplay
* Remove OverlayDiagnostic correctly (not remaining overlay texture).
* under line of the caption should be longer than the length of the
  caption in TargetVisualizer
* align the position of the text of TargetVisualizer to left
* add CancelAction and PublishTopic plugin to hydro of jsk_rviz_plugin
* add visualizer to visualize pose stamped with target mark
* Contributors: Ryohei Ueda

1.0.6 (2014-07-14)
------------------
* add new plugin to visualize diagnostic status on ovrelay layer
* hide movable text of DiagnosticDisplay at first
* support font size field in DiagnosticDisplay
* diagnostics namespace and frame_id fields of DiagnosticsDisplay is now
  selectable according to the current ROS topics
* support axis color to colorize SparseOccupancyGridMap
* use rviz::PointCloud to render jsk_pcl_ros::SparseOccupancyGridArray to optimize
* hotfix to fix the position of overlay text
* does not update scale if the dimension is same to the previous data in OccupancyGridDisplay
* implement rviz plugin to visualize jsk_pcl_ros::SparseOccupancyGridArray
* add QuietInteractiveMarker
* Contributors: Ryohei Ueda

1.0.5 (2014-06-29)
------------------
* add overlay camera display
* close overlay menu firmly
* add new rviz plugin: OverlayImage
  visualize sensor_msgs::Image as HUD on rviz 3D rendering window
* add new plugin: OverlayMenu
* Contributors: Ryohei Ueda

1.0.4 (2014-05-31)
------------------
* jsk_rviz_plugins: use depend tag add mk/rosbuild to build_depend
* update the initial parameter of FootstepDisplay
* add line width property to BoundingBoxArrayDisplay
* add new plugin: BoundingBoxArray for jsk_pcl_ros/BoundingBoxArray
* Contributors: Ryohei Ueda, Kei Okada

1.0.3 (2014-05-22)
------------------
* add normals param and change skip_rate to set Percentage

1.0.2 (2014-05-21)
------------------
* Fixes a moc generation error with boost >= 1.48
* add color which will be deviced by curvature

1.0.1 (2014-05-20)
------------------
* add README and images, modify some fails
* Contributors: Yuto Inagaki

1.0.0 (2014-05-17)
------------------
* show border as default. add auto coloring option to show
  clusters efficiently.
* decrease the number of the error messages from NormalDispaly
* Contributors: Ryohei Ueda

0.0.3 (2014-05-15)
------------------
* supress erro message of NormalDisplay
* depends to hark_msgs is no longer needed
* Contributors: Ryohei Ueda, Kei Okada

0.0.2 (2014-05-15)
------------------
* overlay sample for groovy
* make NormalDisplay work on catkin.
  add normal_visual.cpp to jsk_rviz_plugins.so
* fix for using ambient_sound
* rename the name of plugin from PolygonArrayDisplay to PolygonArray
* add rviz_plugins icons
* change the color of the pie chart according to the absolute value
* smaller size for the font and add new line to the text of diagnostics display
* add a bool property to toggle auto scale for Plotter2DDisplay
* Merge remote-tracking branch 'refs/remotes/origin/master' into add-auto-color-changing-feature-to-plotters
  Conflicts:
  jsk_rviz_plugins/src/plotter_2d_display.cpp
  jsk_rviz_plugins/src/plotter_2d_display.h
* add auto color change boolean property and max color to change
  the color according to the value
* add sample for overlay rviz plugins
* support DELETE action to disable OvelrayText
* use qt to draw OverlayText
* does not call setSceneBlending twice
* add caption to 2d plotter
* add margin to plotter
* does not create QPainter without argument to supress the warning message of "painter not activate"
* initialize `orbit_theta_` and check overflow of the value
* add `update_interval_` to control the time to update the chart
* do not delete movable text in when the widget is disabled, delete it in deconstructor
* does not plot a chart if rviz is invoked with the plotter plugin disabled
* add DiagnosticsDisplay
* call hide in the destructor of overlay widgets
* add text to show caption and value.
  in order to toggle caption, added new check box.
  as caption, use the widget name.
* implement piechart on rviz using overlay technique
* add showborder property to 2d rviz plotter
* add plotter2d plugin
* use non-static and uniq string for overlay object
* implement OverlayText display plugin
* compile overlay text display
* add OverlayText.msg
* delete unneeded wrench files
* delete unneeded effort related files
* Merge pull request `#23 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/23>`_ from aginika/add-normal-diplay
  Add normal diplay
* add color channel and style property
* update to display in rviz
* update norml_display
* add normal_displays and normal_visuals
* delete point_display.cpp and point_visual.cpp
* Add the line to make the code in hydro
* ignore lib directory under jsk_rviz_plugins
* add gitignore for jsk_rviz_plugins
* do not create .so file under src directory
* depends on rviz using <depend> tag, because rviz failed to detect plugins from jsk_rviz_plugins without depend tag
* remove duplicated include line from polygon_array_display.h
  this duplication and quates in #include line happens compilation error about
  moc file of qt4
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: add wxwidgets dependency to jsk_rviz_plugins
* add dummy jsk-rviz-plugins.test
* use rosdep name for rviz and actionlib_msgs
* rendering backside face
* enabling alpha blending for PolygonArray
* fixing catkin cmake and dependency
* adding plugin to visualize PolygonArray
* add depends to jsk_footstep_msgs
* clear cache when toggle the check box of Footstep
* adding rviz plugin to visualize footstep
* paint point black if color is not available
* add select_point_cloud_publish_action for publish select points (no color)
* select action using combobox
* change msg type to actionlib_msgs
* add panel to cancel action
* add jsk_rviz_plugin::PublishTopic and remove Effort, wrenchStamped, PointStamped
* add rviz panel to send empty msg
* comment out SOURCE_FILES waiting for Issue `#246 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/246>`_
* use EXTRA_CMAKE_FLAGS to check to use ROSBUILD
* add dependencies to jsk_hark_msgs
* fix: validateFloats should be class method
* fix strequal ROS_DISTRO env
* use ROS_Distributions instead of ROS_DISTRO for electric
* add ambient_sound for groovy
* write libjsk_rviz_plugins under {PROJECT_SOURCE_DIR}/lib for and add export rviz to packages.xml, for groovy/catkin compile
* add debug message
* remove LIBRARY_OUTPUT_PATH and use catkin_package
* fix version
* fix to install plugin_descriptoin.xml and libjsk_rviz_plugins.so
* add comments
* fix for electric
* change msg:hark_msgs/HarkPower -> jsk_hark_msgs/HarkPower
* support groovy/cmake compile
* fix typo jsk_rviz_plugin -> jsk_rviz_plugins
* add test
* add package.xml
* add grad property
* added display ambient sound power
* add robot_description property
* add effort/max_effort property
* fix set sample color value for any scale value
* support enable button for each joint `#3597460 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3597460>`_
* remove color property
* fix when max_effort is zero, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* support scale for effort_plugin, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* update jsk_rviz_plugins
* add jsk_rviz_plugins
* Contributors: Shohei Fujii, Youhei Kakiuchi, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Ryohei Ueda, Yusuke Furuta
