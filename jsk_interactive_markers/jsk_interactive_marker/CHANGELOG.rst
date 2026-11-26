^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive_marker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add LICENSE files (BSD 3-Clause) and use the SPDX identifier BSD-3-Clause in package.xml (`#910 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/910>`_)
* Contributors: Kei Okada

3.0.1 (2026-11-19)
------------------

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
* [jsk_interactive_marker] build/exec_depend on jsk_recognition_utils, not a generic depend
  check_dependency (catkin_pkg of focal) failed: the generic ROS 1 depend on
  jsk_recognition_utils is redundant with the ROS 2 build_depend and
  exec_depend, whatever their conditions.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] depend on jsk_recognition_utils on ROS 2 as well
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
* [jsk_interactive_marker] dummy_camera.py: fall back to rospy without jsk_ros1_ros2_compat on ROS 1
  jsk_ros1_ros2_compat is in jsk_common >= 3.0, which is released for ROS 2
  and ROS One only, not for indigo, kinetic, melodic and noetic, where
  dummy_camera.py has to work as before without it.
  Without jsk_ros1_ros2_compat (ROS 1 only), the five functions of
  rospy_rclpy_compat it uses are forwarded to rospy, as rospy_rclpy_compat
  does on ROS 1; inline, as it is the only script of this package using it.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] build_depend and exec_depend of jsk_recognition_msgs on ROS 2, not depend
  check_dependency failed: catkin_pkg reported the generic dependency of ROS 2 on
  jsk_recognition_msgs (0850d44) as redundant with its build_depend, build_export_depend and
  exec_depend of ROS 1, as it checks them without evaluating the conditions. On ROS 2 it is a
  build_depend and an exec_depend, as the generic one would be except for the export.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] depend on jsk_recognition_msgs on ROS 2 with JSK_RECOGNITION_MSGS_FROM_SOURCE=1
  As jsk_rviz_plugins: without the dependency, colcon may build jsk_interactive_marker before
  jsk_recognition_msgs of the sparse checkout of the ROS 2 CI, and bounding_box_marker and
  srv/IndexRequest.srv are skipped. The dependency is only with JSK_RECOGNITION_MSGS_FROM_SOURCE=1,
  which the CI sets, so rosdep of a normal workspace does not need the unreleased package.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] bounding_box_marker, IndexRequest.srv: build on ROS 2 when jsk_recognition_msgs is found
  jsk_recognition_msgs is not released for ROS 2, but builds there from
  source (`jsk-ros-pkg/jsk_recognition#2903 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2903>`_). With the same policy as for
  jsk_rviz_plugins, the ROS 2 build finds it QUIET and, when
  found, generates srv/IndexRequest.srv and builds bounding_box_marker (and
  its test); otherwise it says what it skips. It is not a ROS 2 dependency
  in package.xml, so rosdep does not look for it; colcon does not order it
  before this package either, so build it in an underlay.
  bounding_box_marker, in #if blocks on ROS 2: the same ~ topics and
  bounding_box_interactive_marker server on the node; stamp.nsec is nanosec
  there. indexRequest(), which is not advertised (and does not return), stays
  ROS 1 only.
  polygon_marker, footstep_marker, interactive_point_cloud and
  urdf_model_marker need jsk_recognition_utils as well, so they stay ROS 1
  only; footstep_marker is the only user of jsk_footstep_msgs (released for
  ROS 2), so nothing else could be ported for it.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] CMakeLists.txt: list the nodes not ported to ROS 2 yet
  triangle_foot, door_foot and point_cloud_config_marker, the DRC (Atlas)
  tools with their /map and /pelvis frames, need nothing that ROS 2 lacks
  but are not ported yet; door_foot's ~foot_list is a list of dicts, which
  ROS 2 parameters can not hold.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] add the ROS 2 tests
  test/ros2/test_samples.py (launch_testing) runs the ROS 2 samples
  without rviz2 and checks that
  - each marker_6dof of marker_6dof_sample serves its marker in map and
  echoes ~move_marker on ~pose;
  - transformable_server_sample inserts a box for request_marker_operate,
  serves it on simple_marker, publishes ~pose_with_name for ~set_pose and
  returns the pose for ~get_pose;
  - camera_info_publisher publishes the default and the yaml camera info;
  - dummy_camera.py publishes its camera info;
  - all of them exit cleanly.
  ROS 1 has no tests of this package.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] add the ROS 2 launch files of the samples
  launch/marker_6dof_sample.launch.py, transformable_server_sample.launch.py,
  sample_camera_info_publisher.launch.py and dummy_camera.launch.py, the
  ROS 2 counterparts of the .launch files of the nodes ported to ROS 2, with
  the rviz2 configs in config/ros2/ (rviz2 configs can not be shared with
  ROS 1), as jsk_rviz_plugins does. `gui:=false` runs them without rviz2.
  - transformable_server_sample.launch.py has no KORG nanoKONTROL and
  spacenav nodes (korg_nanokontrol and the configure scripts are ROS 1
  only); torus_udiv/torus_vdiv of the .launch are global parameters which
  the node does not read, so they are not given to it either.
  - dummy_camera.launch.py: tf2_ros's static_transform_publisher with the
  same x y z yaw pitch roll; launch_ros has no $(anon), so the default
  name is dummy_camera.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] dummy_camera.py: rospy -> jsk_ros1_ros2_compat
  dummy_camera.py runs on ROS 1 and ROS 2 through
  jsk_ros1_ros2_compat.rospy_rclpy_compat, imported directly as
  jsk_rqt_plugins does; jsk_ros1_ros2_compat becomes an exec_depend of both
  (on ROS 1 the CI builds jsk_common from source, .travis.rosinstall, until
  jsk_common 3.x is released for ROS 1). The publishing loop is a timer of
  the same 10 Hz; the stamps stay zero (rospy.Time(0)); CameraInfo's D, K,
  R, P are d, k, r, p on ROS 2.
  The ROS 2 build installs it with a python3 shebang, and config/ and
  models/ (the yaml files of camera_info_publisher and
  transformable_server_sample, the mesh of marker_6dof).
  transformable_markers_client.py (jsk_recognition_msgs/utils),
  semantic_robot_state_generator.py (the global /robot_description) and the
  joy/spacenav configure scripts stay ROS 1 only.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] transformable_server_sample: run on ROS 2
  jsk_rviz_plugins builds on ROS 2 now, so its messages (OverlayText,
  TransformableMarkerOperate, RequestMarkerOperate) are there, and
  jsk_rviz_plugins becomes a dependency of both.
  In #if blocks on ROS 2:
  - TransformableInteractiveServer: the same ~ topics and services on the
  node simple_marker (or its remapped name), created in a ROS 2
  constructor; the services through jsk_interactive_marker::
  advertiseService(), which calls the ROS 1 (Request&, Response&)
  callbacks unchanged. `n\_->param()` is a ParamHandle, the publishers are
  jsk_interactive_marker::Publisher, so `pub.publish(msg)` stays as is.
  cfg/InteractiveSetting.cfg (dynamic_reconfigure) is the parameters of
  the same names and ranges, which call configCallback() when set.
  tf2_ros's buffer for the transformation of the poses (with ~strict_tf).
  The interactive marker server is simple_marker as on ROS 1.
  - TransformableObject: tf2_ros's broadcaster, tf2_eigen for
  eigen_conversions; InteractiveSettingConfig is a struct of the same
  fields on ROS 2.
  - YamlMenuHandler: on the node; the menu topics are relative to the
  private namespace as with ROS 1's NodeHandle("~").
  ~use_parent_and_child (ParentAndChildInteractiveMarkerServer, which
  derives from the ROS 1 server and calls other servers' services from its
  callbacks) stays ROS 1 only; on ROS 2 it warns and uses the simple server.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] return from YamlMenuHandler::initMenu() and the TransformableObject defaults
  YamlMenuHandler::initMenu() fell off its end, and so did the default
  TransformableObject::getVisualizationMsgMarker() and
  getInteractiveMarkerScale(), which is undefined behavior: built for ROS 2
  (gcc 13), transformable_server_sample died with SIGILL right after
  reading ~yaml_filename. initMenu() returns true when it read the menu,
  the defaults an empty marker and the scale 1.0.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] world2yaml: build on ROS 2
  world2yaml is not a ROS node; of tf it uses Matrix3x3/Quaternion only,
  which are tf2's LinearMath on ROS 2 (the tf:: names are aliased there).
  It does not include ros1_ros2_compat.h, which is for the nodes, but the
  same <ros/ros.h> detection. tinyxml becomes a dependency of both; the
  ROS 2 build finds it with pkg-config (no cmake_modules there).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] camera_info_publisher: run on ROS 2
  In #if blocks on ROS 2:
  - ~camera_info, ~input and the interactive marker server on the node's
  name as on ROS 1; the timer of ~static_rate through createTimer();
  tf2_ros for the transform of the marker's feedback and the broadcast of
  ~parent_frame_id -> ~frame_id.
  - cfg/CameraInfoPublisher.cfg (dynamic_reconfigure) is the parameters
  ~width, ~height and ~f with the same defaults and range; setting them
  calls configCallback() as dynamic_reconfigure does. Integers are taken
  (dynamically typed) as dynamic_reconfigure takes them.
  - the fields of sensor_msgs/CameraInfo are d, k, r, p on ROS 2.
  - ROS_FATAL in ros1_ros2_compat.h, which jsk_ros1_ros2_compat lacks.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] marker_6dof: run on ROS 2
  ros1_ros2_compat.h gets, on ROS 2 only, the ROS 1 APIs that the nodes
  use most, so that their ROS 1 lines stay as they are:
  - jsk_interactive_marker::ParamHandle: pnh.param()/getParam()/hasParam()
  of a private ros::NodeHandle on the node's parameters, dynamically typed
  as rospy's (an integer is read as a double as on ROS 1);
  - jsk_interactive_marker::createTimer() with a ros::TimerEvent
  (current_real) for the ROS 1 timer callbacks.
  marker_6dof, in #if blocks: the node is ROS1_ROS2_COMPAT::g_node set by
  main(); ~pose/~move_marker are ~/pose and ~/move_marker; the interactive
  marker server is on the node's name (the same /<node>/update,
  /<node>/feedback); tf2_ros's broadcaster/buffer for ~publish_tf;
  jsk_topic_tools::readVectorParameter for ~initial_orientation is the
  parameter's double array. tf2 of ROS 2 refuses frame ids with a leading
  slash, so the one of ~frame_id (default /map) is dropped there.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] im_helpers (interactive_marker_helpers.cpp): build on ROS 2
  - include/jsk_interactive_marker/ros1_ros2_compat.h: the ROS 1 / ROS 2
  detection of jsk_ros1_ros2_compat/ros_version.h (`#if
  ROS_VERSION_MAJOR != 1` is ROS 2), and on ROS 2 only
  jsk_ros1_ros2_compat/compat.h and boost::bind's _1. It does not include
  jsk_ros1_ros2_compat on ROS 1, where jsk_common >= 3.0 is not released,
  so the ROS 1 dependencies stay as they are.
  - interactive_marker_helpers.h: the ROS 2 message headers, with the ROS 1
  spelling of the message types (JSK_ROS1_ROS2_COMPAT_MSG_ALIAS), so the
  declarations are the same on both.
  - interactive_marker_helpers.cpp: tf2's LinearMath for tf::Transform,
  Vector3, Quaternion and tf::quaternionTFToMsg on ROS 2; the zero stamp
  of makeGraspMarker() is builtin_interfaces' Time there.
  The ROS 2 build has it as jsk_interactive_marker_lib, since the target
  jsk_interactive_marker is the messages' one on ROS 2 (rosidl).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_interactive_marker] build on ROS 1 (catkin) and ROS 2 (ament) from one package.xml/CMakeLists.txt
  package.xml: format 3. The format 1 build_depend + run_depend pairs are
  depend, the dependencies only ROS 1 has (catkin, mk, rosbuild,
  message_generation/runtime, roscpp, tf, dynamic_reconfigure, ...) and
  those of the nodes that stay ROS 1 only get condition="$ROS_VERSION == 1";
  ament, rosidl, rclcpp, tf2 and jsk_ros1_ros2_compat get
  condition="$ROS_VERSION == 2"; build_type follows ROS_VERSION.
  CMakeLists.txt: the ROS 1 part is unchanged; the ROS 2 (ament) build is
  prepended in `if("$ENV{ROS_VERSION}" EQUAL 2) ... else()`, and the file
  ends with `endif()`. The ROS_VERSION guess for builds without
  ros_environment is the one of jsk_common's packages.
  This commit generates the messages and services on ROS 2; the nodes are
  added to the ROS 2 build with their ports in the following commits.
  rosidl does not accept two things of the ROS 1 definitions, so the ROS 2
  build generates them from copies in the build directory with
  `duration` -> `builtin_interfaces/Duration` (JointTrajectoryPointWithType)
  and `graspPose` -> `grasp_pose` (MoveObject). srv/IndexRequest.srv uses
  jsk_recognition_msgs, which is not released for ROS 2, so it is ROS 1
  only, as are the nodes that need jsk_recognition_msgs/utils,
  jsk_footstep_msgs, jsk_rviz_plugins, rviz, pcl_ros, dynamic_tf_publisher
  or urdf/kdl (listed in CMakeLists.txt).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

2.1.10 (2024-12-11)
-------------------
* jsk_interactive_marker: use catkin_install_python in CMakeLists.txt (`#897 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/897>`_)

* Contributors: Kei Okada

2.1.9 (2024-12-11)
------------------
* enable to jsk_rviz_plugins, without rviz/ogre (`#848 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/848>`_)

  use jsk_rviz_plugins message within embedded environment, compile with
  ```
  catkin bt -vi --cmake-args -DUSE_VISUALIZATION=OFF
  ```
* [ROS-O] patches (`#891 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/891>`_)

* drop ROS_DISTRO STRGREATER tests
  As the Debian packages use "Debian"
  https://salsa.debian.org/science-team/ros-ros-environment/-/blame/master/debian/rules?ref_type=heads#L6
  which is incompatible, does not trigger them
  c++14 is the default from melodic onward, so the additional statements
  there are not necessary
* CMakeLists.txt: check both kinetic and indigo

* [jsk_interactive_marker/scripts/transformable_markers_client.py] Fix for Python3 (`#882 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/882>`_)

* Fixed typo of Software License Agreement. and/o2r to and/or (`#853 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/853>`_)

* Contributors: Kei Okada, Shun Hasegawa, iory, v4hn

2.1.8 (2022-01-11)
------------------
* Resolve fixed frame of rviz (`#842 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/842>`_)

  * frame_id\_ is std::string, so we need to use c_str() or ROS_ERROR_STREAM
  * Resolve fixed frame of rviz
    * The pose in callback function of interactive marker is represented
    respect to fixed frame of rviz.
    * In order to publish consistent tf transformation, the pose should be
    transformed respect to the frame_id which is specified as ros
    parameter.

* Fix sample display robot state demo (`#838 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/838>`_)

  * [jsk_interactive_marker] fix bugs
  * [jsk_interactive_marker] change the robot of
    sample_display_robot_state from JAXON to PR2

* [jsk_interactive_markers][jsk_rqt_plugins] fix yaml load (`#818 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/818>`_)
* Enable to publish camera info from yaml file (`#793 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/793>`_)

* Contributors: Adi Vardi, Kei Okada, Koki Shinjo, Naoya Yamaguchi, Ryohei Ueda

2.1.7 (2020-10-17)
------------------

* set property for ccache if cmake version < 3.4 (`#780 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/780>`_)
* Remove meaningless lock (`#750 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/750>`_)
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * run 2to3 -f print
  * support noetic, use c++14, convert to package format 3

* Contributors: Kei Okada, Ryohei Ueda, Yuki Furuta

2.1.6 (2020-04-13)
------------------
* Remove unnecessary ROS_INFO (`#755 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/755>`_)
* Contributors: Yuto Uchimi

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* replace boost::shared_ptr by std::shared_ptr (`#710 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/710>`_)
  * manual urdf typedefs
  * add headers for urdf shared_ptr typedefs
  * use 'isnan' from 'std' namespace
  * use urdf typedefs for shared_ptr
  * enable C++11
  * replace boost pointers by std pointers

* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Call USE_SOURCE_PERMISSIONS before PATTERN
  * Update comment about installation
  * Install 'scripts' into SHARE_DESTINATION
  * Add comment for install destination
  * Fix installation destination

* fix typo in jsk_interactive_marker (`#718 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/718>`_)
* Remove newline and leading spaces from package.xml (`#706 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/706>`_)
* Explicitly resolve dependency for jsk_recognition_utils in package.xml (`#699 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/699>`_)
* Object 3d annotation using transformable interactive marker (`#668 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/668>`_)
  * do not compile transformable_marker_operator.cpp for jsk_recognition_msgs < 1.2.0
  * need 1.2.0 of jsk_recognition_msgs for Objects.msg
  * Update for updated Object.msg
  * Make object array topic editable and searched automatically
  * Insert mesh models dynamically by rviz panel
  * Change interactive_mode using dynamic reconfigure

* Contributors: Christian Rauch, Kei Okada, Kentaro Wada, Masaki Murooka, Shingo Kitagawa, Yuto Uchimi, Naoki Mizuno

2.1.3 (2017-10-26)
------------------

2.1.2 (2017-07-07)
------------------
* [jsk_rviz_plugins][classification_result_visualizer] minor bugfix (`#669 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/669>`_)
* [jsk_interactive_marker] Install scripts in jsk_interactive_marker (`#670 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/670>`_)
* Contributors: Kei Okada, Kentaro Wada

2.1.1 (2017-02-15)
------------------
* CMakeLists.txt: fix to support qt5
* package.xml: remove pr2eus_moveit from dependency
* Contributors: Yuki Furuta, Kei Okada

2.1.0 (2017-02-13)
------------------
* migration to kinetic, which uses qt5 wehre as indig/jade uses qt4 (`#662 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/662>`_ )
* Feature to transform markers in rviz (`#661 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/661>`_ )
  * Fix for same config in CMakeLists as jsk_rviz_plugins
    To fix error on hydro
  * Fix dependency of jsk_interactive_marker
  * Use throttle for ROS_ERROR
  * Move TransformableMarkerOperatorAction to jsk_interactive_marker
  * Update rviz for sample of transformable_markers
  * Use better marker size
  * Support transformation of dimension in transformable_markers_client
* Add client node for transformable markers (only boxes) (`#658 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/658>`_ )
  * Add TODO to extend transformable_markers_client
  * Rename transformable_boxes_client -> transformable_markers_client
  * Add client node for transformable boxes
* [maker_6dof] support mesh, publish pose topoic periodically ( `#657 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/657>`_ )
  * [jsk_interactive_marker/marker_6dof] add sample launch and doc.
  * [jsk_interactive_marker/src/marker_6dof] fix minor bug in line shape color setting.
  * [jsk_interactive_marker/src/marker_6dof] add option to select interactive marker size. default interactive marker size does not change.
  * [jsk_interactive_marker/src/marker_6dof] add option to publish pose periodically. default behavior does not change.
  * [jsk_interactive_marker/src/marker_6dof] supoort mesh as marker shape.
* Fix `#655 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/655>`_ : fix frame_id of tf published by marker 6dof( `#657 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/657>`_ )
  * Stop using fixed frame_id_ for simplicity
  * Transform pose in feedback with expected frame_id
  * Check frame_id of pose before publishing as TF
  * marker_6dof: publish tf after transformed to frame_id_, update is published with fixed frame of rviz
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Hiroto Mizohana

2.0.1 (2016-12-15)
------------------
* Fix jsk_recognition_msgs>=1.0.0 dep by jsk_interactive_marker
* Contributors: Kentaro Wada

2.0.0 (2016-12-14)
------------------
* ** Major Release** : Migrate srv files from jsk_pcl_ros to jsk_recognition_msgs ( `#644 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/644>`_ )
  see
  - https://github.com/jsk-ros-pkg/jsk_recognition/pull/1827
  - https://github.com/jsk-ros-pkg/jsk_recognition/pull/1914
  from this version, this package requires jsk_recognition higher than 1.0.0
* Remove dependency: jsk_interactive_marker on jsk_pcl_ros
* Stop using deprecated jsk_topic_tools/log_utils.h
  see
  - https://github.com/jsk-ros-pkg/jsk_common/pull/1462
  - https://github.com/jsk-ros-pkg/jsk_common/issues/1461
* Contributors: Kei Okada, Kentaro Wada

1.0.34 (2016-09-29)
-------------------

1.0.33 (2016-09-13)
-------------------
* Show description of only object which is selected `#633 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/633>`_
* [jsk_interactive_marker] Mode to display interactive manipultor only when selected `#626 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/626>`_

  * Refactor: remove variable which is not used
  * Add doc symlink for jsk_interactive_marker
  * Menu to enable/disable interactive manipulator
  * Mode to display interactive manipultor only when selected

* Validate object name is not empty to insert `#621 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/621>`_
* Set marker pose periodically for re-enabling on rviz `#618 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/618>`_

* Contributors: Kentaro Wada

1.0.32 (2016-07-20)
-------------------
* Fix missing cpp format string
* Contributors: Kentaro Wada

1.0.31 (2016-05-19)
-------------------

1.0.30 (2016-03-25)
-------------------

1.0.29 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_interactive_marker/euslisp] add transformable-object-util.
* [jsk_interactive_marker] Add document (JP) about how to use moveit_msgs/DisplayRobotState
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda

1.0.28 (2016-02-03)
-------------------
* [jsk_interactive_marker] Utility script and euslisp function to use moveit_msgs/DispalayRobotState
* [jsk_interactive_marker] Add ~use_2d parameter to remove z-axis translation
  and rotation around x-y axis
  Modified:
  - jsk_interactive_markers/jsk_interactive_marker/cfg/FootstepMarker.cfg
  - jsk_interactive_markers/jsk_interactive_marker/include/jsk_interactive_marker/footstep_marker.h
  - jsk_interactive_markers/jsk_interactive_marker/src/footstep_marker.cpp
* [jsk_interactive_marker]add parent and child interactive marker
* [out_of_body_marker.launch] odom_on_ground will be deprecated
* [jsk_interactive_marker] add some arguments to out_of_body_marker.launch
* [jsk_interactive_marker] add out_of_body_marker.launch
* Fix problem of genjava error caused by not listed message_generation
  Closes `#551 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/551>`_
  Modified:
  - jsk_interactive_markers/jsk_interactive_marker/package.xml
* [jsk_interactive_marker/urdf_model_marker] Check if urdf file exists
* [jsk_interactive_marker] Fix paren position of urdf_model_marker
* [jsk_interactive_marker] Untabify urdf_model_marker
* [jsk_interactive_marker] Add polygon marker to select one polygon
  out of multiple polygons
* Contributors: Eisoku Kuroiwa, Kentaro Wada, Ryohei Ueda, Yu Ohara

1.0.27 (2015-12-08)
-------------------
* Use ccache to make it faster to generate object file
* Contributors: Kentaro Wada

1.0.26 (2015-12-03)
-------------------
* [jsk_interactive_marker] add initialization for marker control
* [jsk_interactive_marker] Add ~force_to_replan service interface to
  footstep marker
* Contributors: Ryohei Ueda, Yu Ohara

1.0.25 (2015-10-10)
-------------------
* change service -> topic
* add right click config with yaml
* [jsk_interactive_markers] Ignore rvizconfig generated at build time
* Contributors: Kentaro Wada, Yu Ohara

1.0.24 (2015-09-08)
-------------------
* add jsk_recognition_msgs to catkin_package(CATKIN_DEPEND)
* add wait for trans to avoid tf nan error
* [jsk_interactive_markers/marker_6dof] Add ~initial_orientation parameter
* publish pose with focused marker name in transformable_interactive_server
* publish name of focused marker
* [jsk_interactive_markers/marker_6dof] Remove axis marker when remove
  6dof circle
* [jsk_interactive_marker/marker_6dof] Add ~initial_x,y,z parameters to
  specify initial position of the marker
* [jsk_interactive_marker/marker_6dof] Add ~publish_tf parameter to
  publish tf frame at the position of interactive marker.
* [jsk_interactive_marker/marker_6dof] Fix indent
* [jsk_interactive_marker] Add ~lock_joint_states and ~unlock_joint_states to furutaractive marker
  not to move joint angles of the model even if reset_joint_states is called
* [jsk_interactive_marker/footstep_marker] Publish current pose even if the marker is move via topic
* [jsk_interactive_marker/urdf_control_marker] Call dynamic_tf untili success if it failed
* [jsk_interactive_marker] Print error message if dynamic_tf is failed
  in urdf_control_marker
* [jsk_interactive_marker] Add hide/show empty service to transformable interactive server
* [jsk_interactive_marker/footstep_marker] Add dynamic reconfirue API to toggle projection to pointcloud
* [jsk_interactive_marker/footstep_marker] Always publish current pose of marker
* [jsk_interactive_marker/footstep_marker] Publish snapped pose even if service and topic API
  is used
* [jsk_interactive_marker/footstep_marker] Fix initialization of marker pose
* [jsk_interactive_marker] Use ros::ServiceClient::waitForExistence
  instead of ros::service::waitForService in order to wait
  dynamic_tf_publisher. Use different API leads potential bugs
* [jsk_interactive_marker/urdf_model_marker] Fix furutaractive when it fails to lookup tf transformation.
  1) use JSK_ROS_INFO, JSK_ROS_ERROR to output debug message
  2) Fix handling of tf lookup exception not to ignore addChildLinkNames
* [jsk_interactive_marker/urdf_control_marker] Visualize furutaractive marker without clicking urdf_control_marker by
  calling dynamic_tf in constructor
* [jsk_interactive_marker/footstep_marker] Set size of footsteps correctly
* [jsk_interactive_marker/footstep_marker] Fix projection of footprint
  when service interface is used
* Contributors: JSK Lab Member, Kei Okada, MasakiMurooka, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------
* [jsk_interactive_marker] Install include directory and library
* Contributors: Ryohei Ueda

1.0.22 (2015-06-24)
-------------------
* [jsk_interactive_marker] Install binaries
* [jsk_interactive_marker/footstep_marker] Enable ~footstep_margin parameter again
* always publish pose of transformable model
* Contributors: Ryohei Ueda, Masaki Murooka

1.0.21 (2015-06-11)
-------------------
* [jsk_interactive_marker] Add menu to select rleg or lleg as the first step
* [jsk_interactive_marker] Add menu to set heuristic
* [jsk_interactive_marker] Add ~always_planning parameter to footstep_marker
* [jsk_interactive_marker] Fix for terrain task
* [jsk_interactive_marker] Add topic interface to footstep_marker
* [jsk_interactive_marker] Decide footstep margin from robot name
* [jsk_interactive_marker] enable to change mesh marker control size
* [jsk_interactive_marker] add changing focus marker name line
* Contributors: Ryohei Ueda, Yu Ohara

1.0.20 (2015-05-04)
-------------------
* update furutaructive system. add load-ros-manifest. fix hand model path
* [jsk_interactive_marker/urdf_control_marker] Transform pose respected to fixed_frame_id
  when urdf_control_marker is moved by topic interface
* [jsk_interactive_marker] Supress messages while loading urdf mesh
* [jsk_interactive_marker] Add look ground menu to footstep_marker
* [jsk_interactive_marker] Fix timestamp handling in transformable object when ~strict_tf:=false
* Contributors: Masaki Murooka, Ryohei Ueda

1.0.19 (2015-04-09)
-------------------
* [jsk_interactive_marker/transformable_object] Add ~stric_tf parameter to
  relax timestamp soundness
* [jsk_interactive_marker] Add transformation of lleg and rleg when
  projecting goal footstep
* [jsk_interactive_marker/footstep_marker] Add ~use_projection_service to
  use other service to project footprint marker
* [jsk_interactive_marker] Fix projection of marker coordinates onto grid
  in footstep_marker
* [jsk_interactive_marker] Re-project projected coords if error is not
  small enough
* [jsk_interative_marker] Use SimpleOccupancyGridArray to project
  footprint marker
* [jsk_interactive_marker] Check tf2 exception in footstep_marker
* [jsk_interactive_marker] Check polygon to be snapped is valid or not
* add topic for control control
* use jsk_recognition_msgs instead of jsk_pcl_ros
* add offset topic for interactive control
* add configure for change control orientation mode
* add_mesh_model_in_transformable_marker
* suppress print in urdf_model_marker
* set radius instead of diameter when primitive shape is cylinder
* fix to set type correctly in getting dimenstions service
* Remove rosbuild files
* Contributors: Masaki Murooka, Ryohei Ueda, Yu Ohara, Yuki Furuta

1.0.18 (2015-01-30)
-------------------
* add link to boost

1.0.17 (2015-01-29)
-------------------
* [jsk_interactive_markers] Use jsk_recognition_msgs instead of jsk_pcl_ros
* add bounding box cropper example launch
* add bounding box marker version and line version
* revert target-cds-list to prevent ik failure
* modify robot-actions and pass the msg to box movement
* use move-object function in box-movement-callback
* add move base function and tuck arm function
* use base to carry object
* rename robot_actions_test to robot_actions_sample
* Contributors: Ryohei Ueda, Yusuke Furuta, Yuto Inagaki

1.0.16 (2015-01-04)
-------------------
* [jsk_interactive_marker] Add bsd license header to marker_6dof.cpp
* [jsk_interactive_marker] Keep pose of the marker even after toggling
  rotation circle in marker_6dof
* [jsk_interactive_marker] Add menu to toggle 6dof circle
* add README.md
* add rviz for robot_actions_test.launch
* add test for robot-actions.l
* change to load baxter-moveit instead of baxter-interface
* add moveit param to robot-actions.l
* use null space for aligning orientation
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into replace-self-to-ri
* replace self to *ri* in robot-actions.l
* check irtviewer
* Merge branch 'master' into split-ik-controller
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/euslisp/ik-controller.l
* change loading order
* add one click grasp parameter
* devide ik-controller.l into two scripts
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into refactoring-ik-control
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/euslisp/ik-controller.l
* use apply-joint_state
* refactoring ik controller deleted deprecated functions
* remove hoge from ik-control.l

1.0.15 (2014-12-13)
-------------------
* use robot-joint-interface in move bounding box
* add service request index to choose box from other program
* added config for show or not show controlls
* added spaces infront and behind equal
* update interactive marker controller for hrp2w
* changed dyn_reconfogure_effect_trigger_to_switch
* add interface to send joint trajectory
* add grasp hand method
* use moveit
* add JointTrajectoryPointWithType.msg
* Contributors: Ryohei Ueda, Yusuke Furuta, Yu Ohara, Yuto Inagaki

1.0.14 (2014-12-09)
-------------------
* added initializer if box_msg is empty
* added subscriber to set point_marker pose
* add show and hide marker method
* update pr2 spacenav sample
* load baxter-interface when using baxter
* refactoring ik-controller (delete functions and meanless comment out)
* add baxter to ik contollers
* Merge branch 'master' into irteus-to-urdf-interactive-marker
* add grasp offset
* fix generate msg
* added codes to pub relative_pose
* move and recognize handle
* Merge branch 'master' into irteus-to-urdf-interactive-marker
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/catkin.cmake
* changed marker_type because BUTTON_CLICK was not driven
* update joint state and root
* check ptr is null
* change staro config
* add open door function
* added service to check marker existence. added copy to marker operation.
* added service to get/set color and focus.
* added service function and service files for GetTransformableMarkerPose and SetTransformableMarkerPose
* publish tf of transformable marker in timer callback
* update transformable marker when dimension is set.
* added topic to add pose to transformable mareker with relative coords
* publish marker dimensions as topic. added MarkerDimensions.msg and use this message in service files.
* publish tf of transformable object.
* publish tf at low frequency
* add new msg
* add marker description and publish marker model movement
* add black list to fasten converting ros package file style
* improve speed of creating marker
* fix bag
* fix link when robot mode
* add center marker to urdf control marker
* modified teleop source for hrp2jsknts
* set fixed link
* adding hrp2w interactive markers
* rotate marker in world coorlds
* merge origin/master and fixed conflicts
* added service to set/get marker dimensions
* use msg int value for object shape.
* move msg to jsk_rviz_plugins
* add transformable marker operator panel
* add RequestMarkerOperate
* Merging and update
* integrate files and arrange srv
* disable one click grasp by default
* add reconfigure to transformable server
* use Eigen for transfomation in addPose
* added new srvice for transformable object. added ros service to insert/erase marker dynamically.
* update marker pose when marker menu changed
* display ik controller info
* Merge remote-tracking branch 'tarukosu/hand-marker-for-im-control' into hand-marker-for-im-control
* add scale for urdf marker
* use robot description for hand marker
* update launch file
* add samplerobot yaml
* add samplerobot controller
* add staro hand
* update base pose
* extract hand urdf from all urdf
* do not use *robot-offset*
* Remove warning message of FootstepMarker
* Supress messages from footstep_marker
* delete old code
* load urdf to visualize hand pose
* move functions to interactive_marker_utils.cpp
* add hand marker for hrp2
* tiny debug to avoid compilation warning
* Contributors: Hiroaki Yaguchi, Kei Okada, Yuto Inagaki, Ryohei Ueda, Masaki Murooka, Eisoku Kuroiwa, Yusuke Furuta, Yu Ohara

1.0.13 (2014-10-10)
-------------------
* add tabletop launch file
* Solve inverse kinematics with use-torso t
* do not use ik server for pr2
* add staro interactive marker
* add staro launch file
* add parameter to set menu
* add staro
* add arm ik and torso ik
* set frame id
* check having legs
* interactive marker control for hrp2
* add dependency on jsk_rviz_plugins
* Speed up grasp-pose movement
* add parameter
* rename launch file
* publish first handle pose
* add callback to grasp object
* subscribe initial handle pose
* add bounding box marker name and remove description
* refactor launch file
* do not use old ik-server-function
* move to jsk_interactive_marker and modify spacenav rotate
  add GetPose.srv
* Does not set the name of interactive marker for bounding box, because
  the name is too annoying
* update urdf model with topic
* add launch file to make bounding box interactive marker
* Contributors: Ryohei Ueda, Eisoku Kuroiwa, Yusuke Furuta, Yuto Inagaki

1.0.12 (2014-09-23)
-------------------
* Generate srv files on catkin
* Contributors: Ryohei Ueda

1.0.11 (2014-09-22)
-------------------
* jsk_interactive_marker does not depend on geometry
* Contributors: Ryohei Ueda

1.0.10 (2014-09-13)
-------------------
* add new executable to control CameraInfo with interactive marker
* Contributors: Ryohei Ueda

1.0.9 (2014-09-07)
------------------
* add ${PROJECT_NAME}_gencfg to all depends
* Contributors: Kei Okada

1.0.8 (2014-09-04)
------------------
* control marker with topic
* reset hand pose
* update catkin.cmake for urdf_control_marker
* root control marker for urdf marker
* rename config file
* use jsk_topic_tools::TimeAccumulator instead of jsk_pcl_ros::TimeAccumulator
* add include for catkin
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.7 (2014-08-06)
------------------
* add new program: pointcloud_cropper to crop pointcloud with interactive marker
* add config file for interactive point cloud
* update launch for pr2 gripper
* receive handle pose and publish it
* pick and place sample eus
* add reset root pose functions
* add reset marker callback
* rm empty line
* revert README.txt
* move .rviz.default to .rviz when making
* rename .rviz to .rviz.default
* use Eigen::Vector3f in footstep_marker because of the change of the api
  of jsk_pcl_ros
* depends on ${catkin_EXPORTED_TARGETS} to wait for message generation
* update footstep_marker to publish snapped pose to the planes
* support resuming the previous footstep on footstep_marker
* toggle 6dof marker via menu of footstep_marker
* toggle visualization of 6dof marker of footstep_marker via ~show_6dof_control parameter
* publish hand marker pose
* publish selected marker index
* snap the goal direction to the planes even with joy stick command
* do not use deprecated functions to convert tf and kdl instances to avoid
  compilation warning
* add 'Cancel Walk' menu to footstep marker
* Initialize the position of the marker to the frame if ~initial_reference_frame is specified
* register planDoneCB to the sendGoal function to the planner in footstep_marker
* asynchronously get the result of the footstep planning in footstep_marker
* add interactive_point_cloud.h
* add bounding box
* change paramater with dynamic reconfigure
* publish marker pose
* add interactive point cloud
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.6 (2014-07-14)
------------------
* add grasp method
* publish root pose when clicked
* launch file for pr2 gripper marker
* display multi marker
* add PR2 gripper xacro and setting file
* set initial joint state
* add class to set urdf marker config
* Contributors: Yusuke Furuta

1.0.5 (2014-06-29)
------------------
* add param to designate tf origin
* add new menu to call "estimate occlusion"
* skip planning until release the marker
* automatically snap the footstep marker to the plane if ~use_plane_snap
  is set to true
* publish the selected bounding box as BondingBoxArray for visualization
* publish the selected box as well as the index of the box
* add dummy camera launch file
* Contributors: Ryohei Ueda, Yusuke Furuta, Masaki Murooka

1.0.4 (2014-05-31)
------------------
* jsk_interactive_marker: fix for rosbuild, add mk/rosbuild to package.xml
* add "execute the plan" and "force to replan" mouse menu to footstep_marker
* add bounding_box_marker to select jsk_pcl_ros/BoundingBoxArray
* Contributors: Ryohei Ueda, Kei Okada

1.0.3 (2014-05-22)
------------------
* update depreceted functions
* add depend to roslib roscpp for ros::package

1.0.2 (2014-05-21)
------------------
* add interactive_markers and urdf

1.0.1 (2014-05-20)
------------------
* use geometry package to install orocos_kdl, since orocos_kdl is not installed via rosdep https://github.com/ros/rosdistro/pull/4336
* Contributors: Kei Okada

1.0.0 (2014-05-17)
------------------

0.0.3 (2014-05-15)
------------------

0.0.2 (2014-05-15)
------------------
* compile executables after message generation
* wait for service before making service client
* remove dependency on hrpsys_gazebo_atlas when using pr2
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into service-persistent-true
* use rotation-axis in inverse-kinematics
* set persistent true in dynamic_tf_publisher_client
* delete code using robot_state_publisher
* delete move_base_marker
* add jsk_pcl_ros message dependency
* change the location of catkin_package and generate_messages
* change marker frame id to /map
* Add method to set marker root link to robot root link
* Not use joint_state_publisher but use dynamic_tf_publisher when making
  robot marker
* add method to publish base pose
* add an interface to call footstep_controller from other programs to footstep_marker
* support foot offset parameters for initial feet placements
* use tf_conversions and eigen_conversions to convert tf::Transform to geometry_msgs::Pose
* use tf::Transform to store offset information
* implement readPoseParam
  a function to read geometry_msgs/Pose from a rosparameter
* add gitignore to jsk_interactive_marker
* add move base marker sample
* add controller to move base
* use tf_prefix instead of model name
* divide ik-controller.l into class and make instance
* use yaml for move base marker
* use `'tf_prefix_' instead of 'model_name_ + /'`
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix typo of jsk_interactive_marker of manifest.xml
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: reverted depend tags in manifest.xml of jsk_interactive_markers
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: add actionlib dependency to jsk_interactive_marker
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix catkin cmake syntax: CATKIN-DEPENDS -> CATKIN_DEPENDS
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix description of jsk_interactive_marker/manifest.xml
* use rosdep name for rviz and actionlib_msgs
* add urdf marker in order to move base link
* add method to get joint state from robot
* update footstep_marker in order to reset iniital pose
* not use ik-server
* trying to deal with new ik server
* fixing urdf_model_marker to link urdf_model_maker_main.cpp
* divide urdf_model_marker into class definition and main function
* fixing the position of the frame id
* use interactive_marker_helpers
* initialize feet position correctly
* add hand frame slot in ik-controller
* delete ros warining and make faster
* modify pass to pr2 ik server
* adding marker to visualize initial state
* adding method to estimate initial state of footstep from frame_id
* catkinize jsk_interactive_marker for hydro
* use joint state publisher when using pr2
* add mesh file path in linkMarkerMap
* fix bag in method to find ros package path from full path
* add method to move root link
* add ik controller and launch file
* update urdf model markers testfile
* adding marker_6dof, which is controllable via rostopic and rviz
* add launch file to controll robot with interactive marker
* add base_frame parameter in point_cloud_marker
* supporting z-direction
* calling SnapIt from outer program
* enable footstep planner in sample
* support to disable planner calling from footstep_marker
* add use_visible_color parameter to change color
* adding interactive marker for footstep planning
* adding footstep interactive marker
* set Use Link as Arm by default
* rotate hand in local coordinates
* add src to convert .world to .yaml
* rename Don't allow rotation / allow rotation, use 6D / 3D, 3D (positon) as default
* add subscriber to toggle rotation axis
* add subscriber to toggle start ik
* add center sphere marker to control position
* change door marker size
* show footsteps each 2
* remenmber previous door position
* fix previous step button
* supporting showing footstep list
* set foot step by rosparam
* update
* change resolution of knob color
* change control size to max size of box
* add color knob
* get scale from urdf
* clean up code and write dummy 0 joint-angle to Joint::PRISMATIC
* add wall in door_foot.cpp
* change foot position when open door
* use robot description in atlas-real
* add sphere and box marker in urdf model marker
* add sphere and box marker in urdf model marker
* be quiet
* updating rviz
* add look at menu and message
* add marker to visualize door and foot
* fix foot position of triangle
* add move it exec cancel button
* update defaultset
* fix bag of urdf_model_marker
* add Triangle Marker to visualize foot position
* add Touch It msg
* adding clear function for external program
* changing default value
* not publish joint state all time
* adding some external control
* updating for external programs
* untabify
* add change marker size menu
* stop ik by default
* fix bag and reset marker id when clear button is pressed
* add IM to get designated Point Cloud
* add menu to select using ik server
* reset when marker was reset
* fix to use joint_state_publisher and robot_state_publisher
* add joint_state_publisher.py
* add use_dynamic_tf to disable dynamic tf
* change marker size of urdf marker
* publishJointState on resetMarkerCB
* add special pose (fg manip pose)
* we can show and hide interactive marker
* add .rviz  for interactive_marker
* change frame-id from odom to map
* modify caliculation of tf from odom to marker
* add menu to cancel planned motion
* add visualizaion mode to visualize IK
* we can select Arm Ik , Torso Ik or Fullbody Ik
* add registration mode in urdf_model_marker
* added marker_array for viewing collision lines in rviz
* add .rviz for atlas_joint_marker
* Use package:// instead of file:// to designate mesh file name
* use jsk urdf model for atlas
* add launch file for moving joints for atlas
* update README.txt
* display parent link marker when fixed joint clicked
* add joint limit in joint robot marker
* add Function to set 1 Joint Angle
* reset robot marker to real robot
* add patch file for atlas.urdf to use RobotIM
* add Move Robot Joint Marker
* add cylinder marker when joint dont include mesh
* add yaml for Fridge model in 73b2
* add msg to designate marker movement
* attach Grasp Point to Model Marker
* change display of move marker when clicking
* use configuration yaml file to set models
* get full path of gazebo model
* set Move Marker based on Joint axis
* add dependancy on dynamic_tf_publisher
* making interactive marker based on urdf model
* add finger interactive marker
* add menu to change whether robot use torso
* add Marker Type in msgs
* add hand shape interactive marker
* add interactive operation sample of eus simulator
* add head marker and change msg
* add jsk_interactive_markers/ by yusuke furuta
* Contributors: Youhei Kakiuchi, Kei Okada, Shintaro Noda, Masaki Murooka, Ryohei Ueda, Yusuke Furuta
