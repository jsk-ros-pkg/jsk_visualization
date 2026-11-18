^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [jsk_visualization] a package which depends on the others, not a metapackage
  bloom refuses to release the metapackage on ROS 2 because its
  CMakeLists.txt is not the one of REP 127 (it branches on $ROS_VERSION),
  so, as jsk_common does, export no <metapackage/> and call catkin_package()
  on ROS 1.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

3.0.0 (2026-11-19)
------------------
* Merge pull request `#905 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/905>`_ from k-okada/rqt-plugins-ros1-ros2
  ROS 1 / ROS 2 for the whole repository from one source tree (jsk_ros1_ros2_compat)
* [jsk_visualization] build the metapackage on ROS 1 (catkin) and ROS 2 (ament)
  As on the ros2 branch (`#903 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/903>`_, "add jsk_visualization meta package"):
  package.xml format 3, ament_cmake on ROS 2 and the author line; here
  from one package.xml/CMakeLists.txt for both, as jsk_common does:
  - package.xml: catkin on ROS 1, ament_cmake (and ros_environment) on
  ROS 2; jsk_rqt_plugins, jsk_rviz_plugins and jsk_interactive_marker
  build on both, so they are exec_depends of both; jsk_interactive and
  jsk_interactive_test are not ported to ROS 2 (euslisp), so they are
  ROS 1 only. build_type follows ROS_VERSION.
  - CMakeLists.txt: the ROS 1 lines (catkin_metapackage()) are unchanged,
  the ament build is prepended in `if("$ENV{ROS_VERSION}" EQUAL 2) ...
  else()` and the file ends with `endif()`. catkin keeps building it as a
  metapackage; catkin_pkg only warns that a metapackage should have no
  buildtool_depend but catkin, since it does not evaluate the conditions
  of the ROS 2 ones there.
  Co-authored-by: Yoshiki Obinata <mqcmd196@hotmail.co.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

2.1.10 (2024-12-11)
-------------------

2.1.9 (2024-12-11)
------------------

2.1.8 (2022-01-11)
------------------

2.1.7 (2020-10-17)
------------------

2.1.6 (2020-04-13)
------------------

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------

2.1.3 (2017-10-26)
------------------

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

1.0.30 (2016-03-25)
-------------------

1.0.29 (2016-03-20)
-------------------

1.0.28 (2016-02-03)
-------------------

1.0.27 (2015-12-08)
-------------------

1.0.26 (2015-12-03)
-------------------

1.0.25 (2015-10-10)
-------------------

1.0.24 (2015-09-08)
-------------------

1.0.23 (2015-07-15)
-------------------

1.0.22 (2015-06-24)
-------------------

1.0.21 (2015-06-11)
-------------------

1.0.20 (2015-05-04)
-------------------
* [jsk_visualization] Add jsk_visualization meta package
* Contributors: Ryohei Ueda

1.0.19 (2015-04-09)
-------------------

1.0.18 (2015-01-30)
-------------------

1.0.17 (2015-01-29)
-------------------

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

1.0.4 (2014-05-31)
------------------

1.0.3 (2014-05-22)
------------------

1.0.2 (2014-05-21)
------------------

1.0.1 (2014-05-20)
------------------

1.0.0 (2014-05-17)
------------------

0.0.3 (2014-05-15 14:00)
------------------------

0.0.2 (2014-05-15 11:06)
------------------------
