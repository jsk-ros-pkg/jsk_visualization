^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.7 (2020-10-17)
------------------
* add noetic test (e`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * fix python to pass python3 -m compileall

* Contributors: Kei Okada

2.1.6 (2020-04-13)
------------------

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Install 'scripts' into SHARE_DESTINATION
  * Add comment for install destination
  * Fix installation destination
* Contributors: Yuto Uchimi

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

1.0.19 (2015-04-09)
-------------------
* Remove rosbuild files
* [jsk_interactive_test] Remove joy_mouse dependency
* Contributors: Ryohei Ueda

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
* use joy_mouse package
* Contributors: Ryohei Ueda

1.0.7 (2014-08-06)
------------------

1.0.6 (2014-07-14)
------------------

1.0.5 (2014-06-29)
------------------

1.0.4 (2014-05-31)
------------------
* jsk_interactive_test: catkinize pckage
* Contributors: Kei Okada

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
* add show score on the web
* add displayResult
* recording the result into web server
* supporting CONTROLLER argument
* fix size from 3(27) -> 2(8)
* update jsk_interactive_test with bottle and handle
* change spacenav value to hydra message to move interactive_cursor
* use nanokontrol for trackpoint
* use nanokontrol for trackpoint
* update launch file for xbox
* modified top_view and side_view
* launch rviz for top and side view in trackpoint.launch
* changed coordinate system of moving hand marker with trackpoint
* added trackpoint_align_windows.sh
* added rviz config files for top view and side view
* add argument and param MAKE_INTERACTIVE_MARKER_ARROW in interactive_test.launch
* added trackpoint.launch
* enable to select whether display interactive marker arrow or not by rosparam.
* add trackpoint_controller.py: move interactive_marker with trackpoint and nanokontrol
* add only rotation mode for spacenav
* rpy rotation suppported
* add publishing done time
* fix :time_from_start -> duration
* add use_rotation parameter
* add speed up/down button
* update rate and message
* add spacenav controller
* updating view plugin and adding move_marker topic
* add jsk_interactive_test
* Contributors: Youhei Kakiuchi, Kei Okada, Shintaro Noda, Masaki Murooka, Ryohei Ueda, Yusuke Furuta
