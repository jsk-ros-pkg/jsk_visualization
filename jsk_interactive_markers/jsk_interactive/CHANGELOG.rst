^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.7 (2020-10-17)
------------------

2.1.6 (2020-04-13)
------------------

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Add comment for install destination
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
* Contributors: Ryohei Ueda

1.0.18 (2015-01-30)
-------------------

1.0.17 (2015-01-29)
-------------------

1.0.16 (2015-01-04)
-------------------

1.0.15 (2014-12-13)
-------------------
* use robot-joint-interface in move bounding box
* add grasp hand method
* use moveit
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.14 (2014-12-09)
-------------------
* reduce load
* add baxter to ik contollers
* Merge pull request `#192 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/192>`_ from hyaguchijsk/feature/add_hrp2w_interactive_marker_control
  adding hrp2w interactive markers
* modified teleop source for hrp2jsknts
* adding hrp2w interactive markers
* do not use *robot-offset*
* Contributors: Yuki Furuta, Hiroaki Yaguchi, Kei Okada, Masaki Murooka, Yuto Inagaki

1.0.13 (2014-10-10)
-------------------
* interactive marker control for hrp2
* publish move time
* Contributors: Yusuke Furuta

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
* add grasp method
* Contributors: Yusuke Furuta

1.0.5 (2014-06-29)
------------------
* add load-ros-manifest jsk_interactive_marker
* Contributors: Yusuke Furuta

1.0.4 (2014-05-31)
------------------
* jsk_interactive: catkinize
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
* add controller to move base
* add service to get joint state
* add callback to move gripper
* fix torso angle bag when receive joint state
* add robot spin once in loop
* modify joint-interaface in order to use other robot
* add interaface to move real robot run in robot
* not use (model2real) in initialization
* move to special pose (fg reset-manip-pose)
* don't reset robot pose when initialization
* don't use x::window-main-one when display doesn't exist
* wait until tf is published
* enable to select whether make irtviewer or not in interactive marker files
* do not make viewr in atals-im-main.l
* do not make viewer in atals-joint.l
* change frame-id from odom to map
* modify caliculation of tf from odom to marker
* add plan and execute mode
* we can select Arm Ik , Torso Ik or Fullbody Ik
* add .rviz for atlas_joint_marker
* Use package:// instead of file:// to designate mesh file name
* refactor atlas-joint.l in jsk_interactive
* merge joint-controll to robot-im.l
* change the folder of lisp code
* mv file for control joint
* mv euslisp file from scripts to euslisp
* add robot joint marker control
* ik including base for pr2
* pr2 moves when interactive model moves
* add finger interactive marker
* use torso when solving IK
* Can change headmode (Automatic or by Hand)
* use defined value by msg
* add head-marker callback
* add jsk_interactive_markers/ by yusuke furuta
* Contributors: Yusuke Furuta, Kei Okada, Masaki Murooka
