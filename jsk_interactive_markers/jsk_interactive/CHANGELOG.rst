^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
