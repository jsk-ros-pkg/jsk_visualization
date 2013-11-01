#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(load "package://jsk_interactive/scripts/atlas-im.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(my-init :set-reset-pose t)
(send *ri* :start-st)

(setq *atlas-im* (instance atlas-im :init :robot *atlas* :ri *ri* :base-frame "/odom"))

(send *atlas-im* :main-loop)