#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(load "package://jsk_interactive/euslisp/atlas-im.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")


(atlas-init-ex :view nil)
(real2model)
(model2real :time 5000 :wait t)
(fix-leg)

(setq *atlas-im* (instance atlas-im :init :robot *atlas* :ri *ri* :base-frame "/map"))

(send *atlas-im* :main-loop)