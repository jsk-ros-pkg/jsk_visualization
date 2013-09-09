#!/usr/bin/env roseus

(ros::load-ros-manifest "atlas_interactive")
(load "package://jsk_interactive/scripts/atlas-im.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(my-init)

(setq *atlas-im* (instance atlas-im :init :robot *atlas* :ri *ri* :server-nodename "atlas_manipulation_interactive_marker" :base-frame "/utorso"))

(send *atlas-im* :main-loop)