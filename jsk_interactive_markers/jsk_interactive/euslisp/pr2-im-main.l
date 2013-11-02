#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(load "package://jsk_interactive/euslisp/pr2-im.l")
(load "package://pr2eus/pr2-interface.l")

(pr2)
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

(setq *pr2-im* (instance pr2-im :init :robot *pr2* :ri *ri*))

(send *pr2-im* :main-loop)