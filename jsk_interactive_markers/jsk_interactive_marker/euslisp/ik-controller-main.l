#!/usr/bin/env roseus

(ros::roseus "ik-controller")

(require "package://jsk_interactive_marker/euslisp/ik-controller.l")

(setq *im-controller* (instance im-controller :init))
