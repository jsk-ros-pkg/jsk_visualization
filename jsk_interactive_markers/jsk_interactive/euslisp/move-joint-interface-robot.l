#!/usr/bin/env roseus

(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::roseus "move_joint_interface")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://pr2eus/pr2-interface.l")

(defvar *robot-name* (ros::get-param "~robot" "PR2"))

(cond
 ((equal (string-upcase *robot-name*) "PR2")
  (setq *robot-name* "PR2")
  (pr2-init)
  (setq *robot* *pr2*)
  )
 ((equal (string-upcase *robot-name*) "ATLAS")
  (setq *robot-name* "atlas")
  (atlas-init-ex :view nil :set-reset-pose nil)
  (setq *robot* *atlas*)
  )
 )

(when (and (boundp '*irtviewer*) *irtviewer*)
  (send *irtviewer* :change-background #f(0.3 0.3 0.7))
  (send *irtviewer* :title "Interactive Marker Joints")
  (send *irtviewer* :draw-objects))

(defun joint-state-callback
  (msg)
  (let ((joint-names (send msg :name))
       (joint-angles (send msg :position))
       joint-name joint-angle
       )
    (dotimes (x (length joint-names))
      (setq joint-name (elt joint-names x))
      (let ((hand-method (intern (string-upcase joint-name) *keyword-package*)))
	(when (find-method *robot* hand-method)
	  (if (subclassp (class (send *robot* hand-method)) rotational-joint)
	      (setq joint-angle (rad2deg (elt joint-angles x)))
	    (setq joint-angle (* 1000.0 (elt joint-angles x)))
	    )
	  (send *robot* hand-method :joint-angle joint-angle)))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
;;    (model2real :wait t :time 5000)
    )
  )

(setq server-nodename "jsk_model_marker_interface")

(ros::subscribe (format nil "~A/~A/joint_states_ri_move" server-nodename *robot-name*)
		sensor_msgs::JointState #'joint-state-callback)

(ros::rate 30)

(warn "type (start-loop) to start~%")

(defun start-loop
  ()
  (while t
    (send *ri* :spin-once)
    (ros::spin-once)
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::sleep)
    (unless (ros::ok) (return)))
  )

(start-loop)