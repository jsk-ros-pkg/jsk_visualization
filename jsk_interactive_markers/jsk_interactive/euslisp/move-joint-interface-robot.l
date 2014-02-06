#!/usr/bin/env roseus

(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::roseus "atlas-test")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://pr2eus/pr2-interface.l")

#|
(atlas-init-ex :view nil :set-reset-pose nil)
(real2model)
|#
(pr2-init)

(setq *robot* *pr2*)


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
	(setq joint-angle (rad2deg (elt joint-angles x)))
	(if (find-method *robot* hand-method)
	    (send *robot* hand-method :joint-angle joint-angle)))
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
;;    (model2real :wait t :time 5000)
    )
  )

(defun move-rleg-ankle-p
  (msg)
  (real2model)
  (send *robot* :rleg :ankle-p :joint-angle (send msg :data))
  (model2real :time 4000)
  )

(defun move-lleg-ankle-p
  (msg)
  (real2model)
  (send *robot* :lleg :ankle-p :joint-angle (send msg :data))
  (model2real :time 4000)
  )

(setq server-nodename "jsk_model_marker_interface")

(ros::subscribe (format nil "~A/PR2/joint_states_ri_move" server-nodename)
		sensor_msgs::JointState #'joint-state-callback)

(ros::subscribe (format nil "~A/PR2/rleg_ankle_p" server-nodename)
		std_msgs::float32 #'move-rleg-ankle-p)

(ros::subscribe (format nil "~A/PR2/lleg_ankle_p" server-nodename)
		std_msgs::float32 #'move-lleg-ankle-p)


(ros::rate 30)

(warn "type (start-loop) to start~%")

(defun start-loop
  ()
  (while t
   (ros::spin-once)
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)

   (unless (ros::ok) (return)))
  )

(start-loop)