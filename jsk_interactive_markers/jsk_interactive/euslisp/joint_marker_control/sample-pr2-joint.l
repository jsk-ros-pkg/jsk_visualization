#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(ros::roseus "pr2-test")
(load "package://pr2eus/pr2-interface.l")

(pr2-init t)
(send *pr2* :angle-vector (send *ri* :state :angle-vector))

(defun joint-state-callback
  (msg)
  (let ((joint-names (send msg :name))
       (joint-angles (send msg :position))
       joint-name joint-angle
       )
    (dotimes (x (length joint-names))
      (setq joint-name (elt joint-names x))
      (setq joint-angle (rad2deg (elt joint-angles x)))
      (when (find-method *pr2* (intern (string-upcase joint-name) *keyword-package*))
	(send *pr2* (intern (string-upcase joint-name) *keyword-package*)
	      :joint-angle joint-angle)
	)
      )
    (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )
  )

(defun marker-menu-callback
  ( msg )
  (let ((menu (send msg :menu)))
    (cond
     ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
      (send *ri* :angle-vector (send *pr2* :angle-vector) :time 5000)
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
      (send *pr2* :angle-vector (send *ri* :state :angle-vector))
      (let ((joint-angles nil)
	    (joint-names nil)
	    (joint-list (send *pr2* :joint-list))
	    (joint-state-msg 
	     (instance sensor_msgs::JointState :init 
		       :header (instance std_msgs::header :init 
					 :stamp (ros::time-now)))))
	(dotimes (x (length joint-list))
	  (push (deg2rad (send (elt joint-list x) :joint-angle)) joint-angles)
	  (push (send (elt joint-list x) :name) joint-names)
	   )
	(send joint-state-msg :position joint-angles)
	(send joint-state-msg :name joint-names)

	(ros::publish (format nil "~A/PR2/reset_joint_states" server-nodename)
		      joint-state-msg)
	))

     )
    ))

(setq server-nodename "jsk_model_marker_interface")
(ros::subscribe (format nil "~A/PR2/joint_states" server-nodename)
		sensor_msgs::JointState #'joint-state-callback)

(ros::subscribe (format nil "~A/marker_menu" server-nodename)
		jsk_interactive_marker::MarkerMenu #'marker-menu-callback)

(ros::advertise (format nil "~A/PR2/reset_joint_states" server-nodename)
		sensor_msgs::JointState)

(ros::rate 30)

(warn "type (start-loop) to start~%")

(defun start-loop
  ()
  (while t
   (ros::spin-once)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )