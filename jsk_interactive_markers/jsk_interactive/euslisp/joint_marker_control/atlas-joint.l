#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(ros::roseus "atlas-test")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(atlas-init-ex :view nil :set-reset-pose nil)
;;(atlas-init-ex :set-reset-pose nil)
;; (set-user)
(real2model)
;;(model2real :time 5000 :wait t)

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
      (setq joint-angle (rad2deg (elt joint-angles x)))
      (send *atlas* (intern (string-upcase joint-name) *keyword-package*)
	    :joint-angle joint-angle)
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      )
    )
  )


(defun model2marker
  ()
  (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (let ((joint-angles nil)
	(joint-names nil)
	(joint-list (send *atlas* :joint-list))
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

    (ros::publish (format nil "~A/atlas/reset_joint_states" server-nodename)
		  joint-state-msg)
    )
  )

(defun reset-marker-pose
  ()
  (real2model)
  (model2marker)
)


(defun marker-menu-callback
  ( msg )
  (let ((menu (send msg :menu)))
    (cond
     ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
      (model2real :wait t :time 5000)
      ;; (if (y-or-n-p)
      ;; 	  (model2real :wait t :time 5000)
      ;; 	(warn "canceled~%")
      ;; ))
      )
     ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
      (reset-marker-pose)
     )
     ;;stand pose
     ((eq menu 100)
      (send *atlas* :stand-pose)
      (model2marker)
      )

     ;;manip pose
     ((eq menu 101)
      (send *atlas* :reset-manip-pose)
      (model2marker)
      )
    ))
)

(setq server-nodename "jsk_model_marker_interface")
(ros::subscribe (format nil "~A/atlas/joint_states" server-nodename)
		sensor_msgs::JointState #'joint-state-callback)

(ros::subscribe (format nil "~A/marker_menu" server-nodename)
		jsk_interactive_marker::MarkerMenu #'marker-menu-callback)

(ros::advertise (format nil "~A/atlas/reset_joint_states" server-nodename)
		sensor_msgs::JointState)

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

;;(set-user)
(reset-marker-pose)
(start-loop)