#!/usr/bin/env roseus

(ros::roseus "move-base-controller")

(require "package://jsk_interactive_marker/euslisp/ik-controller.l")

(defclass base-controller
  :super im-controller
  :slots (target-coords
	  *move-joint-nodename*))


(defmethod base-controller
  (:init
   ()
   (defvar *start-loop* nil)
   (setq *move-joint-nodename* "jsk_model_marker_interface")
   (defvar *im-nodename* "jsk_move_base_marker")
   (send-super :init)

   (ros::advertise (format nil "~A/~A/base_move" *move-joint-nodename* *robot-name*)
		   geometry_msgs::PoseStamped)

   (ros::subscribe
    (format nil "~A/base_pose" *im-nodename*)
    geometry_msgs::PoseStamped #'send self :target-coords-callback)



   (send self :demo-pos-controller)
   )


  (:marker-menu-callback2
   ( msg )
   (let ((menu (send msg :menu))
	 (type (send msg :type)))
     (cond
      ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
       (print "publish move base target")
       (send self :publish-target-coords)
       )
      (t
       (send-super :marker-menu-callback2 msg)
       )
      ))

   )


  (:target-coords-callback
   (msg)
   (setq target-coords (ros::tf-pose-stamped->coords msg))
   (print "update target")
   )


  (:publish-target-coords
   ()
   (print target-coords)
   (when target-coords
     (let ((base-msg (ros::coords->tf-pose-stamped target-coords (send target-coords :name))))
       (ros::publish (format nil "~A/~A/base_move" *move-joint-nodename* *robot-name*)
		     base-msg)
       )
     )
   )
  )



(setq *base-controller* (instance base-controller :init))
