#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive_marker")

(defclass robot-joint-interface
  :slots (robot
	  joint-traj-msg
	  joint-traj-topic))
(defmethod robot-joint-interface
  (:init
   (robot-model topic-name)
   (setq robot robot-model)
   (setq joint-traj-topic topic-name)
   (ros::advertise joint-traj-topic jsk_interactive_marker::JointTrajectoryWithType)
   (send self :init-joint-traj-msg)
   )

  (:init-joint-traj-msg
   ()
   (setq joint-traj-msg (instance jsk_interactive_marker::JointTrajectoryWithType :init))
   (send joint-traj-msg :joint_names (send robot :joint-list :name))
   )

  (:reset
   ()
   (send self :init-joint-traj-msg))

  (:start-grasp
   (&rest args)
   (let ((point (instance jsk_interactive_marker::JointTrajectoryPointWithType :init))
	 (points (send joint-traj-msg :points)))
     (send point :args (format nil "~A" args))
     (send point :type jsk_interactive_marker::JointTrajectoryPointWithType::*CLOSE_HAND*)
     (send joint-traj-msg :points (append points (list point)))
     ))
  

  (:stop-grasp
   (&rest args)
   (let ((point (instance jsk_interactive_marker::JointTrajectoryPointWithType :init))
	 (points (send joint-traj-msg :points)))
     (send point :args (format nil "~A" args))
     (send point :type jsk_interactive_marker::JointTrajectoryPointWithType::*OPEN_HAND*)
     (send joint-traj-msg :points (append points (list point)))
     ))

  (:angle-vector
   (av &optional (tm 3000) &key (moveit nil))
   (let ((point (instance jsk_interactive_marker::JointTrajectoryPointWithType :init))
	 (points (send joint-traj-msg :points)))

     (if moveit
	 (send point :type jsk_interactive_marker::JointTrajectoryPointWithType::*COLLISION_AVOIDANCE*)
       (send point :type jsk_interactive_marker::JointTrajectoryPointWithType::*JOINT_INTERPOLATION*))

     (send point :wait t)

     ;;set positions
     (send robot :angle-vector av)
     (send point :positions (send robot :joint-list :ros-joint-angle))
     
     ;;set time
     (send point :time_from_start (ros::time (/ tm 1000.0)))
     (send joint-traj-msg :points (append points (list point)))
     ))

  (:start-trajectory
   ()
   (send joint-traj-msg :header :stamp (ros::time-now))
   (ros::publish joint-traj-topic joint-traj-msg)
   (send self :reset)
   )
  )


#|
(require "package://pr2eus/pr2-interface.l")
(ros::roseus "robot-joint-interface-test")
(setq *pr2* (pr2))
(setq *rji* (instance robot-joint-interface :init *pr2* "/jsk_model_marker_interface/robot/joint_trajectory_with_type_ri_move"))

(send *pr2* :reset-pose)
(send *rji* :angle-vector (send *pr2* :angle-vector) 5000)
(send *pr2* :reset-manip-pose)
(send *rji* :angle-vector (send *pr2* :angle-vector) 5000)

(send *rji* :stop-grasp :rarm)

(send *pr2* :reset-pose)
(send *rji* :angle-vector (send *pr2* :angle-vector) 5000 :moveit t)

(send *rji* :start-grasp :rarm :gain 0.004)

(send *rji* :start-trajectory)
(print-ros-msg (cdadr (send *rji* :slots)))
#|
