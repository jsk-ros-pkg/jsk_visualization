#!/usr/bin/env roseus

;;(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::load-ros-manifest "jsk_interactive_marker")
(ros::roseus "move_joint_interface")

(load "package://pr2eus/pr2-interface.l")

(defvar *robot-name* (ros::get-param "~robot" "PR2"))
(defvar *robot-topic-name* (ros::get-param "~robot_topic_name" "robot"))

(cond
 ((equal (string-upcase *robot-name*) "PR2")
  (setq *robot-name* "PR2")
  ;;use moveit
  (load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
  (pr2-init)
  (send *ri* :set-moveit-environment (instance moveit-environment :init))
  (setq *robot* *pr2*)
  )
 ((equal (string-upcase *robot-name*) "HRP2JSKNT")
  (setq *robot-name* "hrp2jsknt")
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (hrp2jsknt-init)
  (setq *robot* *hrp2jsknt*)
  )
 ((equal (string-upcase *robot-name*) "HRP2JSKNTS")
  (setq *robot-name* "hrp2jsknts")
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  )
 ((equal (string-upcase *robot-name*) "HRP2W")
  (setq *robot-name* "hrp2w")
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
  (hrp2w-init)
  (setq *robot* *hrp2w*)
  )
 ((equal (string-upcase *robot-name*) "BAXTER")
  (setq *robot-name* "baxter")
  (load "package://baxtereus/baxter-interface.l")
  (baxter-init)
  (setq *robot* *baxter*)
  )
 ((equal (string-upcase *robot-name*) "STARO")
  (setq *robot-name* "staro")
  (load "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
  (staro-init)
  (setq *robot* *staro*)
  )
 ((equal (string-upcase *robot-name*) "ATLAS")
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
  (setq *robot-name* "atlas")
  (atlas-init-ex :view nil :set-reset-pose nil)
  (setq *robot* *atlas*)
  )
 ((equal (string-upcase *robot-name*) "SAMPLEROBOT")
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
  (setq *robot-name* "samplerobot")
  (samplerobot-init)
  (setq *robot* *sr*)
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
	joint-name joint-angle)
    (dotimes (x (length joint-names))
      (setq joint-name (elt joint-names x))
      (let ((hand-method (intern (string-upcase joint-name) *keyword-package*)))
	(when (find-method *robot* hand-method)
	  (if (subclassp (class (send *robot* hand-method)) rotational-joint)
	      (setq joint-angle (rad2deg (elt joint-angles x)))
	    (setq joint-angle (* 1000.0 (elt joint-angles x)))
	    )
	  (send *robot* hand-method :joint-angle joint-angle)))
      )
    (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    

    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (ros::publish "move_joint_interface/move" (instance std_msgs::Float32 :init :data 5))
    )
  )

(defun joint-trajectory-callback
  (msg)
  (let ((joint-names (send msg :joint_names))
	(joint-trajectory-points (send msg :points))
	joint-trajectory-point
	joint-angles
	joint-name joint-angle
	avs tms
	)

    (dolist (joint-trajectory-point joint-trajectory-points)
      (setq joint-angles (send joint-trajectory-point :positions))

      (dotimes (x (length joint-names))
	(setq joint-name (elt joint-names x))
	(let ((hand-method (intern (string-upcase joint-name) *keyword-package*)))
	  (when (find-method *robot* hand-method)
	    (if (subclassp (class (send *robot* hand-method)) rotational-joint)
		(setq joint-angle (rad2deg (elt joint-angles x)))
	      (setq joint-angle (* 1000.0 (elt joint-angles x)))
	      )
	    (send *robot* hand-method :joint-angle joint-angle)))
	)
      (push (send *robot* :angle-vector) avs)
      (let ((time-from-start (send (send joint-trajectory-point :time_from_start) :to-sec))
	    (time-prev 0))
	(push (* 1000 (- time-from-start time-prev)) tms)
	(setq time-prev time-from-start)
	)
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      )
    (setq avs (reverse avs))
    (setq tms (reverse tms))

    (send *ri* :angle-vector-sequence avs tms)
    (ros::publish "move_joint_interface/move" (instance std_msgs::Float32 :init :data 5))
    (send *ri* :wait-interpolation)
    )
  )

(defun apply-joint-angles
  (joint-names joint-angles)
  (dotimes (x (length joint-names))
    (let (joint-name joint-angle)
      (setq joint-name (elt joint-names x))
      (let ((hand-method (intern (string-upcase joint-name) *keyword-package*)))
	(when (find-method *robot* hand-method)
	  (if (subclassp (class (send *robot* hand-method)) rotational-joint)
	      (setq joint-angle (rad2deg (elt joint-angles x)))
	    (setq joint-angle (* 1000.0 (elt joint-angles x)))
	    )
	  (send *robot* hand-method :joint-angle joint-angle)))
      )
    )
  )


(defun joint-trajectory-with-type-callback
  (msg)
  (let ((joint-names (send msg :joint_names))
	(joint-trajectory-points (send msg :points))
	joint-angles
	avs tms
	(sentinel-type -1)
	sentinel-point
	)
    ;;add sentinel
    (setq sentinel-point (instance jsk_interactive_marker::JointTrajectoryPointWithType :init :type sentinel-type))
    (setq joint-trajectory-points (append joint-trajectory-points (list sentinel-point)))

    (dotimes (i (length joint-trajectory-points))
      (let* ((joint-trajectory-point (elt joint-trajectory-points i))
	     (joint-angles (send joint-trajectory-point :positions))
	     next-joint-trajectory-point
	     )
	(cond
	 ((equal (send joint-trajectory-point :type) sentinel-type)
	  (return-from joint-trajectory-with-type-callback)
	  )
	 ((equal (send joint-trajectory-point :type) jsk_interactive_marker::JointTrajectoryPointWithType::*JOINT_INTERPOLATION*)
	  (apply-joint-angles joint-names joint-angles)
	  (push (send *robot* :angle-vector) avs)
	  (let ((time-from-start (send (send joint-trajectory-point :time_from_start) :to-sec))
		(time-prev 0))
	    (push (* 1000 (- time-from-start time-prev)) tms)
	    (setq time-prev time-from-start)
	    )

	  (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

	  ;; send joint state trajectory

	  (setq next-joint-trajectory-point (elt joint-trajectory-points (+ i 1)))
	  (unless (equal (send next-joint-trajectory-point :type) jsk_interactive_marker::JointTrajectoryPointWithType::*JOINT_INTERPOLATION*)
	    (setq avs (reverse avs))
	    (setq tms (reverse tms))

	    (send *ri* :angle-vector-sequence avs tms)
	    (setq avs nil)
	    (ros::publish "move_joint_interface/move" (instance std_msgs::Float32 :init :data (/ (reduce #'+ tms) 1000.0)))
	    (send *ri* :wait-interpolation)
	    )
	  )
	 ((equal (send joint-trajectory-point :type) jsk_interactive_marker::JointTrajectoryPointWithType::*COLLISION_AVOIDANCE*)
	  ;;;;;;;;;;;;;;;;
	  ;;   moveit   ;;
	  ;;;;;;;;;;;;;;;;
	  (apply-joint-angles joint-names joint-angles)
	  ;; move only head
	  (send *ri* :angle-vector (send *robot* :angle-vector) (* (send (send joint-trajectory-point :time_from_start) :to-sec) 1000) :head-controller)

	  (send *ri* :angle-vector-motion-plan
	  	(send *robot* :angle-vector)
	  	:move-arm :arms :use-torso t)
	  (send *ri* :wait-interpolation)
	  
	  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
	  )
	 ;;stop grasp
	 ((equal (send joint-trajectory-point :type) jsk_interactive_marker::JointTrajectoryPointWithType::*CLOSE_HAND*)
	  (send* *ri* :start-grasp (read-from-string (send joint-trajectory-point :args)))
	  )
	 ;;start grasp
	 ((equal (send joint-trajectory-point :type) jsk_interactive_marker::JointTrajectoryPointWithType::*OPEN_HAND*)
	  (send* *ri* :stop-grasp (read-from-string (send joint-trajectory-point :args)))
	  )
	 )
	)
      )
    )
  )


(defun hand-callback
  (msg)
  (let ((hand-msg (send msg :data))
	move-hand move-type)
    ;; set move-hand
    (cond
     ((substringp "rarm" hand-msg)
      (setq move-hand :rarm)
      )
     ((substringp "larm" hand-msg)
      (setq move-hand :larm)
      )
     (t
      (setq move-hand :arms)
      )
     )

    ;; set move-type
    (cond
     ((substringp "start-grasp" hand-msg)
      (setq move-type :start-grasp)
      )
     ((substringp "stop-grasp" hand-msg)
      (setq move-type :stop-grasp)
      )
     (t
      (return-from hand-callback)
      )
     )

    (when (find-method *ri* move-type)
      (send *ri* move-type move-hand)
      )
    )
  )

(defun base-callback
  (msg)
  ;;do callback
  (let ((move-coords (ros::tf-pose-stamped->coords msg)))
    (send *ri* :move-to move-coords :frame-id (send move-coords :name))
    )
  )


(defun get-joint-state-srv
  (req)
  (setq req-tmp req)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (let ((joint-list (send *robot* :joint-list))
	(res (send req :response))
	(joint-state-msg
	 (instance sensor_msgs::JointState :init
		   :header (instance std_msgs::header :init
				     :stamp (ros::time-now))))
	(joint-angles (send *robot* :angle-vector)))
    (send joint-state-msg :position
	  (mapcar #'(lambda (joint)
		      (if (subclassp (class joint) rotational-joint)
			  (deg2rad (send joint :joint-angle))
			(/ (send joint :joint-angle) 1000.0)))
		  joint-list))
    (send joint-state-msg :name (send-all joint-list :name))
    (send res :joint_state joint-state-msg)
    res
    ))


(setq server-nodename "jsk_model_marker_interface")

;;(ros::advertise (format nil "~A/~A/joint_states" server-nodename *robot-topic-name*)
;;		sensor_msgs::JointState)

;;(ros::subscribe (format nil "~A/~A/send_joint_states" server-nodename *robot-topic-name*)
;;		std_msgs::Empty #'send-joint-state-callback)
(ros::advertise-service (format nil "~A/~A/get_joint_states" server-nodename *robot-topic-name*)
			jsk_interactive_marker::GetJointState
			#'get-joint-state-srv)

(ros::advertise "move_joint_interface/move" std_msgs::Float32)

(ros::subscribe (format nil "~A/~A/joint_states_ri_move" server-nodename *robot-topic-name*)
		sensor_msgs::JointState #'joint-state-callback)

(ros::subscribe (format nil "~A/~A/joint_trajectory_ri_move" server-nodename *robot-topic-name*)
		trajectory_msgs::JointTrajectory #'joint-trajectory-callback)

(ros::subscribe (format nil "~A/~A/joint_trajectory_with_type_ri_move" server-nodename *robot-topic-name*)
		jsk_interactive_marker::JointTrajectoryWithType #'joint-trajectory-with-type-callback)

(ros::subscribe (format nil "~A/~A/hand_ri_move" server-nodename *robot-topic-name*)
		std_msgs::String #'hand-callback)

(ros::subscribe (format nil "~A/~A/base_move" server-nodename *robot-topic-name*)
		geometry_msgs::PoseStamped #'base-callback)

(ros::spin)
