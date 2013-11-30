#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")
(load "package://roseus/euslisp/actionlib.l")
(load "models/arrow-object.l")

(defclass robot-im
  :slots
  (ri robot
      auto-look
      planned-angle-vector
      head-target-coords
      base-target-coords
      origin-coords target-coords old-coords
      moving-arm ik-rotation-axis constraint-move
      use-torso use-fullbody
      server-nodename
      visualization-nodename
      visualization-robot
      base-frame
      marker-sequence
      r-finger-rpy l-finger-rpy
      ))


(defmethod robot-im
  (:init
   (&key ((:robot r)) ((:ri i)) ((:base-frame bf) "/base_link")
	 ((:server-nodename sn) "/jsk_interactive_marker_manipulation")
	 ((:urdf-model-nodename umn) "/jsk_model_marker_interface")
	 ((:visualization-nodename vn) "/jsk_ik_visualization")
	 ((:visualization-robot vr) "/atlas_visualization")
	 )
   (setq robot r)
   (setq ri i)
   (setq base-frame bf)
   (setq server-nodename sn)
   (setq urdf-model-nodename umn)
   (setq visualization-nodename vn)
   (setq visualization-robot vr)

   (ros::roseus sn)
   (setq *tfb* (instance ros::transform-broadcaster :init))
   (setq *tfl* (instance ros::transform-listener :init))
   
   ;;check if tf is published
   (send self :get-base-coords)

   (ros::advertise (format nil "~A/marker_list" server-nodename)
		   visualization_msgs::MarkerArray)

   (ros::advertise (format nil "~A/marker" server-nodename)
		   visualization_msgs::Marker)

   (ros::advertise (format nil "~A/origin_marker" server-nodename)
		   visualization_msgs::Marker)

   (ros::subscribe (format nil "~A/pose" server-nodename)
		   jsk_interactive_marker::MarkerPose #'send self :pose-callback)

   (ros::subscribe (format nil "~A/marker_menu" server-nodename)
		   jsk_interactive_marker::MarkerMenu #'send self :move-callback)

   (ros::subscribe (format nil "~A/move_object" urdf-model-nodename)
		   jsk_interactive_marker::MoveObject #'send self :model-move-object-callback)

   ;;3dmouse move
   (ros::subscribe (format nil "~A/spacenav/joy" server-nodename)
		   sensor_msgs::Joy #'send self :joy-callback)

   (ros::advertise (format nil "~A/~A/reset_joint_states" visualization-nodename visualization-robot)
		   sensor_msgs::JointState)


   ;;urdf marker move
   #|
   (ros::subscribe (format nil "~A/~A/joint_states" urdf-model-nodename (send robot :name))
   sensor_msgs::JointState #'send self :joint-state-callback)

   (ros::subscribe (format nil "~A/marker_menu" urdf-model-nodename)
   jsk_interactive_marker::MarkerMenu #'send self :urdf-menu-callback)

   (ros::advertise (format nil "~A/~A/reset_joint_states" urdf-model-nodename (send robot :name))
   sensor_msgs::JointState)
   |#



   #|
   (ros::advertise (format nil "~A/force_marker" server-nodename)
   visualization_msgs::Marker)

   (ros::advertise (format nil "~A/force_marker_strength" server-nodename)
   visualization_msgs::Marker)
   (ros::advertise (format nil "~A/force_marker_list" server-nodename)
   visualization_msgs::MarkerArray)
   |#

   (setq moving-arm :rarm)
   (setq ik-rotation-axis t)
   (setq auto-look t)
   (setq use-torso t)
   (setq use-fullbody nil)
   
   ;;initialize with invalid value
   (setq r-finger-rpy (list 10 0 0))
   (setq l-finger-rpy (list 10 0 0))

   (setq target-coords (instance arrow-object :init))
   (send target-coords :translate #f(500 0 0)) ;; initial pose
   (setq origin-coords (instance arrow-object :init)) ;;arrow for origin of object
   (send origin-coords :translate #f(500 0 0))
   (setq old-coords (send target-coords :copy-worldcoords))

   ;;(setq head-target-coords (instance arrow-object :init))
   (setq head-target-coords (make-icosahedron 70))
   (send head-target-coords :translate #f(500 0 0)) ;; initial pose

   (setq base-target-coords (instance arrow-object :init))
   (send base-target-coords :translate #f(0 0 0)) ;; initial pose


   (when (boundp '*irtviewer*) (objects (list robot target-coords origin-coords head-target-coords base-target-coords)))
   (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
   (send self :set-marker-pose-robot (send target-coords :worldcoords))

   (send robot :angle-vector (send ri :state :potentio-vector))
   (if (not (boundp 'marker-sequence))(setq marker-sequence nil))
   )

  (:main-loop
   ()
   (send self :set-origin)
   (ros::rate 30)

   (send self :publish-target-arrow-obj)
   (send self :publish-origin-arrow-obj)

   (ros::ros-info "finish initializing and start loop")
   (do-until-key
    ;;(send self :publish-target-arrow-tf)
    ;;(send self :publish-origin-arrow-tf)

    (ros::spin-once)
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::sleep)
    (unless (ros::ok) (return)))
   )

  (:set-origin
   ()
   (let (handle-r handle-l)
     (send robot :angle-vector (send ri :state :potentio-vector))
     (setq handle-r (make-cascoords :coords (send target-coords :transformation (send robot :rarm :end-coords))))
     (setq handle-l (make-cascoords :coords (send target-coords :transformation (send robot :larm :end-coords))))
     (setq old-coords (send target-coords :copy-worldcoords))
     (send origin-coords :move-to target-coords :world)
     )
   )

  (:get-base-coords
   ()
   (send robot :worldcoords)
   )

  (:solve-ik
   (target-coords &key (move-arm :rarm) (rotation-axis t) (debug-view t) (use-torso nil) (use-fullbody nil))
   (cond
    ((equal move-arm :arms)
     (unless
	 (send robot :rarm
	       :inverse-kinematics (car target-coords)
	       :rotation-axis rotation-axis :debug-view debug-view)
       (return-from :solve-ik nil)
       )
     (send robot :larm
	   :inverse-kinematics (cadr target-coords)
	   :rotation-axis rotation-axis :debug-view debug-view)
     )

    (t
     (send robot move-arm
	   :inverse-kinematics target-coords
	   :rotation-axis rotation-axis :debug-view debug-view)
     )))

  (:pose-move
   (&key (mem t) (move t))
   (let ((inter-num 10)(vector-list nil)
	 handle-r handle-l hadle-r-world handle-l-world
	 new-coords reset-coords old-coords add-coords)
     ;;reset-coords add-coords)

     ;;モデルへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     (setq handle-r (make-cascoords :coords (send origin-coords :transformation (send robot :rarm :end-coords))))
     (setq handle-l (make-cascoords :coords (send origin-coords :transformation (send robot :larm :end-coords))))

     (setq new-coords (send target-coords :copy-worldcoords))
     (setq reset-coords (send origin-coords :copy-worldcoords))
     ;;     (setq reset-coords (send old-coords :copy-worldcoords))
     (setq old-coords (send origin-coords :copy-worldcoords))

     ;; (when auto-look
     ;;   (send self :look-at (send old-coords :worldpos) :wait t)
     ;;   (send head-target-coords :locate (send old-coords :worldpos) :world)
     ;;   )

     ;;inter-num で分割して送る
     (block interpolation
       (dotimes (i inter-num)
	 (setq add-coords (instance arrow-object :init))
	 (setq handle-r-world (send handle-r :copy-worldcoords))
	 (setq handle-l-world (send handle-l :copy-worldcoords))
	 (send add-coords :move-to (midcoords (/ (+ i 1) (float inter-num)) old-coords new-coords))

	 (send handle-r-world :transform add-coords :world)
	 (send handle-l-world :transform add-coords :world)

	 (case moving-arm
	   (:rarm
	    (unless
		(send self :solve-ik handle-r-world :move-arm :rarm
		      :rotation-axis ik-rotation-axis
		      :use-torso use-torso :use-fullbody use-fullbody)
	      ;;(send target-coords :move-to reset-coords :world)
	      ;;(send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))

	   (:larm
	    (unless
		(send self :solve-ik handle-l-world :move-arm :larm
		      :rotation-axis ik-rotation-axis
		      :use-torso use-torso :use-fullbody use-fullbody)
	      ;;(send target-coords :move-to reset-coords :world)
	      ;;(send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))
	   (t
	    (unless
		(send self :solve-ik (list handle-r-world handle-l-world) :move-arm :arms
		      :rotation-axis ik-rotation-axis 
		      :use-torso use-torso :use-fullbody use-fullbody)
	      ;;(send target-coords :move-to reset-coords :world)
	      ;;(send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      )
	    )
	   )

	 (when auto-look
	   (send robot :head :look-at
		 (send add-coords :worldpos))
	   (send head-target-coords :locate (send add-coords :worldpos) :world)
	   )
	 (push (send robot :angle-vector) vector-list)
	 (when (boundp '*irtviewer*) (objects (list robot origin-coords add-coords)))
	 (print add-coords)
	 (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
	 (setq reset-coord (send add-coords :copy-worldcoords))
	 )
       )
     (when (boundp '*irtviewer*) (objects (list robot origin-coords target-coords head-target-coords base-target-coords)))
     
     (print "exe")
     (setq vector-list (reverse vector-list))
     
     ;;just plan
     (unless move
       (print "finished planning")
       (setq planned-angle-vector vector-list)
       (send self :publish-joint-states)
       (return-from :pose-move)
       )
     (setq planned-angle-vector nil)
     (unless (null vector-list)
       (print "sending vector list...")
       (setq tmp-vec vector-list)

       (send ri :angle-vector-sequence vector-list (make-sequence 'list (length vector-list) :initial-element 500))
       (send ri :wait-interpolation)
       (print "done")
       )

     ;;robotへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     
     (setq renew-arrow-coord-r (send (send robot :rarm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-r :transform (send handle-r :inverse-transformation))
     (setq renew-arrow-coord-l (send (send robot :larm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-l :transform (send handle-l :inverse-transformation))

     (case moving-arm
       (:rarm
	(send target-coords :move-to renew-arrow-coord-r :world)
	)
       (:larm
	(send target-coords :move-to renew-arrow-coord-l :world)
	)
       (t
	(send target-coords :move-to (midcoords 0.5 renew-arrow-coord-r renew-arrow-coord-l) :world)
	))
     (setq old-coords (send target-coords :copy-worldcoords))
     (send origin-coords :move-to target-coords :world)

     (send self :set-marker-pose-robot (send target-coords :worldcoords))
     (send self :set-origin)
     )
   )

  (:planned-move
   nil
   (let (handle-r handle-l hadle-r-world handle-l-world
		  new-coords reset-coords old-coords add-coords)

     (if (null planned-angle-vector)
	 (return-from :planned-move nil)
       )
     (print "sending vector list...")
     (send robot :angle-vector (send ri :state :potentio-vector))
     
     (setq handle-r (make-cascoords :coords (send origin-coords :transformation (send robot :rarm :end-coords))))
     (setq handle-l (make-cascoords :coords (send origin-coords :transformation (send robot :larm :end-coords))))
     (send ri :angle-vector-sequence planned-angle-vector (make-sequence 'list (length planned-angle-vector) :initial-element 500))
     (send ri :wait-interpolation)
     (print "done")

     ;;robotへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     
     (setq renew-arrow-coord-r (send (send robot :rarm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-r :transform (send handle-r :inverse-transformation))
     (setq renew-arrow-coord-l (send (send robot :larm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-l :transform (send handle-l :inverse-transformation))
     
     (case moving-arm
       (:rarm
	(send target-coords :move-to renew-arrow-coord-r :world)
	)
       (:larm
	(send target-coords :move-to renew-arrow-coord-l :world)
	)
       (t
	(send target-coords :move-to (midcoords 0.5 renew-arrow-coord-r renew-arrow-coord-l) :world)
	))
     (setq old-coords (send target-coords :copy-worldcoords))
     (send origin-coords :move-to target-coords :world)

     (send self :set-marker-pose-robot (send target-coords :worldcoords))
     (send self :set-origin)
     )
   )
  
  
  (:publish-joint-states
   nil
   (let ((joint-angles nil)
	 (joint-names nil)
	 (joint-list (send robot :joint-list))
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

     (ros::publish (format nil "~A/~A/reset_joint_states" visualization-nodename visualization-robot)
		   joint-state-msg)
     )
   )

  
  (:pose-move-object
   (&optional (target-cds target-coords)
	      (origin-cds origin-coords)
	      &key (mem t) (arm :arms))
   (let ((inter-num 10)(vector-list nil)
	 handle-r handle-l hadle-r-world handle-l-world
	 new-coords reset-coords old-coords add-coords)

     ;;モデルへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     (setq handle-r (make-cascoords :coords (send origin-cds :transformation (send robot :rarm :end-coords))))
     (setq handle-l (make-cascoords :coords (send origin-cds :transformation (send robot :larm :end-coords))))

     (setq new-coords (send target-cds :copy-worldcoords))
     (setq reset-coords (send origin-cds :copy-worldcoords))
     ;;     (setq reset-coords (send old-coords :copy-worldcoords))
     (setq old-coords (send origin-cds :copy-worldcoords))

     ;;interpolation
     (block interpolation
       (dotimes (i inter-num)
	 (setq add-coords (instance arrow-object :init))
	 (setq handle-r-world (send handle-r :copy-worldcoords))
	 (setq handle-l-world (send handle-l :copy-worldcoords))
	 (send add-coords :move-to (midcoords (/ (+ i 1) (float inter-num)) old-coords new-coords))

	 (send handle-r-world :transform add-coords :world)
	 (send handle-l-world :transform add-coords :world)

	 (case arm
	   (:rarm
	    (unless
		(send self :solve-ik handle-r-world :move-arm :rarm
		      :rotation-axis ik-rotation-axis 
		      :use-torso use-torso :use-fullbody use-fullbody)
	      (send target-cds :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))

	   (:larm
	    (unless
		(send self :solve-ik handle-l-world :move-arm :larm
		      :rotation-axis ik-rotation-axis 
		      :use-torso use-torso :use-fullbody use-fullbody)
	      (send target-cds :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))
	   (t
	    (unless
		(send self :solve-ik (list handle-r-world handle-l-world) :move-arm :arms
		      :rotation-axis ik-rotation-axis
		      :use-torso use-torso :use-fullbody use-fullbody)
	      (send target-cds :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      )
	    )
	   )

	 ;;	 (when auto-look
	 (when t
	   (send robot :head :look-at
		 (send add-coords :worldpos))
	   (send head-target-coords :locate (send add-coords :worldpos) :world)
	   )
	 (push (send robot :angle-vector) vector-list)
	 (when (boundp '*irtviewer*) (objects (list robot origin-cds add-coords)))
	 (print add-coords)
	 (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
	 (setq reset-coord (send add-coords :copy-worldcoords))
	 )
       )
     (when (boundp '*irtviewer*) (objects (list robot origin-cds target-cds head-target-coords base-target-coords)))
     
     (print "exe")
     (setq vector-list (reverse vector-list))

     (unless (null vector-list)
       (print "sending vector list...")
       (setq tmp-vec vector-list)
       (send ri :angle-vector-sequence vector-list (make-sequence 'list (length vector-list) :initial-element 500))
       (send ri :wait-interpolation)
       (print "done")
       )

     ;;robotへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     
     (setq renew-arrow-coord-r (send (send robot :rarm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-r :transform (send handle-r :inverse-transformation))
     (setq renew-arrow-coord-l (send (send robot :larm :end-coords) :copy-worldcoords))
     (send renew-arrow-coord-l :transform (send handle-l :inverse-transformation))

     (case moving-arm
       (:rarm
	(send target-cds :move-to renew-arrow-coord-r :world)
	)
       (:larm
	(send target-cds :move-to renew-arrow-coord-l :world)
	)
       (t
	(send target-cds :move-to (midcoords 0.5 renew-arrow-coord-r renew-arrow-coord-l) :world)
	))
     (setq old-coords (send target-cds :copy-worldcoords))
     (send origin-cds :move-to target-cds :world)

     (send self :set-marker-pose-robot (send target-cds :worldcoords))
     (send self :set-origin)
     )
   )

  (:set-origin-to-hand
   (&optional (arm :rarm))
   (send robot :angle-vector (send ri :state :potentio-vector))
   (send target-coords :move-to (send robot arm :end-coords :worldcoords) :world)
   (send self :set-origin)
   (send self :set-marker-pose-robot (send target-coords :worldcoords))
   )

  (:start-grasp
   (&optional (arm :rarm))
   (send ri :start-grasp moving-arm))

  (:stop-grasp
   (&optional (arm :rarm))
   (send ri :stop-grasp moving-arm))

  (:harf-grasp
   (&optional (arm :rarm))
   )


  (:look-at
   (coords &key (wait t))
   (send robot :angle-vector (send ri :state :potentio-vector))
   (send robot :head :look-at coords)
   (send ri :angle-vector (send robot :angle-vector) 5000)
   (if wait
       (send ri :wait-interpolation))
   )

  (:move-to
   (coords)
   (send ri :move-to coords))

  (:move-callback
   ( msg )
   (setq a (send msg :menu))
   (print a)
   (setq b msg)
   (let ((menu (send msg :menu))
	 (type (send msg :type)))
     (cond
      ;; BASE MARKER
      ((eq type jsk_interactive_marker::MarkerMenu::*BASE_MARKER*)
       (cond
	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
	 (print "base move")
	 (send self :move-to (send base-target-coords :worldcoords))
	 (send self :set-marker-pose-robot (make-coords) :marker-name (send msg :marker_name))
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
	 (print "reset base marker")
	 (send self :set-marker-pose-robot (make-coords) :marker-name (send msg :marker_name))
	 )
	)
       )

      ;; HEAD MARKER
      ((eq type jsk_interactive_marker::MarkerMenu::*HEAD_MARKER*)
       (cond
	((eq menu jsk_interactive_marker::MarkerMenu::*LOOK_AUTO*)
	 (print "look auto")
	 (setq auto-look t)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*HEAD_TARGET_POINT*)
	 (setq auto-look nil)
	 (print "look target point")
	 (send self :set-marker-pose-robot (send head-target-coords :worldcoords) :marker-name "head_point_goal")
	 )
	)
       )

      ;; FINGER MARKER
      ((eq type jsk_interactive_marker::MarkerMenu::*RFINGER_MARKER*)
       (cond
	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
	 (print "update finger rarm")
	 (send self :update-finger :rarm)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
	 (print "reset finger marker")
	 (send self :reset-finger :rarm)
	 )
	)
       )

      ((eq type jsk_interactive_marker::MarkerMenu::*LFINGER_MARKER*)
       (cond
	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
	 (print "update finger larm")
	 (send self :update-finger :larm)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
	 (print "reset finger marker")
	 (send self :reset-finger :larm)
	 )
	)
       )


      (t
       (cond
	((eq menu jsk_interactive_marker::MarkerMenu::*START_GRASP*)
	 (print "start grasp")
	 (send self :start-grasp moving-arm))

	((eq menu jsk_interactive_marker::MarkerMenu::*HARF_GRASP*)
	 (print "harf grasp")
	 (send self :harf-grasp moving-arm))

	((eq menu jsk_interactive_marker::MarkerMenu::*STOP_GRASP*)
	 (print "stop grasp")
	 (send self :stop-grasp moving-arm))

	;;pick up
	((eq menu jsk_interactive_marker::MarkerMenu::*PICK*)
	 (print "under construction"))

	((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN*)
	 (send self :set-origin)
	 (send self :set-hand-marker-pose))

	;;reset target coords
	((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
	 (send target-coords :move-to old-coords :world)
	 (send self :set-marker-pose-robot (send target-coords :worldcoords))
	 (send self :set-hand-marker-pose)
	 )

	;;set origin to hand
	((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_RHAND*)
	 (print "set origin to rhand")
	 (send self :set-origin-to-hand :rarm)
	 (send self :set-hand-marker-pose))
	((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_LHAND*)
	 (print "set origin to lhand")
	 (send self :set-origin-to-hand :larm)
	 (send self :set-hand-marker-pose))
	
	;;change ik-rotation-axis
	((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_T*)
	 (print "set ik-rotation-axis t")
	 (setq ik-rotation-axis t)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_NIL*)
	 (print "set ik-rotation-axis nil")
	 (setq ik-rotation-axis nil)
	 )

	;;change use-torso
	((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_T*)
	 (print "use torso t")
	 (setq use-torso t)
	 (setq use-fullbody nil)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_NIL*)
	 (print "use torso nil")
	 (setq use-torso nil)
	 (setq use-fullbody nil)
	 )

	((eq menu jsk_interactive_marker::MarkerMenu::*USE_FULLBODY*)
	 (print "use torso nil")
	 (setq use-fullbody t)
	 )
	#| don't use now
	(4 (send *force-arrow* :scale :x 1)
	(send *force-arrow* :scale :y 1)
	(send *force-arrow* :scale :z 0)
	)
	(5
	(setq *force-flag* nil)
	(set-marker-pose target-coords)
	)
	(6
	(setq *force-flag* t)
	(set-marker-pose *force-coord*)
	(send *force-arrow* :scale :x 1)
	(send *force-arrow* :scale :y 1)
	(send *force-arrow* :scale :z 1)
	)
	|#

	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
	 (send self :pose-move))

	((eq menu jsk_interactive_marker::MarkerMenu::*PLAN_EXECUTE*)
	 (send self :pose-move))

	((eq menu jsk_interactive_marker::MarkerMenu::*PLAN*)
	 (send self :pose-move :move nil))

	((eq menu jsk_interactive_marker::MarkerMenu::*EXECUTE*)
	 (send self :planned-move)
	 )

	((eq menu jsk_interactive_marker::MarkerMenu::*CANCEL_PLAN*)
	 (setq planned-angle-vector nil)
	 (send robot :angle-vector (send ri :state :potentio-vector))
	 (send self :publish-joint-states)
	 )



	;;force-move
	;;      (7
	;;       (marker-teaching)
	;;       (force-move)
	;;       )
	;;      (9
	;;       ;;	    (reset-teaching)
	;;       (reset-marker-sequence)
	;;       )
	;;      (8
	;;       (setq *reproduce-mode* t)
	;;       ;;	    (teached-move)
	;;       (set-interactive-markers)
	;;       )

	#|
	(12
	(setq *force-flag* t)
	(set-marker-pose *force-coord*)
	)
	(13
	(setq *force-flag* nil)
	(set-marker-pose target-coords)
	)
	|#

	;;changeControllArm
	((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_RARM*)
	 (print "move rarm")
	 (setq moving-arm :rarm)
	 (send self :set-hand-marker-pose)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_LARM*)
	 (print "move larm")
	 (setq moving-arm :larm)
	 (send self :set-hand-marker-pose)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_ARMS*)
	 (print "move arms")
	 (setq moving-arm :arms)
	 (send self :set-hand-marker-pose)
	 )
	
	;;change constraint
	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE_CONSTRAINT_T*)
	 (setq constraint-move t)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*MOVE_CONSTRAINT_NIL*)
	 (setq constraint-move nil)
	 )
	((eq menu jsk_interactive_marker::MarkerMenu::*PUBLISH_MARKER*)
	 (send self :publish-target-arrow-obj)
	 (send self :publish-origin-arrow-obj)
	 )
	)
       )
      )
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
     )
   (send self :publish-target-arrow-tf)
   (send self :publish-origin-arrow-tf)
   
   )


  (:urdf-menu-callback
   ( msg )
   (let ((menu (send msg :menu)))
     (cond
      ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
       (if (y-or-n-p)
	   (send ri :angle-vector (send robot :angle-vector) 5000)
	 (warn "canceled~%")
	 ))
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
       (real2model)
       (let ((joint-angles nil)
	     (joint-names nil)
	     (joint-list (send robot :joint-list))
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

	 (ros::publish (format nil "~A/~A/reset_joint_states" server-nodename (send robot :name))
		       joint-state-msg)
	 ))

      )
     )
   )

  (:joint-state-callback
   (msg)
   (let ((joint-names (send msg :name))
	 (joint-angles (send msg :position))
	 joint-name joint-angle
	 )
     (dotimes (x (length joint-names))
       (setq joint-name (elt joint-names x))
       (setq joint-angle (rad2deg (elt joint-angles x)))
       (when (find-method robot (intern (string-upcase joint-name) *keyword-package*))
	 (send robot (intern (string-upcase joint-name) *keyword-package*)
	       :joint-angle joint-angle)
	 )
       )
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
     )
   )


  (:set-hand-marker-pose 
   ()
   )

  (:close-finger
   (arm rad)
   (print rad)
   )

  (:update-finger
   (arm)
   )

  (:reset-finger
   (arm)
   )

  (:finger-callback
   (cds arm)
   (let* ((rpy (elt (rpy-angle (send cds :rot)) 0))
	  (r (elt rpy 0))
	  (p (elt rpy 1))
	  old-rpy
	  old-r old-p
	  )
     (cond
      ((equal arm :rarm)
       (setq old-rpy r-finger-rpy)
       )
      ((equal arm :larm)
       (setq old-rpy l-finger-rpy)
       )
      )
     (setq old-r (elt old-rpy 0))
     (setq old-p (elt old-rpy 1))
     
     ;;(print rpy)
     (when (< (abs (- old-r r)) 1.0e-6)
       (cond
	((> r 3)
	 (send self :close-finger arm (- p old-p))
	 )

	((> r -3)
	 (send self :close-finger arm (- (- p old-p)))
	 )

	((< r -3)
	 (send self :close-finger arm (- p old-p))
	 )

	))
     (cond
      ((equal arm :rarm)
       (setq r-finger-rpy rpy)
       )
      ((equal arm :larm)
       (setq l-finger-rpy rpy)
       )
      ))
   )

  (:pose-callback
   ( msg )
   (setq a msg)
   (print (list "callback" msg))
   (let* ((pose (send msg :pose))
	  (cds (ros::tf-pose-stamped->coords pose))
	  )
     (cond
      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*BASE_MARKER*)
       (send base-target-coords :move-to (send (send self :get-base-coords) :worldcoords) :world)
       (send base-target-coords :transform cds)
       ;;(send self :look-at (send head-target-coords :worldpos))
       )
      ;;rfinger
      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*RFINGER_MARKER*)
       (send self :finger-callback cds :rarm)
       )

      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*LFINGER_MARKER*)
       (send self :finger-callback cds :larm)
       )

      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*HEAD_MARKER*)
       ;;((equal (send msg :marker_name) "head_point_goal")
       (send head-target-coords :move-to (send (send self :get-base-coords) :worldcoords) :world)
       (send head-target-coords :transform cds)
       (send self :look-at (send head-target-coords :worldpos))
       )

      (t
       (send target-coords :move-to (send (send self :get-base-coords) :worldcoords) :world)
       (send target-coords :transform cds)
       (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
       (send self :set-hand-marker-pose)
       )
      ))
   (send self :publish-target-arrow-tf)
   (send self :publish-origin-arrow-tf)

   )
  
  (:joy-callback
   (msg)
   (let (axes coords pos rpy pos-scale rpy-scale rotate-origin-coords 
	      (min-pos-thre 1.0e-6)
	      (min-rpy-thre 1.0e-6))
     (setq pos-scale 100)
     (setq rpy-scale 0.2)
     (setq axes (send msg :axes))
     (setq pos (scale pos-scale (subseq axes 0 3)))
     (setq rpy (scale rpy-scale (float-vector (elt axes 5) (elt axes 4) (elt axes 3))))
     (if (and (< (norm (subseq axes 0 3)) min-pos-thre)
	      (< (norm (subseq axes 4 6)) min-rpy-thre)
	      )
	 (return-from :joy-callback))
     (send target-coords :translate pos :world)
     (setq coords (make-coords :rpy rpy))
     (setq rotate-origin-coords (make-coords :pos (send target-coords :worldpos)))

     (send target-coords :transform coords rotate-origin-coords)
     (send self :set-marker-pose-robot target-coords)
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
     ;;(print target-coords)
     )
   )

  (:model-pose-callback
   ( msg )
   (setq a msg)
   (print (list "callback" msg))

   (let* ((pose (send msg :pose))
	  (cds (ros::tf-pose-stamped->coords pose))
	  )
     (print cds)
     #|
     (send target-coords :move-to (send (send self :get-base-coords) :worldcoords) :world)
     (send target-coords :transform cds)
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
     (send self :set-hand-marker-pose)
     |#
     )
   )

  (:model-move-object-callback
   (msg)
   (let ((origin (ros::tf-pose-stamped->coords (send msg :origin)))
	 (goal (ros::tf-pose-stamped->coords (send msg :goal)))
	 (grasp (ros::tf-pose->coords (send msg :graspPose))))
     (setq tmp-msg msg)
     (print (list "callback" msg))
     (print (list "callback" origin))
     (print (list "callback" goal))
     (setq a origin)
     (send self :model-move-object msg)
     #|
     
     (print (list "test" (send origin :worldcoords)))

     (setq base->origin (send *tfl* :lookup-transform "base_link" (send origin :name)  (ros::time 0)))
     (send origin :transform base->origin :world)
     (send goal :transform base->origin :world)
     (send grasp :transform origin :world)
     (setq a origin)
     ;;(send self :pose-move origin goal)
     
     ;;grasp

     ;;   (send self :solve-ik (list handle-r-world handle-l-world) :move-arm :arms
     ;;	 :rotation-axis ik-rotation-axis :use-torso use-torso)
     (send self :solve-ik grasp :move-arm :rarm :rotation-axis nil)
     (send ri :angle-vector (send robot :angle-vector) 5000)
     (send ri :wait-interpolation)

     (setq b grasp)

     ;;grasp

     ;;:rotation-axis ik-rotation-axis :use-torso use-torso)

     (send self :pose-move-object origin goal :arm :rarm)
     |#
     )
   )

  
  (:set-marker-pose-robot
   (coords &rest args)
   (if args
       (send* self :set-marker-pose coords args) 
     (send self :set-marker-pose coords) 
     ))

  (:set-marker-pose 
   ( coords &key (marker-name) (frame base-frame) (markers) (server server-nodename))
   (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker-name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_pose" server)
			req)
     ))
  

  (:set-markers 
   ( coords &key (frame base-frame) (markers) (server server-nodename) (marker_name 0))
   (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker_name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_markers" server)
			req)
     ))
  
  (:del-markers 
   ( coords &key (frame base-frame) (markers) (server server-nodename) (marker_name 0))
   (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker_name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/del_markers" server)
			req)
     ))

  (:reset-marker-pose 
   ( &key (server server-nodename))
   (let ((req (instance jsk_interactive_marker::SetPoseRequest :init)))
     (ros::service-call (format nil "~A/reset_pose" server)
			req)
     ))

  (:publish-target-arrow-tf
   ()
   (send *tfb* :send-transform (send (send self :get-base-coords) :transformation target-coords :local)
	 base-frame "/im_target_coords_arrow"))

  (:publish-target-arrow-obj
   ()
   (send self :publish-target-arrow-tf)
   (let ((org  (send target-coords :copy-worldcoords)))
     (send target-coords :reset-coords)
     (send target-coords :worldcoords)
     (send-all (send target-coords :bodies) :worldcoords)
     (ros::publish (format nil "~A/marker" server-nodename)
		   (object->marker-msg target-coords
				       (instance std_msgs::header :init
						 :frame_id "/im_target_coords_arrow")))
     (send target-coords :transform org)
     (send target-coords :worldcoords)
     )
   )

  (:publish-origin-arrow-tf
   ()
   (send *tfb* :send-transform (send (send self :get-base-coords) :transformation origin-coords :local)
	 base-frame "/im_origin_coords_arrow"))

  (:publish-origin-arrow-obj
   ()
   (send self :publish-origin-arrow-tf)
   (let ((org  (send origin-coords :copy-worldcoords)))
     (send origin-coords :reset-coords)
     (send origin-coords :worldcoords)
     (send-all (send origin-coords :bodies) :worldcoords)
     (ros::publish (format nil "~A/origin_marker" server-nodename)
		   (object->marker-msg origin-coords
				       (instance std_msgs::header :init
						 :frame_id "/im_origin_coords_arrow")))
     (send origin-coords :transform org)
     (send origin-coords :worldcoords)
     )
   )

  

  (:set-interactive-markers 
   ()
   (dump-loadable-structure "marker-seq-new.l" marker-sequence)
   (setq marker-sequence (reverse marker-sequence))
   (marker-memory :pose-move)
   (if (> (length marker-sequence) 1)
       (send (elt (elt marker-sequence 1) 2) :locate (send (elt (elt marker-sequence 0) 0) :worldpos ) :world))
   ;;移動過程をassoc
   (setq loop-num 0)
   (dolist (move-list marker-sequence)
     (setq loop-num (+ 1 loop-num))
     (if (= loop-num (length marker-sequence)) (return))
     (send (elt move-list 0) :assoc (elt (elt marker-sequence loop-num) 0))
     (send (elt (elt marker-sequence loop-num) 2) :locate (send (elt (elt marker-sequence (- loop-num 1)) 0) :worldpos) :world)
     )

   ;; (dolist (move-list marker-sequence)
   ;;   (setq loop-marker-name (+ 1 loop-marker-name))
   ;;   (if (= loop-marker-name (length marker-sequence)) (return))
   ;;   (set-markers (elt (elt marker-sequence loop-marker-name) 0) :marker_name loop-marker-name)
   ;;   ;;force-arrowをマーカーのところに動かす
   ;;   )

   (setq marker-list nil)
   (setq marker_name 0)
   ;;delete normal marker
   (send self :del-markers (make-coords) :marker_name 0)
   (dolist (move-list marker-sequence)
     (send self :set-markers (elt move-list 0) :marker_name marker_name)
     (print marker_name)
     (setq marker_name (+ 1 marker_name))
     )
   )
  )

