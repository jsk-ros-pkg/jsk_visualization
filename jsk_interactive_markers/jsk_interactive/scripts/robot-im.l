#!/usr/bin/env roseus

(setq *server-nodename* "atlas_manipulation_interactive_marker")

(ros::load-ros-manifest "atlas_interactive")
(load "package://roseus/euslisp/actionlib.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://jsk_interactive/scripts/task3.l")
(load "models/arrow-object.l")

(setq marker-sequence nil)

(defclass robot-im
  :slots
  (ri robot
      origin-coords target-coords old-coords
      moving-arm ik-rotation-axis constraint-move
      server-nodename
      base-frame
      ))


(defmethod robot-im
  (:init
   (&key ((:robot r)) ((:ri i)) ((:base-frame bf) "/base_link")
	 ((:server-nodename sn) "/interactive_marker_manipulation"))
   (setq robot r)
   (setq ri i)
   (setq base-frame bf)
   (setq server-nodename sn)

   (ros::roseus "read_marker_sample")
   (setq *tfb* (instance ros::transform-broadcaster :init))

   (ros::advertise (format nil "~A/marker_list" server-nodename)
		   visualization_msgs::MarkerArray)

   (ros::advertise (format nil "~A/marker" server-nodename)
		   visualization_msgs::Marker)

   (ros::advertise (format nil "~A/origin_marker" server-nodename)
		   visualization_msgs::Marker)

   (ros::subscribe (format nil "~A/pose" server-nodename)
		   atlas_interactive_marker::MarkerPose #'send self :pose-callback)

   (ros::subscribe (format nil "~A/move_flag" server-nodename)
		   atlas_interactive_marker::MarkerMenu #'send self :move-callback)

   ;;3dmouse move
   (ros::subscribe "/spacenav/joy"
		   sensor_msgs::Joy #'send self :joy-callback)

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

   (setq target-coords (instance arrow-object :init))
   (send target-coords :translate #f(500 0 0)) ;; initial pose
   (setq origin-coords (instance arrow-object :init)) ;;arrow for origin of object
   (send origin-coords :translate #f(500 0 0))
   (setq old-coords (send target-coords :copy-worldcoords))

   (objects (list robot target-coords origin-coords))
   (send *irtviewer* :draw-objects)
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

   (print "initialized")
   (print "start loop")
   (do-until-key
    (send self :publish-target-arrow-tf)
    (send self :publish-origin-arrow-tf)

    (ros::spin-once)
    (x::window-main-one)
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

  (:pose-move
   (&key (mem t))
   (let ((inter-num 10)(vector-list nil)
	 handle-r handle-l hadle-r-world handle-l-world
	 new-coords reset-coords old-coords add-coords)
     ;;モデルへの反映
     (send robot :angle-vector (send ri :state :potentio-vector))
     (setq handle-r (make-cascoords :coords (send origin-coords :transformation (send robot :rarm :end-coords))))
     (setq handle-l (make-cascoords :coords (send origin-coords :transformation (send robot :larm :end-coords))))

     (setq new-coords (send target-coords :copy-worldcoords))
     (setq reset-coords (send origin-coords :copy-worldcoords))
     (setq old-coords (send origin-coords :copy-worldcoords))

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
	    (unless (send robot :rarm
			  :inverse-kinematics handle-r-world
			  ;;:link-list link-list-r
			  ;;:move-target move-target-r
			  :rotation-axis ik-rotation-axis :debug-view nil)
	      (send target-coords :move-to reset-coords :world)
	      ;;(set-marker-pose-utorso reset-coord)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))
	   
	   (:larm
	    (unless (send robot :larm :inverse-kinematics handle-l-world
			  :rotation-axis ik-rotation-axis :debug-view nil)
	      (send target-coords :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      ))
	   
	   (t
	    (unless (send robot :rarm :inverse-kinematics handle-r-world
			  :rotation-axis ik-rotation-axis :debug-view nil)
	      (send target-coords :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      )
	    (unless (send robot :larm :inverse-kinematics handle-l-world
			  :rotation-axis ik-rotation-axis :debug-view nil)
	      (send target-coords :move-to reset-coords :world)
	      (send self :set-marker-pose-robot (send reset-coords :worldcoords))
	      (print "can't solve IK")
	      (return-from interpolation)
	      )
	    )
	   )
	 
	 
	 (send robot :head :look-at
	       (send add-coords :worldpos))
	 (push (send robot :angle-vector) vector-list)
	 (objects (list robot origin-coords add-coords))
	 (print add-coords)
	 (send *irtviewer* :draw-objects)
	 (setq reset-coord (send add-coords :copy-worldcoords))
	 )
       )
     (objects (list robot origin-coords target-coords))
     
     (print "exe")
     (setq vector-list (reverse vector-list))
     ;;(print vector-list)
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

  (:move-callback
   ( msg )
   (case (send msg :menu)
     ;;grasp callback
     (1
      (send self :start-grasp moving-arm))
     (31
      (send self :stop-grasp moving-arm))

     ;;pick up
     (32
      (print "under construction"))

     (2
      (send self :set-origin))

     ;;reset target coords
     (3
      (send target-coords :move-to old-coords :world)
      (send self :set-marker-pose-robot (send target-coords :worldcoords))
      )

     ;;set origin to hand
     (33
      (print "set origin to rhand")
      (send self :set-origin-to-hand :rarm))
     (34
      (print "set origin to lhand")
      (send self :set-origin-to-hand :larm))

     ;;change ik-rotation-axis
     (36
      (print "set ik-rotation-axis t")
      (setq ik-rotation-axis t)
      )
     (37
      (print "set ik-rotation-axis nil")
      (setq ik-rotation-axis nil)
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


     (0
      (send self :pose-move))

     ;;force-move
     (7
      (marker-teaching)
      (force-move)
      )
     (9
      ;;	    (reset-teaching)
      (reset-marker-sequence)
      )
     (8
      (setq *reproduce-mode* t)
      ;;	    (teached-move)
      (set-interactive-markers)
      )
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
     (20
      (setq moving-arm :rarm)
      )
     (21
      (setq moving-arm :larm)
      )
     (22
      (setq moving-arm :arms)
      )
     
     ;;change constraint
     (23
      (setq constraint-move t)
      )
     (24
      (setq constraint-move nil)
      )

     (40
      (send self :publish-target-arrow-obj)
      (send self :publish-origin-arrow-obj)
   


      )
     )
   )
  

  (:pose-callback 
   ( msg )
   (print (list "callback" msg))
   (let* ((pose (send msg :pose))
	  (cds (ros::tf-pose-stamped->coords pose))
	  (trs (send target-coords :copy-worldcoords))
	  )
     (setq cds-old (send target-coords :copy-worldcoords))
     (send target-coords :move-to (send (send self :get-base-coords) :worldcoords) :world)
     (send target-coords :transform cds)
     (send *irtviewer* :draw-objects)
     )
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
     (send *irtviewer* :draw-objects)
     ;;(print target-coords)
     )
   )
  
  (:set-marker-pose-robot
   (coords)
   (send self :set-marker-pose coords)
   )

  (:set-marker-pose 
   ( coords &key (frame base-frame) (markers) (server server-nodename))
   (let ((req (instance interactive_marker_interface::SetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_pose" server)
			req)
     ))
  

  (:set-markers 
   ( coords &key (frame base-frame) (markers) (server server-nodename) (marker_name 0))
   (let ((req (instance interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker_name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_markers" server)
			req)
     ))
  
  (:del-markers 
   ( coords &key (frame base-frame) (markers) (server server-nodename) (marker_name 0))
   (let ((req (instance interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker_name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/del_markers" server)
			req)
     ))

  (:reset-marker-pose 
   ( &key (server server-nodename))
   (let ((req (instance interactive_marker_interface::SetPoseRequest :init)))
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

