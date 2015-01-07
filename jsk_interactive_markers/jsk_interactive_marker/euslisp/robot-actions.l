#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_interactive_marker")

(require "models/arrow-object.l")
(require :robot-joint-movement "package://jsk_interactive_marker/euslisp/robot-joint-movement.l")

(defclass robot-actions
  :super robot-joint-movement
  :slots
  (*movingp*
   *frame-id*
   *use-moveit*
   *one-click-grasp*
   *one-click-grasp-arm*
   *use-base-movement*
   *use-torso*
   )
  )

(defmethod robot-actions
  (:grasp-pose-callback
   (msg)
   (when *movingp*
     (ros::ros-error "still moving")
     (return-from :grasp-pose-callback nil))
   (setq *movingp* t)
   ;;grasp
   (let* ((pose msg)
	  (cds (ros::tf-pose-stamped->coords msg))
	  map->marker
	  pre-pose
	  )
     (setq cds (send self :get-eus-coords cds (send pose :header)))
     (send *target-coords* :move-to (send (car (send *robot* :links)) :copy-worldcoords) :world)
     (send *target-coords* :transform cds :local)

     (send *target-coords* :translate (float-vector 10 0 0)) ; move a little bit forward
     (setq *tmp-rotation-axis* t)
     (dotimes (i 2)
       (send *ri* :stop-grasp *one-click-grasp-arm*)
       (unix:sleep 1))
     (let (avs time-sequences)
       (send *robot* *one-click-grasp-arm* :move-end-pos (float-vector -100 0 0))
       (send *robot* *one-click-grasp-arm* :move-end-pos (float-vector -50 0 0))
       (push (send *robot* :angle-vector) avs)

       (setq pre-pose (send (send *target-coords* :copy-worldcoords) :translate (float-vector -100 0 0)))
       (send pre-pose :translate (float-vector 0 0 50) :world)
       (when (> (abs (v. (send pre-pose :x-axis) (float-vector 0 0 1))) 0.5) ;??
         (send pre-pose :translate (float-vector 0 0 50) :world)
         (send pre-pose :translate (float-vector -30 0 0))
         )
       (unless (send *robot* :inverse-kinematics
                     pre-pose
                     :move-target (send *robot* *one-click-grasp-arm* :end-coords)
		     :use-torso t
                     :rotation-axis t
                     :debug-view t)
         (send pre-pose :rotate (deg2rad -180) :x)
         (if (send *robot* :inverse-kinematics
                   pre-pose
		   :move-target (send *robot* *one-click-grasp-arm* :end-coords)   
                   :use-torso t
                   :rotation-axis t
                   :debug-view t)
             (progn
               (send *target-coords* :rotate (deg2rad -180) :x)
               )
           (progn
             (setq *movingp* nil)
             (return-from :grasp-pose-callback nil)
             )))
       (push (send *robot* :angle-vector) avs)

       (unless (send *robot* :inverse-kinematics
                     *target-coords*
                     :move-target (send *robot* *one-click-grasp-arm* :end-coords) 
                     :use-torso t
                     :rotation-axis t
                     :debug-view t)
         (setq *movingp* nil)
         (return-from :grasp-pose-callback nil))

       (push (send *robot* :angle-vector) avs)

       (setq avs (reverse avs))
       (cond 
	((equal (string-upcase *robot-name*) "BAXTER")
	 (setq time-sequences  (list 2000 5000 3000)))
	(t
	 (setq time-sequences  (list 500 500 500))))
       (send *ri* :angle-vector-sequence avs time-sequences)
       )
     (send *ri* :wait-interpolation)
     (send *ri* :start-grasp  *one-click-grasp-arm*)
     (send *ri* :wait-interpolation)
     ;;(unix:usleep (* 1000 1000))
     (let (avs time-sequences)
       ;; pick up and place
       (push (send *robot* :angle-vector) avs)
       (send *robot* *one-click-grasp-arm* :move-end-pos #f(0 0 100) :world)
       (push (send *robot* :angle-vector) avs)
       
       (send *target-coords* :locate #f(0 0 -100) :world)
       (cond 
	((equal (string-upcase *robot-name*) "BAXTER")
	 (setq time-sequences  (list 3000 3000)))
	(t
	 (setq time-sequences  (list 1000 1000))))
       (send *ri* :angle-vector-sequence avs time-sequences)
       )

     (when (boundp '*irtviewer*)
       (send *irtviewer* :draw-objects))
     )
   (setq *movingp* nil)
   )

  (:move-model-callback
   (msg)
   (require "models/room73b2-scene.l")
   (ros::ros-info "move-model-callback")
   (print-ros-msg msg)
   (setq move-model msg)
   ;;set model
   (cond
    ((functionp (read-from-string (send msg :description)))
     (setq model (funcall (read-from-string (send msg :description)))))
    (t
     (ros::ros-error "model is undefined : ~A" (send msg :description))
     (return-from :move-model-callback nil))
    )
   (setq model-origin (copy-object model))
   (setq model-goal (copy-object model))

   (setq handle (copy-object (send model :handle)))
   (dolist (one-handle (send model :handle))
     (if (find-method model (read-from-string (string-upcase (send one-handle :name))))
	 (send  (send model (read-from-string (string-upcase (send one-handle :name))) :child-link) :assoc one-handle)
       )
     )

   (send model-origin :move-to (send self :pose-stamped->coords (send msg :pose_origin)) :world)
   (send model-goal :move-to (send self :pose-stamped->coords (send msg :pose_goal)) :world)
   (send model :move-to (send self :pose-stamped->coords (send msg :pose_goal)) :world)
   (send *robot* :move-to (send self :pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) *robot-origin*)) :world)

   (apply-joint_state (send msg :joint_state_origin) model-origin)
   (apply-joint_state (send msg :joint_state_goal) model-goal)

   (when (and (boundp '*irtviewer*) *irtviewer*)
     (objects (list model model-origin model-goal *robot*))
     )

   (setq joint-num (length (send model-origin :joint-list)))

   (when (> joint-num 0)
     ;;check move joint
     (setq move-joint (make-list joint-num :initial-element nil))
     (dotimes (i joint-num)
       (setq angle-origin (send (elt (send model-origin :joint-list) i) :joint-angle))
       (setq angle-goal (send (elt (send model-goal :joint-list) i) :joint-angle))

       (cond
	((derivedp (elt (send model-origin :joint-list) i) rotational-joint)
	 (if (> (abs (- angle-origin angle-goal)) 5) ;; 5 degree
	     (setf (elt move-joint i) t)
	   ))
	((derivedp (elt (send model-origin :joint-list) i) linear-joint)
	 (if (> (abs (- angle-origin angle-goal)) 50) ;; 50 mm
	     (setf (elt move-joint i) t)
	   ))
	))

     (print move-joint)
     (send model :move-to (send model-goal :copy-worldcoords) :world)
     (when (boundp '*irtviewer*)
       (objects (list model model-origin model-goal *robot*)))
     (dotimes (i joint-num)
       (when (elt move-joint i)
	 (setq grasp-handle nil)
	 (dolist (one-handle (send model :handle))
	   (when (equal (send one-handle :parent) (send (elt (send model :joint-list) i) :child-link))
	     (setq grasp-handle one-handle)
	     (return)
	     ))

	 (setq angle-origin (send (elt (send model-origin :joint-list) i) :joint-angle))
	 (setq angle-goal (send (elt (send model-goal :joint-list) i) :joint-angle))
	 (setq interpolation-num 10)
	 (setq grasp-handle-list nil)
	 (setq move-model-list nil)
	 (dotimes (j (1+ interpolation-num))
	   (send (elt (send model :joint-list) i) :joint-angle
		 (+ (* (/ (- angle-goal angle-origin) interpolation-num) j) angle-origin))

	   (push (send (send  grasp-handle :copy-worldcoords) :rotate pi :z) grasp-handle-list)
	   (push (copy-object model) move-model-list)
	   )
	 (setq grasp-handle-list (reverse grasp-handle-list))
	 (setq move-model-list (reverse move-model-list))
	 (send self :move-model-ik grasp-handle-list move-model-list)
	 )
       )
     )
   
   (when (equal joint-num 0)
     (ros::ros-info "pre grasp")
     (send self :move-object 
	   (send self :pose-stamped->coords (send msg :pose_origin))
	   (send self :pose-stamped->coords (send msg :pose_goal))
	   :model model)
     )
   )
  (:move-model-ik
   (grasp-handle-list move-model-list)
   (setq ik-args
	 (list :use-base 0.01
	       :use-torso t
	       :base-range (list :min #f(-10000 -10000 -10000) :max #f(10000 10000 10000))
	       :debug-view nil
	       ))
   (dotimes (loop-times 2)
     (let (first-handle
	   handle-vec
	   (offset-length 500)
	   initial-vec
	   initial-rot
	   initial-coords)

       ;; move *robot* in front of grasp object
       ;; initial position for ik
       (setq first-handle (send (car grasp-handle-list) :copy-worldcoords))
       (setq handle-vec (normalize-vector (send first-handle :rotate-vector #f(1 0 0))))

       (setq initial-vec (v- (send first-handle :worldpos) (scale offset-length handle-vec)))
       (setq initial-vec (float-vector (elt initial-vec 0) (elt initial-vec 1) 0))
       (setq initial-rot (atan2 (elt handle-vec 1) (elt handle-vec 0)))

       (setq initial-coords (make-coords :pos initial-vec
					 :rot (rotate-matrix (unit-matrix 3) initial-rot :z)))
       (send *robot* :move-to initial-coords :world)
       )
     (when (boundp '*irtviewer*)
       (objects (list model model-origin model-goal *robot*)))
     (setq test-ik-index (list 0 5 10 5 0))
     (dolist (i test-ik-index)
       (send* *robot* :rarm
	      :inverse-kinematics (elt grasp-handle-list i)
	      :use-base 0.1 ;;parameter
	      ik-args)
       (when (boundp '*irtviewer*)
	 (send *irtviewer* :draw-objects))

       )

     (dolist (i test-ik-index)
       (send* *robot* :rarm
	      :inverse-kinematics (elt grasp-handle-list i)
	      :use-base 0.01 ;;parameter
	      ik-args)
       (when (boundp '*irtviewer*)
	 (send *irtviewer* :draw-objects))

       )

     (dolist (i test-ik-index)
       (send* *robot* :rarm
	      :inverse-kinematics (elt grasp-handle-list i)
	      :use-base 0.001 ;;parameter
	      ik-args)
       (when (boundp '*irtviewer*)
	 (send *irtviewer* :draw-objects))
       )
     
     ;;tuck arm
     (pr2-tuckarm-pose)
     ;;move to
     (send *ri* :move-to (send *robot* :worldcoords) :frame-id *frame-id*)
     (unless (send *ri* :simulation-modep)
       (send *robot* :move-to (send self :pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) *robot-origin*)) :world)
       )

     ;;ik
     (send *robot* :head :look-at (send (car grasp-handle-list) :worldpos))
     (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     (unless (send *ri* :simulation-modep)
       (ros::ros-info "sleep for nearest bounding box")
       (unix::sleep 7)
       (ros::ros-info "sleep end for nearest bounding box")
       )
     ;;get nearest bounding box and modify grasp-handle-list
     (let ((req (instance jsk_pcl_ros::NearestBoundingBoxRequest :init))
	   req-pose-stamped
	   res
	   nearest-grasp-coords)

       (setq req-pose-stamped (ros::coords->tf-pose-stamped (car grasp-handle-list) *frame-id*))
       (send req :header (send req-pose-stamped :header))
       (send req :pose (send req-pose-stamped :pose))
       (send req :max_distance 0.300)

       (setq res (ros::service-call "get_nearest_bounding_box" req))
       (print-ros-msg res)
       ;;try once more
       (when (equal (send res :box :header :frame_id) "")
	 (unix::sleep 3)
	 (setq res (ros::service-call "get_nearest_bounding_box" req)))
       (cond
	((equal (send res :box :header :frame_id) "")
	 (return)
	 )
	(t
	 (setq grasp-offset (float-vector 30 0 0))

	 ;;revert grasp offset
	 (unless (eq loop-times 0)
	   (dolist (grasp-handle grasp-handle-list)
	     (send grasp-handle :translate (scale -1 grasp-offset) :local))
	   )

	 (print "get nearest bounding box")
	 (setq nearest-grasp-coords
	       (send self :pose-stamped->coords (instance geometry_msgs::PoseStamped :init :header (send res :box :header) :pose (send res :box :pose))))
	 (setq offset-vector (v- (send nearest-grasp-coords :worldpos) (send (car grasp-handle-list) :worldpos)))
	 (setf (elt offset-vector 2) 0) ;;ignore z offset
	 (print offset-vector)
	 (dolist (grasp-handle grasp-handle-list)
	   (send grasp-handle :translate offset-vector :world))

	 ;;in order to grasp tightly
	 (dolist (grasp-handle grasp-handle-list)
	   (send grasp-handle :translate grasp-offset :local))
	 )
	)))
   
   ;;reset pose
   (send *robot* :reset-pose)
   (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
   (send *ri* :wait-interpolation)

   (when (boundp '*irtviewer*)
     (objects (list model *robot*))
     (x::window-main-one))


   (setq avs nil)
   (dotimes (i (length grasp-handle-list))
     (if
	 (send* *robot* :rarm
		:inverse-kinematics (elt grasp-handle-list i)
		:look-at-target t
		:use-base nil ;;parameter
		ik-args)
	 (progn
	   (push (send *robot* :angle-vector) avs)
	   (setq model (elt move-model-list i))
	   (when (boundp '*irtviewer*)
	     (objects (list model *robot*))
	     (send *irtviewer* :look-all)
	     (send *irtviewer* :draw-objects)
	     (x::window-main-one)
	     (unix:sleep 3))
	   )
       (return)
       )
     )
   (setq avs (reverse avs))

   (when avs
     (send *ri* :stop-grasp :rarm :wait t)
     (send *robot* :angle-vector (elt avs 0))
     (send *robot* :rarm :move-end-pos #f(-200 0 0))
     (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
     (send *ri* :wait-interpolation)
     (send *robot* :angle-vector (elt avs 0))
     (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     (send *ri* :start-grasp :rarm)
     )

   (send *ri* :angle-vector-sequence avs 1000)
   (send *ri* :wait-interpolation)
   (when (boundp '*irtviewer*)
     (send *irtviewer* :draw-objects))
   )

  (:get-eus-coords
   (cds header)
   (let (origin->frame
	 (frame-id (send header :frame_id))
	 ;;(time (send header :stamp))
	 (time (ros::time-now))
	 (wait-times 3)
	 )
     (dotimes (i wait-times)
       (setq origin->frame
	     (send *tfl* :lookup-transform
		   *robot-origin* frame-id time))
       (cond
	(origin->frame
	 (send cds :transform origin->frame :world)
	 (return-from :get-eus-coords cds)
	 )
	(t
	 (ros::ros-info "~A -> ~A cannot transform yet" *robot-origin* frame-id)
	 (unix:sleep 1)
	 (ros::sleep)
	 ))))
   nil
   )

  (:pose-stamped->coords
   (pose-stamped)
   (let ((cds (ros::tf-pose-stamped->coords pose-stamped)))
     (while t
       (setq frame-id->ps-frame-id
	     (send *tfl* :lookup-transform
		   *frame-id* (send pose-stamped :header :frame_id) (ros::time 0)))
       (cond
	(frame-id->ps-frame-id
	 (send cds :transform frame-id->ps-frame-id :world)
	 (send cds :name *frame-id*)
	 (return)
	 )
	(t
	 (ros::ros-info "~A -> ~A cannot transform yet" *frame-id* (send pose-stamped :header :frame_id))
	 (unix:sleep 1)
	 (ros::sleep)
	 )
	)
       )
     cds
     )
   )

  (:inverse-kinematics
   (target-coords &rest args &key 
		  (move-arm :rarm)
		  ((:use-torso ut) *use-torso*)
		  (rotation-axis t)
		  (look-at-target t)
		  (debug-view nil)
		  &allow-other-keys)
   (send* *robot* :inverse-kinematics
	  target-coords
	  :move-target (send *robot* move-arm :end-coords)
	  :use-torso ut
	  :rotation-axis rotation-axis
	  :look-at-target look-at-target
	  :debug-view debug-view
	  args
	  )
   )

  (:send-angle-vector
   (&optional (tm 5000) &key (use-moveit *use-moveit*))
   
   (if use-moveit
       (progn
	 (send *ri* :angle-vector (send *robot* :angle-vector) tm :head-controller)
	 (send *ri* :angle-vector-motion-plan (send *robot* :angle-vector)
	       :move-arm :arms :use-torso t))
     (send *ri* :angle-vector (send *robot* :angle-vector) tm))

   (send *ri* :wait-interpolation)

   ;;deal with difference of joint angle between moveit and eus
   (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
   (send *ri* :wait-interpolation)
   )

  (:move-robot-in-front-of-object
   (target-coords &key (offset-length 1000) (real nil))
   (let (target-x-vec
	 pos
	 yaw
	 front-coords)
     (setq target-x-vec (normalize-vector (send target-coords :rotate-vector #f(1 0 0))))

     (setq pos (v- (send target-coords :worldpos) (scale offset-length target-x-vec)))
     (setq pos (float-vector (elt pos 0) (elt pos 1) 0))
     (setq yaw (atan2 (elt target-x-vec 1) (elt target-x-vec 0)))
     
     (setq front-coords 
	   (make-coords :pos pos
			:rot (rotate-matrix (unit-matrix 3) yaw :z)))

     (send *robot* :move-to front-coords :world)
     (when real
       (send *ri* :move-to (send *robot* :worldcoords) :frame-id *frame-id*)
       )
     ))

  (:check-ik-and-move-base
   (target-cds-list &key (move-arm :rarm) (grasp-object nil) (tuck-free-arm nil))
   (let ik-suc
     ;;check if ik can be solved
     (dolist (target target-cds-list)
       (setq ik-suc (send self :inverse-kinematics target :move-arm move-arm))
       (unless ik-suc (return)))

     ;;when ik cannot be solved, move base
     (when (and (not ik-suc) *use-base-movement*)
       (send self :move-robot-in-front-of-object (car target-cds-list) :offset-length 1000)

       (dolist (target target-cds-list)
	 (setq ik-suc
	       (send self :inverse-kinematics target :move-arm move-arm
		     :use-base 0.01
		     :base-range (list :min #f(-10000 -10000 -10000) :max #f(10000 10000 10000))
		     )))

       ;; move base
       (when ik-suc
	 (send self :tuckarm-pose move-arm :grasp-object grasp-object :only-free-arm tuck-free-arm)
	 ;; (case (move-arm
	 ;; 	(:larm (send *robot* :larm :angle-vector #f(25 0 0 -121 0 -6 0)))
	 ;; 	(:rarm (send *robot* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0)))))
	 ;; (send self :send-angle-vector)

	 ;; (let ((side (if grasp-object :outside :inside)))
	 ;;   (send *robot* move-arm :angle-vecor (get-tuckarm move-arm side move-arm))
	 ;;   )
	 ;; (send self :send-angle-vector)
	 ;;move to
	 (send *ri* :move-to (send *robot* :worldcoords) :frame-id *frame-id*)
	 (unless (send *ri* :simulation-modep)
	   (send *robot* :move-to (send self :pose-stamped->coords (ros::coords->tf-pose-stamped (make-coords) *robot-origin*)) :world)
	   )
	 )
       )
     )
   )

  (:tuckarm-pose
   (&optional (arm :rarm) &key (grasp-object nil) (only-free-arm nil))
   (let ((side (if grasp-object :outside :inside)))
     (if only-free-arm
	 (progn
	   (case arm
	     (:larm (send *robot* :larm :angle-vector #f(25 0 0 -121 0 -6 0)))
	     (:rarm (send *robot* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))))
	   (send self :send-angle-vector)
	   (send *robot* arm :angle-vector (get-tuckarm arm side arm))
	   (send self :send-angle-vector))
       (pr2-tuckarm-pose arm side)
   )))
  
  (:move-object 
   (current-coords goal-coords &key (handle-pose) (model) (move-arm :rarm))
   (unless (or handle-pose model)
     (ros::ros-warn "handle-pose or model should be set")
     (return-from :move-object nil)
     )

   (unless handle-pose
     (dolist (one-handle (send model :handle))
       (when (equal (send one-handle :parent) (car (send model :links)))
	 (setq handle-pose (send one-handle :coords))
	 (return)
	 )))

   (unless handle-pose
     (ros::ros-warn "handle is not defined")
     (return-from :move-object nil)
     )
   
   (setq current-coords (send current-coords :transform handle-pose))
   (setq goal-coords (send goal-coords :transform handle-pose))

   (objects (list *robot* model current-coords goal-coords))

     ;;;;;;;;;;;;;;;;;;;;;;
     ;;;;;;; pick ;;;;;;;;;
     ;;;;;;;;;;;;;;;;;;;;;;
   (let* ((target-cds (send current-coords :copy-worldcoords))
	  (pre-cds (send (send current-coords :copy-worldcoords) :translate #f(-100 0 0) :local))
	  (post-cds (send (send current-coords :copy-worldcoords) :translate #f(0 0 100) :world))
	  (target-cds-list (list pre-cds target-cds post-cds))
	  ik-suc)

     (send self :check-ik-and-move-base target-cds-list :move-arm move-arm)

     ;;pre grasp
     (ros::ros-info "pre grasp")
     (setq ik-suc (send self :inverse-kinematics pre-cds :move-arm move-arm))
     (unless ik-suc
       (ros::ros-info "ik failed")
       (return-from :move-object nil))
     (send self :send-angle-vector)
     (send *ri* :stop-grasp move-arm :wait t)

     ;;grasp
     (ros::ros-info "grasp")
     (setq ik-suc (send self :inverse-kinematics target-cds :move-arm move-arm))

     (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     (send *ri* :start-grasp move-arm)
     (send *ri* :wait-interpolation)

     ;;raise
     (ros::ros-info "raise")
     (setq ik-suc (send self :inverse-kinematics post-cds :move-arm move-arm))
     (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     )

     ;;;;;;;;;;;;;;;;;;;;;;
     ;;;;;;; place ;;;;;;;;
     ;;;;;;;;;;;;;;;;;;;;;;
   (let* ((target-cds (send goal-coords :copy-worldcoords))
	  (pre-cds (send (send goal-coords :copy-worldcoords) :translate #f(0 0 100) :world))
	  (post-cds (send (send goal-coords :copy-worldcoords) :translate #f(-100 0 0) :local))
	  (target-cds-list (list pre-cds target-cds post-cds))
	  ik-suc)

     (send self :check-ik-and-move-base target-cds-list :move-arm move-arm :grasp-object t :tuck-free-arm t)
     ;;pre place
     (ros::ros-info "pre place")
     (setq ik-suc (send self :inverse-kinematics pre-cds :move-arm move-arm))
     (unless ik-suc
       (ros::ros-info "ik failed")
       (return-from :move-object nil))
     (send self :send-angle-vector)

     ;;place
     (ros::ros-info "place")
     (setq ik-suc (send self :inverse-kinematics target-cds :move-arm move-arm))

     (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     (send *ri* :stop-grasp move-arm :wait t)

     ;;release
     (ros::ros-info "release")
     (setq ik-suc (send self :inverse-kinematics post-cds :move-arm move-arm))
     (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     )

   ;;reset pose
   (send self :tuckarm-pose move-arm :only-free-arm t)
   (send self :send-angle-vector)
   (send *ri* :start-grasp move-arm)
   )

  (:box-movement-callback
   (msg)
   ;;grasp
   (let* ((pose msg)
	  current-coords
	  des-cds
	  (handle-pose (ros::tf-pose->coords (send msg :handle_pose)))
	  box-cds
	  (move-arm :rarm)
	  (use-torso t)
	  )
     
     (setq current-coords (send self :get-eus-coords (ros::tf-pose->coords (send msg :box :pose)) (send msg :box :header)))
     (setq des-cds (send self :get-eus-coords (ros::tf-pose->coords (send msg :destination :pose)) (send msg :destination :header)))
     (unless current-coords
       (return-from :box-movement-callback nil)
       )
     (unless des-cds
       (return-from :box-movement-callback nil)
       )

     (send self :move-object current-coords des-cds :handle-pose handle-pose)

     )
   )

  (:init 
   ()
   (send-super :init)
   (setq *target-coords* (instance arrow-object :init))
   (send *target-coords* :translate #f(500 0 0)) ;; initial pose
   (setq *use-fullbody* t)
   ;;(setq *use-torso* nil)

   (setq *frame-id* (ros::get-param "~frame_id" "map"))
   (setq *use-moveit* (ros::get-param "~use_moveit" t))
   (setq *use-torso* (ros::get-param "~use_torso" t))
   (setq *use-base-movement* (ros::get-param "~use_base_movement" nil))

   (setq *one-click-grasp* (ros::get-param "~one_click_grasp" nil))
   (setq *one-click-grasp-arm* (intern (string-upcase (ros::get-param "~one_click_grasp_arm" "rarm")) *keyword-package*))

   (setq *robot-origin* (send (car (send *robot* :links)) :name))

   (setq *im-nodename* "jsk_model_marker_interface")
   (setq *server-nodename* "/jsk_interactive_marker_manipulation")

   (setq *interactive-pc-nodename* "/interactive_point_cloud")

   (ros::roseus "robot_actions")
   (setq *tfb* (instance ros::transform-broadcaster :init))
   (setq *tfl* (instance ros::transform-listener :init))

   (ros::subscribe
    (format nil "~A/grasp_pose" *server-nodename*)
    geometry_msgs::PoseStamped #'send self :grasp-pose-callback)

   (ros::subscribe
    (format nil "~A/box_movement" *interactive-pc-nodename*)
    jsk_pcl_ros::BoundingBoxMovement #'send self :box-movement-callback)

   (ros::subscribe
    (format nil "~A/move_model" *im-nodename*)
    jsk_interactive_marker::MoveModel #'send self :move-model-callback)

   (if *one-click-grasp*
       (ros::subscribe
	(format nil "~A/grasp_pose" *interactive-pc-nodename*)
	geometry_msgs::PoseStamped #'send self :grasp-pose-callback)
     )
   
   (when (and x::*display* (> x::*display* 0))
     (objects (list *robot* *target-coords*)))

   (cond
    ((boundp '*irtviewer*)
     (send *irtviewer* :change-background #f(0.9 0.9 1.0))
     (send *irtviewer* :title "Robot Actions")

     (send *irtviewer* :draw-objects)
     (send *irtviewer* :look-all)
     (while (ros::ok)
       (x::window-main-one)
       (ros::spin-once)
       )
     )
    (t
     (ros::spin)
     )
    )
   )
  )

(instance robot-actions :init)
