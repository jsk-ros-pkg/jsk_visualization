#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "jsk_interactive")
(ros::load-ros-manifest "jsk_interactive_marker")

(ros::roseus "ik-controller")

(require "models/arrow-object.l")
(require "package://roseus/euslisp/actionlib.l")
(require "package://pr2eus/pr2-interface.l")

;;;;;;;; setup ;;;;;;;;;;;
(defclass touch-it-client
  :super ros::simple-action-client
  :slots (feedback))
(defmethod touch-it-client
  (:action-feedback-cb
   (msg)
   (ros::ros-debug "[~A] feedback-cb ~A" ros::name-space msg)
   (setq feedback msg))
  (:feedback () feedback)
  (:simple-state () ros::simple-state)
  )


(defclass im-controller
  :slots
  (*robot*
   )
  )


(defvar *touch-it-interface*
  (setq *c* (instance touch-it-client :init
		      "/touchit/act/exec"
		      move_base_msgs::MoveBaseAction)))
(defvar *touch-it-interface-ocs*
  (setq *c* (instance ros::simple-action-client :init
		      "/touchit/act/exec_ocs"
		      move_base_msgs::MoveBaseAction)))
(defvar *touchit-reached* 4)
(defvar *touchit-touched* 3)
(defvar *touchit-cancelled* 6)

(defvar *touchit-target-larm* 0)
(defvar *touchit-target-larm-torso* 1)
(defvar *touchit-target-larm-fullbody* 2)
(defvar *touchit-target-rarm* 3)
(defvar *touchit-target-rarm-torso* 4)
(defvar *touchit-target-rarm-fullbody* 5)

(defvar *touch-it-server-detected* nil)


(defmethod im-controller
  (:call-ik-server
   (&key
    (target-limb
     (case *move-limb*
       (:arms :larm)
       (t *move-limb*)))
    (group_name
     (nth
      (cond
       (*use-torso* 1)
       (*use-fullbody* 2)
       (t 0))
      (case target-limb
	(:larm '("larm_eus" "larm_torso_eus" "larm_fullbody_eus"))
	(:rarm '("rarm_eus" "rarm_torso_eus" "rarm_fullbody_eus")))))
    (req (instance moveit_msgs::GetPositionIKRequest :init))
    (move-coords (make-coords))
    (initial-coords
     (send *robot* target-limb :end-coords :copy-worldcoords))
    (target-coords
     (make-coords
      :pos (v+ (send initial-coords :worldpos)
	       (send move-coords :worldpos))
      :rot (m* (send move-coords :worldrot)
	       (send initial-coords :worldrot)
	       )))
    (target-coords-from-pelvis
     (send (send *robot* :copy-worldcoords)
	   :transformation target-coords))
    (prev-state
     (list (cons :angle-vector
		 (copy-seq (send *robot* :angle-vector)))
	   (cons :worldcoords
		 (copy-object (send *robot* :worldcoords)))))
    (log-stream t)
    (translation-axis *translation-axis*)
    (rotation-axis
     (if *tmp-rotation-axis* *rotation-axis* nil)
     )
    (revert-if-fail t)
    (update-convex t)
    &allow-other-keys
    )
   ;;(cond
   ;;(*ik-stop* (return-from send self :call-ik-server nil)))
   (let* ((c target-coords-from-pelvis)
	  (p (scale 1e-3 (send c :worldpos)))
	  (q (matrix2quaternion (send c :worldrot)))
	  res)
     (send req :ik_request :group_name group_name)
     ;;
     (send *prev-js*
	   :header
	   (instance std_msgs::Header :init
		     :stamp (ros::time 0.0)
		     :frame_id (format nil "/~A" *robot-origin*)))
     (send *prev-js*
	   :position
	   (map float-vector
		#'deg2rad
		(send *robot* :angle-vector)))
     (send req :ik_request :robot_state :joint_state *prev-js*)
     ;;
     #|
     (send req :ik_request :constraints :name
	   (apply
	    #'concatenate
	    (append
	     (list string)
	     (list "(")
	     (mapcar
	      #'(lambda (key val) (format nil "~A ~A " key val))
	      '(:translation-axis :rotation-axis :stop :update-convex
				  :thre :rthre :revert-if-fail)
	      (list
	       (list translation-axis t t)
	       (list rotation-axis t t)
	       15
	       update-convex
	       (list (max (* 0.9 1)
			  (* 0.9 (norm (send move-coords :worldpos))))
		     5 5)
	       (list (max (* 0.9 (deg2rad 1))
			  (* 0.9 (norm
				  (coerce
				   (car (send move-coords :rpy-angle))
				   float-vector))))
		     (deg2rad 5) (deg2rad 5))
	       revert-if-fail
	       ))
	     (list ")"))))
     |#
     ;;
     (send req :ik_request
	   :pose_stamped
	   (instance geometry_msgs::PoseStamped :init
		     :header
		     (instance std_msgs::Header :init
			       :stamp (ros::time 0.0)
			       :frame_id (format nil "/~A" *robot-origin*)
			       )
		     :pose
		     (instance geometry_msgs::Pose :init
			       :position
			       (instance geometry_msgs::Point
					 :init
					 :x (elt p 0)
					 :y (elt p 1)
					 :z (elt p 2))
			       :orientation
			       (instance geometry_msgs::Quaternion
					 :init
					 :x (elt q 1)
					 :y (elt q 2)
					 :z (elt q 3)
					 :w (elt q 0)))
		     ))
     (setq res (ros::service-call "/solve_ik" req))
     (cond
      ((= (send res :error_code :val)
	  moveit_msgs::MoveItErrorCodes::*SUCCESS*)
       (format log-stream "successed!~%")
       (setq *prev-js* (send res :solution :joint_state))
       (send *robot* :angle-vector
	     (map float-vector
		  #'rad2deg
		  (send res :solution :joint_state :position)))
       (send *robot* :fix-leg-to-coords
	     (make-coords) :both)
       (send *viewer* :draw-objects)
       ;;
       (if (or
	    (null *success-state-collection*)
	    (> (norm
		(v- (send *robot* :angle-vector)
		    (cdr (assoc :angle-vector prev-state))))
	       1))
	   (setq *success-state-collection*
		 (subseq
		  (cons
		   (list (cons :angle-vector
			       (copy-object
				(send *robot* :angle-vector)))
			 (cons :worldcoords
			       (copy-object
				(send *robot* :worldcoords))))
		   *success-state-collection*)
		  0 500)))
       (cond
	((eq rotation-axis nil)
	 (send *target-coords*
	       :newcoords
	       (make-coords
		:pos (copy-object (send *target-coords* :worldpos))
		:rot (copy-object
		      (send (send self :tf-model-end-coords) :worldrot))))
	 (send *target-coords* :worldcoords)
	 (send self :set-marker-pose-robot (send *target-coords* :worldcoords))))
       (send (send self :tf-model-end-coords) :draw-on :flush t :color #F(1 0 0) :size 1000)
       (send *robot* :angle-vector)
       )
      (t (format log-stream "failed!~%") nil))))
  (:call-touch-it-server
   (command
    &key
    (ik-type
     (cond
      ((eq *move-limb* :rarm)
       (cond
	(*use-fullbody* *touchit-target-rarm-fullbody*)
	(*use-torso* *touchit-target-rarm-torso*)
	(t *touchit-target-rarm*)))
      ((eq *move-limb* :larm)
       (cond
	(*use-fullbody* *touchit-target-larm-fullbody*)
	(*use-torso* *touchit-target-larm-torso*)
	(t *touchit-target-larm*)))))
    (frame_id (format nil "/~A" *robot-origin*))
    (target-coords *target-coords*)
    (target-coords-from-pelvis
     (send (send *robot* :copy-worldcoords)
	   :transformation target-coords))
    (touch-it-interface
     (case command
       (:prev *touch-it-interface-ocs*)
       (t *touch-it-interface*)))
    (axis
     (case *rotation-axis*
       (:z 0)
       (:x 1)
       (:y 2)
       (nil 3)
       (t 4)))
    res)
   (let (feedback-value feedback-value2)
     ;;(case command
     ;;(:prev nil)
     ;;(t (send touch-it-interface :cancel-all-goals)))
     ;;(unix:sleep 2)
     (case command
       (:cancel (send touch-it-interface :cancel-all-goals))
       (t
	(let* ((c target-coords-from-pelvis)
	       (p (scale 1e-3 (send c :worldpos)))
	       (q (matrix2quaternion (send c :worldrot)))
	       goal)
	  ;; set request
	  (warning-message 2 "[touch_it_client] start waiting for server~%")
	  (cond
	   ((not *touch-it-server-detected*)
	    (setq *touch-it-server-detected* t)
	    (send touch-it-interface :wait-for-server)))
	  (warning-message 2 "[touch_it_client] finish waiting for server ~%")
	  (setq goal (instance move_base_msgs::MoveBaseActionGoal :init))
	  (send goal :goal :target_pose :header :frame_id frame_id)
	  (send goal :goal :target_pose :header :stamp (ros::time 0.0))
	  (send goal :goal :target_pose :pose :position :x (elt p 0))
	  (send goal :goal :target_pose :pose :position :y (elt p 1))
	  (send goal :goal :target_pose :pose :position :z (elt p 2))
	  (send goal :goal :target_pose :pose :orientation :x (elt q 1))
	  (send goal :goal :target_pose :pose :orientation :y (elt q 2))
	  (send goal :goal :target_pose :pose :orientation :z (elt q 3))
	  (send goal :goal :target_pose :pose :orientation :w (elt q 0))
	  (send goal :goal :target_pose :header :seq (+ (* 10 axis) ik-type))
	  ;; send request
	  (warning-message 2 "[touch_it_client] send goal to server ~%")
	  (send touch-it-interface :send-goal goal)
	  ;; wait
	  (warning-message 2 "[touch_it_client] start waiting for result~%")
	  (ros::rate 10)
	  (while (not (equal (send touch-it-interface :simple-state) ros::*simple-goal-state-done*))
	    (warning-message 2 "[touch_it_client] waiting for result ... ~A~%" (send touch-it-interface :feedback))
	    (when (send touch-it-interface :feedback)
	      (setq feedback-value (send (ros::tf-pose-stamped->coords (send (send touch-it-interface :feedback) :feedback :base_position)) :pos))
	      (setq feedback-value2 (car (rpy-angle (send (ros::tf-pose-stamped->coords (send (send touch-it-interface :feedback) :feedback :base_position)) :rot))))
	      (send self :publish-touchit-status
		    (format nil "force ~5,2f / ~5,2f ~% IK ~a  (~5,2f %)"
			    (elt feedback-value 0) (elt feedback-value 1) (if (eps= (elt feedback-value2 0) 0) "FAIL" "SUCC") (elt feedback-value 2)))
	      )
	    (ros::sleep)
	    (ros::spin-once)
	    ))))
     ;; (warning-message 2 "[touch_it_client] result1: ~a ~%" (send *c* :get-result))
     (setq res (send touch-it-interface :get-state))
     ;; (warning-message 2 "[touch_it_client] result2: ~a ~%" res)
     (send self :publish-touchit-status
	   (if (send touch-it-interface :feedback)
	       (progn
		 (setq feedback-value (send (ros::tf-pose-stamped->coords (send (send touch-it-interface :feedback) :feedback :base_position)) :pos))
		 (format nil
			 (format nil "~A (~a)~%force ~5,2f / ~5,2f  (~5,2f %)"
				 (cond
				  ((equal res *touchit-touched*) "TOUCHED")
				  ((equal res *touchit-reached*) "REACHED")
				  ((equal res *touchit-cancelled*) "CANCELLED"))
				 res
				 (elt feedback-value 0) (elt feedback-value 1) (elt feedback-value 2))))
	     (format nil
		     (format nil "~A (~a)"
			     (cond
			      ((equal res *touchit-touched*) "TOUCHED")
			      ((equal res *touchit-reached*) "REACHED")
			      ((equal res *touchit-cancelled*) "CANCELLED"))
			     res))
	     :scale 0.25))
     res))

  ;; initial pose
  (:init-pose
   nil
   (send *robot* :reset-manip-pose)
   (send *robot* :fix-leg-to-coords (make-coords) :both)
   (if (and (boundp '*viewer*) *viewer*)
       (send *viewer* :draw-objects))
   )
;;;;;;;; setup ;;;;;;;;;;;

;;;;;;;; controller ;;;;;;;;;

  (:str-subscriber
   (msg)
   (let ((command (send msg :data))
	 (move))
     (if (zerop (length command))
	 (setq command "hoge"))
     (setq move
	   (scale
	    5
	    (case (aref command 0)
	      (#\u #F(0 0 1))
	      (#\d #F(0 0 -1))
	      (#\r #F(0 -1 0 ))
	      (#\l #F(0 1 0))
	      (#\f #F(1 0 0))
	      (#\b #F(-1 0 0))
	      (#\v (read-from-string
		    (concatenate
		     string
		     "#f"
		     (subseq command 1))))
	      (t #F(0 0 0)))))
     (send self :call-ik-server
	   :move-coords
	   (make-coords :pos move))))

  (:spacenav-subscriber
   (msg
    &optional
    (move-filter '(1 1 1))
    (rpy-filter '(1 1 1)))
   (cond
    ((and (> (aref (send msg :buttons) 0) 0.1)
	  (> (aref (send msg :buttons) 1) 0.1))
     (format t "mouse-mode~%")
     (format t "standby..?~%")
     (setq *mouse-mode* (not *mouse-mode*))
     (print 3) (unix:sleep 1)
     (print 2) (unix:sleep 1)
     (print 1) (unix:sleep 1)
     (format t "start!!~%")
     )
    ((not *mouse-mode*)
     (return-from :spacenav-subscriber nil))
    ((> (aref (send msg :buttons) 1) 0.1)
     (print 'waist-fix)
     (waist-fix :waist-key :waist-r :cnt 3)
     (waist-fix :waist-key :waist-p :cnt 3)
     (waist-fix :waist-key :waist-y :cnt 3)
     (print 'done)
     )
    ((null *success-state-collection*) nil)
    ((> (aref (send msg :buttons) 0) 0.1)
     (print 'pop-back-robot-state)
     (let ((state (pop *success-state-collection*)))
       (send *robot* :angle-vector
	     (cdr (assoc :angle-vector state)))
       (send *robot* :newcoords
	     (cdr (assoc :worldcoords state)))
       (send *viewer* :draw-objects))
     ))
   (let ((move
	  (scale
	   30
	   (coerce
	    (subseq (send msg :axes) 0 3)
	    float-vector)))
	 (rpy
	  (reverse
	   (scale
	    0.12
	    (coerce
	     (subseq (send msg :axes) 3 6)
	     float-vector)))))
     (cond
      ((< (norm
	   (coerce
	    (send msg :axes)
	    float-vector)) 0.01)
       ;; (concatenate
       ;;  float-vector
       ;;  (scale 1e-3 move)
       ;;  rpy)) 0.01)
       (format t "skip ~A ~A~%" move rpy))
      (t
       (setq move
	     (map float-vector
		  #'*
		  move move-filter))
       (setq rpy
	     (map float-vector
		  #'*
		  rpy rpy-filter))
       (if (vectorp
	    (send self :call-ik-server
		  :rotation-axis t
		  :translation-axis t
		  :move-coords
		  (make-coords :pos move :rpy rpy)))
	   (send self :publish-joint-states :real? nil))
       ))))

  (:joint-state-subscriber2
   (msg)
   (format t "[joint-state-subscriber] called ")
   (bench
    (let ((joint-names (send msg :name))
	  (joint-angles (send msg :position))
	  joint-name joint-angle
	  buf
	  )
      ;;(describe msg)
      (dotimes (x (length joint-names))
	(setq joint-name (elt joint-names x))
	;;(print joint-name)
	(setq joint-angle (rad2deg (elt joint-angles x)))
	(setq buf
	      (intern (string-upcase joint-name) *keyword-package*))
	(if (find-method *robot* buf)
	    (send *robot* buf :joint-angle joint-angle))
	)
      (send *robot*
	    :fix-leg-to-coords (make-coords) :both)
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )))

  (:joint-state-subscriber
   (msg)
   (format t "[joint-state-subscriber] called ")
   (bench
    (let ((joint-names (send msg :name))
	  (joint-angles (send msg :position))
	  joint-name joint-angle
	  buf
	  )
      ;;(describe msg)
      (dotimes (x (length joint-names))
	(setq joint-name (elt joint-names x))
	;;(print joint-name)
	(setq joint-angle (rad2deg (elt joint-angles x)))
	(setq buf
	      (intern (string-upcase joint-name) *keyword-package*))
	(if (find-method *real-robot* buf)
	    (send *real-robot* buf :joint-angle joint-angle))
	;; (if (find-method *sandia_hand_left* buf)
	;;     (send *sandia_hand_left* buf :joint-angle joint-angle))
	;; (if (find-method *sandia_hand_right* buf)
	;;     (send *sandia_hand_right* buf :joint-angle joint-angle))
	)
      (send *real-robot*
	    :fix-leg-to-coords (make-coords) :both)
      (case *orgin-key*
	(:free nil)
	(t 
	 (let ((from-frame *robot-origin*)
	       (to-frame (if (eq *orgin-key* :rarm) "right_grasp_frame" "left_grasp_frame")))
	   (let ((trans (send *tfl* :lookup-transform ;pelvis -> grasp_frame
			      from-frame to-frame (ros::time 0.0))))
	     (if trans
		 (progn
		   (send trans :transform 
			 (send (make-coords) :transformation ;origin -> pelvis
			       (send *robot* :copy-worldcoords))
			 :world)
		   (send *origin-coords* :move-to trans :world))
	       (send *origin-coords* :move-to
		     (send *robot* *orgin-key* :end-coords :worldcoords)
		     :world)
	       )))
	 ;; (send *origin-coords* :move-to
	 ;; 	    (send *real-robot* *orgin-key*
	 ;; 		  :end-coords :copy-worldcoords)
	 ;; 	    :world)
	 ))
      
      (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      )))

  ;; (ros::tf-point->tf-pose (point)
  ;; (let ((pose (instance geometry_msgs::PoseStamped :init
  ;;                       :header (instance std_msgs::header :init
  ;;                                         :frame_id (send point :header :frame_id)
  ;;                                         :stamp (send point :header :stamp)))))
  ;;   (send pose :pose :position :x (send point :point :x))
  ;;   (send pose :pose :position :y (send point :point :y))
  ;;   (send pose :pose :position :z (send point :point :z))
  ;;   (send pose :pose :orientation :w 1.0)
  ;;   pose))

  (:waist-fix
   (&key
    (waist-key :waist-r)
    (waist-joint (send *robot* :torso waist-key))
    (waist-angle (send waist-joint :joint-angle))
    (angle-target 0)
    (prev-waist-angle waist-angle)
    (move-step 1)
    (target-coords
     (send *robot* *move-limb*
	   :end-coords :copy-worldcoords))
    (cnt 10)
    (ret #F()))
   (while (and (vectorp ret)
	       (> (abs (- waist-angle angle-target))
		  move-step)
	       (plusp (decf cnt)))
     (x::window-main-one)
     (setq prev-waist-angle waist-angle)
     (setq waist-angle
	   (* (if (> waist-angle 0) 1 -1)
	      (- (abs waist-angle) move-step)))
     (send waist-joint :joint-angle waist-angle)
     (setq ret (send self :call-ik-server ;:target-coords target-coords
		     :rotate? :x)))
   (if (not (vectorp ret))
       (send waist-joint :joint-angle prev-waist-angle)))

  (:hand-even-fix
   (&key
    (rpy-id 2)
    (rpy-target 0)
    (cnt 10)
    (rpy-step (deg2rad 3))
    )
   (let* (now-rpy
	  now-target
	  ret)
     (do-until-key
      (x::window-main-one)
      (setq now-rpy
	    (car (send *gun-coords* :rpy-angle)))
      (setq now-target (nth rpy-id now-rpy))
      (if (< (abs (- now-target rpy-target)) rpy-step)
	  (return-from nil nil))
      (if (< (decf cnt) 0) (return-from nil nil))
      (setq index -1)
      (setq
       ret
       (send self :call-ik-server
	     :move-coords
	     (make-coords
	      :rpy
	      (mapcar
	       #'(lambda (rpy)
		   (cond
		    ((not (eq (incf index) rpy-id)) 0)
		    ((< (- rpy-target now-target) 0)
		     (* -1 rpy-step))
		    (t rpy-step)))
	       now-rpy))))
      (if (not ret) (return-from nil nil)))))

  (:base-fix
   (&key
    (base-id 0)
    (base-rpy
     (car (send *robot* :rpy-angle)))
    (prev-base-rpy base-rpy)
    (move-step
     (let ((index -1))
       (mapcar
	#'(lambda (hoge)
	    (if (eq (incf index) base-id)
		(deg2rad 1) 0))
	(make-list 3))))
    (cnt 10)
    (ret #F())
    buf)
   (while (and (vectorp ret)
	       (> (abs (nth base-id base-rpy))
		  (nth base-id move-step))
	       (plusp (decf cnt)))
     (setq prev-base-rpy base-rpy)
     (setq base-rpy
	   (mapcar
	    #'(lambda (rpy step)
		(* (if (> rpy 0) 1 -1)
		   (- (abs rpy) step)))
	    base-rpy move-step))
     (send *robot* :newcoords
	   (make-coords
	    :pos (copy-object (send *robot* :worldpos))
	    :rpy (copy-object base-rpy)))
     (send *viewer* :draw-objects)
     (read-line)
     (setq ret (send self :call-ik-server
		     :rotate? :x :debug-view nil)))
   (if (not (vectorp ret))
       (send *robot* :newcoords
	     (make-coords
	      :pos (copy-object (send *robot* :worldpos))
	      :rpy (copy-object prev-base-rpy)))))

  ;;Interactive Marker Callback
  (:marker-menu-callback
   ( msg )
   (let ((menu (send msg :menu))
	 (type (send msg :type)))
     (cond
      ((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
       (format t "[marker-menu] move robot!!~%")
       (send self :publish-joint-states :real? t))
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN*)
       (setq *orgin-key* :free)
       (send self :set-origin)
       ;;(send self :publish-joint-states :real? nil)
       )
      ;;reset target coords
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
       (send *target-coords* :move-to *old-coords* :world)
       (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
       )
      ;;set origin to hand
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_RHAND*)
       (let ((message "set origin to rhand"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       ;;(setq *orgin-key* :free)
       (setq *orgin-key* :rarm)
       (send self :set-origin-to-hand :rarm)
       ;;(send self :publish-joint-states :real? nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_LHAND*)
       (let ((message "set origin to lhand"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       ;;(setq *orgin-key* :free)
       (setq *orgin-key* :larm)
       (send self :set-origin-to-hand :larm)
       ;;(send self :publish-joint-states :real? nil)
       )
      ;;change ik-rotation-axis
      ((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_T*)
       (let ((message "set ik mode 6D"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 (setq *rotation-axis* t)
	 )
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*IK_ROTATION_AXIS_NIL*)
       (let ((message "set ik mode 3D"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 (setq *rotation-axis* nil)
	 )
       )
      ;;change use-torso
      ((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_T*)
       (setq *use-torso* t)
       (setq *use-fullbody* nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*USE_TORSO_NIL*)
       (setq *use-torso* nil)
       (setq *use-fullbody* nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*USE_FULLBODY*)
       (setq *use-torso* nil)
       (setq *use-fullbody* t)
       )
      ;;changeControllArm
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_RARM*)
       (let ((message "move rarm"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *move-limb* :rarm)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_LARM*)
       (let ((message "move larm"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *move-limb* :larm)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_ARMS*)
       (let ((message "move both arms"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *move-limb* :arms)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*PUBLISH_MARKER*)
       (send self :publish-target-arrow-obj)
       (send self :publish-origin-arrow-obj)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*PLAN*)
       (let ((message "start IK"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *ik-stop* nil)
       (setq *publish-ik* t)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*CANCEL_PLAN*)
       (let ((message "stop IK"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *ik-stop* t)
       (setq *publish-ik* nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*TOUCHIT_EXEC*)
       (format t "[marker-menu] touch it exec~%")
       (send self :call-touch-it-server :exec)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*TOUCHIT_PREV*)
       (format t "[marker-menu] touch it prev~%")
       ;;(send self :call-touch-it-server :prev)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*TOUCHIT_CANCEL*)
       (format t "[marker-menu] touch it cancel~%")
       (send self :call-touch-it-server :cancel)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*LOOK_RARM*)
       (format t "[marker-menu] look rarm~%")
       (look-hand :rarm :rotate 0)
       (send self :publish-joint-states :real? nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*LOOK_LARM*)
       (format t "[marker-menu] look larm~%")
       (look-hand :larm :rotate 0)
       (send self :publish-joint-states :real? nil)
       )
      )
     (when (boundp '*irtviewer*)
       (send *irtviewer* :draw-objects))
     )
   (send self :publish-arrow-tf)
   )

  (:marker-menu-callback2
   ( msg )
   (let ((menu (send msg :menu))
	 (type (send msg :type)))
     (cond
      ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
       (format t "[marker-menu] move robot!!~%")
       (send self :publish-joint-states :real? t))
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
       (format t "[marker-menu] move robot!!~%")
       (send self :realmodel2model)
       (send self :publish-joint-states :real? nil))
      ((eq menu 100)
       (format t "[marker-menu] stand-pose~%")
       (send *robot* :stand-pose)
       (send self :publish-joint-states :real? nil))
      ((eq menu 101)
       (format t "[marker-menu] reset-manip-pose~%")
       (send *robot* :reset-manip-pose)
       (send self :publish-joint-states :real? nil))
      (t (format t "[marker-menu] unknonw(~A)~%" menu))
      )
     ))

  ;;called when marker is moved
  (:pose-callback
   (msg)
   (let* ((pose (send msg :pose))
	  (cds (ros::tf-pose-stamped->coords pose))
	  map->marker
	  )
     (while t
       (send pose :header :stamp (ros::time 0.0)) ;cheat!
       (setq map->marker
	     (send *tfl* :lookup-transform
		   *robot-origin* (send pose :header :frame_id) (ros::time 0)))
       ;;(setq map->marker (send *tfl* :transform-pose "pelvis" pose))
       (cond
	(map->marker
	 (send cds :transform map->marker :world)
	 (return)
	 )
	(t
	 (ros::ros-info "map -> marker cannnot transform yet")
	 (ros::sleep)
	 )
	)
       )
     (send *target-coords* :move-to
	   (send *robot* :worldcoords) :world)
     (send *target-coords* :transform cds)
     (cond
      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*SPHERE_MARKER*)
       (setq *tmp-rotation-axis* nil))
      (t
       (setq *tmp-rotation-axis* t)))
     (when (boundp '*irtviewer*)
       (send *irtviewer* :draw-objects))
     )
   (send self :publish-arrow-tf)
   (ros::ros-error "~A" *origin-coords*)
   )

  (:tf-end-coords
   (&optional (arm :rarm))
   (let* ((from-frame *robot-origin*)
	  (to-frame (if (eq arm :rarm) "right_grasp_frame" "left_grasp_frame"))
	  (trans (send *tfl* :lookup-transform ;pelvis -> grasp_frame
		       from-frame to-frame (ros::time 0.0))))
     (cond
      (trans
       (send trans :transform
	     (send (make-coords) :transformation ;origin -> pelvis
		   (send *real-robot* :copy-worldcoords))
	     :world)
       trans)
      (t (send *real-robot* arm :end-coords :copy-worldcoords)))))

  (:tf-model-end-coords
   (&key
    (robot *robot*)
    (limb :rarm)
    )
   (let* ((real-end-coords (send self :tf-end-coords limb))
	  (wrist2end (send (copy-object
			    (send
			     (send *real-robot* limb :end-coords :parent)
			     :worldcoords))
			   :transformation
			   real-end-coords)))
     (send (copy-object
	    (send
	     (send *robot* limb :end-coords :parent)
	     :worldcoords))
	   :transform wrist2end :local)))

  (:set-origin
   ()
   (send self :realmodel2model)
   (setq *old-coords* (send *target-coords* :copy-worldcoords))
   (send *origin-coords* :move-to *target-coords* :world)
   )

  (:set-origin-to-hand
   (&optional (arm :rarm))
   (send self :realmodel2model)
   (send *target-coords* :move-to
	 (send self :tf-end-coords arm)
	 :world)
   (send self :set-origin)
   (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
   )

  (:realmodel2model
   ()
   (send *robot*
	 :angle-vector
	 (copy-object
	  (send *real-robot* :angle-vector)))
   (send *robot* :fix-leg-to-coords
	 (make-coords) :both)
   (send self :fix-centroid-object)
   )

  ;;fixed flamae is foot?
  (:get-base-coords
   ()
   (let (coords torso->map)
     (setq coords (send *robot* :copy-worldcoords))
     (while t
       (setq torso->map
	     (send *tfl* :lookup-transform
		   ;;"r_foot" "map" (ros::time 0)))
		   *robot-origin* "map" (ros::time 0)))
       (cond
	(torso->map
	 (send coords :transform torso->map)
	 (return)
	 )
	(t
	 (ros::ros-info "~A -> map cannnot transform yet" *robot-origin*)
	 (ros::sleep)
	 )
	)
       )
     coords
     ;; (make-coords
     ;;  :pos
     ;;  (v+ (send coords :worldpos)
     ;; 	 (v- (send *robot* :worldpos)
     ;; 	     (send (send *robot* :r_foot_lk :worldcoords) :worldpos)))
     ;;  :rot
     ;;  (m*
     ;;   (transpose (send coords :worldrot))
     ;;   (m*
     ;;    (transpose
     ;; 	(send (send *robot* :r_foot_lk :worldcoords) :worldrot))
     ;;    (send *robot* :worldrot)))
     ;;  )
     )
   )

  (:set-marker-pose
   (coords &key (marker-name) (frame "/map") (markers) (server *server-nodename*))
   (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame)))
     (send req :marker_name marker-name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_pose" server)
			req)
     ))

  (:set-marker-pose-robot
   (coords &rest args)
   (if args
       (apply #'send self :set-marker-pose
	      (append
	       (list
		(send (send self :get-base-coords)
		      :transformation coords :local)
		args)))
     (send self :set-marker-pose
	   (send (send self :get-base-coords) :transformation coords :local))
     ))

  (:publish-arrow-tf
   ()
   (send *tfb* :send-transform
	 (send (send self :get-base-coords) :transformation
	       *target-coords* :local)
	 "/map" "/im_target_coords_arrow")
   ;;
   (send *tfb* :send-transform
	 (send (send self :get-base-coords) :transformation
	       *origin-coords* :local)
	 "/map" "/im_origin_coords_arrow")
   )

  (:publish-target-arrow-obj
   ()
   (let ((org  (send *target-coords* :copy-worldcoords)))
     (send *target-coords* :reset-coords)
     (send *target-coords* :worldcoords)
     (send-all (send *target-coords* :bodies) :worldcoords)
     (ros::publish (format nil "~A/marker" *server-nodename*)
		   (object->marker-msg *target-coords*
				       (instance std_msgs::header :init
						 :frame_id "/im_target_coords_arrow")))
     (send *target-coords* :transform org)
     (send *target-coords* :worldcoords)
     )
   )

  (:publish-origin-arrow-obj
   ()
   (let ((org  (send *origin-coords* :copy-worldcoords)))
     (send *origin-coords* :reset-coords)
     (send *origin-coords* :worldcoords)
     (send-all (send *origin-coords* :bodies) :worldcoords)
     (ros::publish (format nil "~A/origin_marker" *server-nodename*)
		   (object->marker-msg *origin-coords*
				       (instance std_msgs::header :init
						 :frame_id
						 "/im_origin_coords_arrow")))
     (send *origin-coords* :transform org)
     (send *origin-coords* :worldcoords)
     )
   )

  (:publish-menu-select
   (text &key (scale 0.15))
   (let ((marker (instance visualization_msgs::marker :init)))
     (send marker :header :frame_id "/im_target_coords_arrow")
     (send marker :id 2)
     (send marker :lifetime (ros::time 5))
     (send marker :pose :position :z 1.05)
     (send marker :pose :orientation :w 1)
     (send marker :color :r 1)
     (send marker :color :g 1)
     ;;(send marker :color :b 1)
     (send marker :color :a 1)
     (send marker :scale :z scale)
     (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
     (send marker :text text)
     (ros::publish (format nil "~A/marker" *server-nodename*) marker)
     )
   )

  (:publish-touchit-status
   (text &key (scale 0.15))
   (let ((marker (instance visualization_msgs::marker :init)))
     (send marker :header :frame_id "/im_target_coords_arrow")
     (send marker :id 1)
     (send marker :lifetime (ros::time 5))
     (send marker :pose :position :z 1.05)
     (send marker :pose :orientation :w 1)
     (send marker :color :r 1)
     (send marker :color :g 1)
     ;;(send marker :color :b 1)
     (send marker :color :a 1)
     (send marker :scale :z scale)
     (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
     (send marker :text text)
     (ros::publish (format nil "~A/marker" *server-nodename*) marker)
     )
   )

  (:publish-joint-states
   (&key
    (real? nil))
   (let ((joint-list
	  (append (send *robot* :joint-list)
		  ;;(send *sandia_hand_right* :joint-list)
		  ;;(send *sandia_hand_left* :joint-list)
		  ))
	 (joint-state-msg
	  (instance sensor_msgs::JointState :init
		    :header (instance std_msgs::header :init
				      :stamp (ros::time-now)))))
     (send joint-state-msg :position
	   (mapcar #'(lambda (joint) (send joint :ros-joint-angle)) joint-list))
     (send joint-state-msg :name (send-all joint-list :name))
     ;;
     (cond
      (real?
       (ros::publish (format nil "~A/~A/joint_states_ri_move" *im-nodename* *robot-name*)
		     joint-state-msg))
      (t
       (ros::publish (format nil "~A/~A/reset_joint_states" *im-nodename* *robot-name*)
		     joint-state-msg)))
     )
   )

   (:make-centroid-object
    ()
    (setq *centroid-object* (make-cylinder 20 400))
    (send *centroid-object* :set-color #F(1 1 0)))
   
   (:fix-centroid-object
    ()
    )


  (:init 
   ()
   (defvar *target-coords* (instance arrow-object :init))
   (send *target-coords* :translate #f(500 0 0)) ;; initial pose
   (defvar *origin-coords* (instance arrow-object :init)) ;;arrow for origin of object
   (send *origin-coords* :translate #f(500 0 0))
   (defvar *old-coords* (send *target-coords* :copy-worldcoords))
   (defvar *use-fullbody* nil)
   (defvar *use-torso* nil)
   (defvar *translation-axis* t)
   (defvar *rotation-axis* nil)
   (defvar *tmp-rotation-axis* t) ;;turn nil when center sphere was moved
   (defvar *publish-ik* nil)
   (defvar *orgin-key* :rarm)
   (defvar *ik-stop* (not *publish-ik*))

   (defvar *robot-name* (ros::get-param "~robot" "PR2"))

   ;;init robot
   (cond
    ((equal (string-upcase *robot-name*) "PR2")
     (setq *robot-name* "PR2")
     (setq *PR2* (pr2))
     (setq *robot* *PR2*)
     (defvar *robot-origin* "base_link")
     )
    ((equal (string-upcase *robot-name*) "ATLAS")
     (setq *robot-name* "atlas")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-end-coords.l")

     (atlas-with-hand)
     (setq *robot* *atlas*)
     (defvar *robot-origin* "pelvis")

     )

    (t
     (ros::ros-error "undefined robot: ~A" *robot-name*)
     (ros::ros-error "use PR2")
     (setq *PR2* (pr2))
     (setq *robot* *PR2*)
     (defvar *robot-origin* "base_link")
     )
    )

   (defvar *im-nodename* "jsk_model_marker_interface")
   (defvar *server-nodename*
     "/jsk_interactive_marker_manipulation")

   ;;initialize joint-index-list
   ;;it is used to convert joint-name -> index
   ;;(joint-index-init)
   
   (ros::roseus "ik_controller")
   (setq *tfb* (instance ros::transform-broadcaster :init))
   (setq *tfl* (instance ros::transform-listener :init))

   ;;
   (ros::roseus-add-msgs "hrpsys_gazebo_atlas")
   (ros::roseus-add-msgs "sensor_msgs")
   ;;
   (ros::subscribe "/ik_server_command"
		   std_msgs::String
		   #'send self :str-subscriber
		   1)

   (ros::subscribe "/spacenav/joy"
		   sensor_msgs::Joy
		   #'send self :spacenav-subscriber
		   1)

   (ros::advertise (format nil "~A/marker" *server-nodename*)
		   visualization_msgs::Marker)
   (ros::advertise
    (format nil "~A/origin_marker" *server-nodename*)
    visualization_msgs::Marker)
   (ros::subscribe
    (format nil "~A/pose" *server-nodename*)
    jsk_interactive_marker::MarkerPose #'send self :pose-callback 1)
   (ros::subscribe
    (format nil "~A/marker_menu" *server-nodename*)
    jsk_interactive_marker::MarkerMenu #'send self :marker-menu-callback 1)
   (ros::subscribe
    (format nil "~A/marker_menu" *im-nodename*)
    jsk_interactive_marker::MarkerMenu #'send self :marker-menu-callback2 1)
   (ros::subscribe
    (format nil
	    "/~A/joint_states_decompressed" *robot-name*)
    sensor_msgs::JointState
    #'send self :joint-state-subscriber 1)

   (ros::subscribe
    "/multisense_sl/joint_states_decompressed"
    sensor_msgs::JointState
    #'send self :joint-state-subscriber 1)

   ;;Update Interactive Marker
   (ros::advertise
    (format nil "~A/~A/reset_joint_states" *im-nodename* *robot-name*)
    sensor_msgs::JointState)


   (ros::subscribe
    (format nil "~A/~A/joint_states" *im-nodename* *robot-name*)
    sensor_msgs::JointState #'send self :joint-state-subscriber2)

   (ros::advertise
    (format nil "~A/~A/joint_states_ri_move" *im-nodename* *robot-name*)
    sensor_msgs::JointState)

   ;;(switch-end-coords :id 2)
   (setq *real-robot* (copy-object *robot*))
   (gl::transparent *real-robot* 0.3)
   (setq *arrow* (arrow))
   (send self :make-centroid-object)

   (send self :init-pose)
   (send self :set-origin-to-hand *orgin-key*)

   (objects
    (list *robot* *real-robot* *origin-coords* *target-coords*))
   ;;
   (send *irtviewer* :change-background #f(0.9 0.9 1.0))
   (send *irtviewer* :title "Interactive IK")

   (send-all (send *robot* :joint-list) :max-joint-velocity 0.3)

   (defvar *mouse-mode* nil)
   (defvar *success-state-collection* nil)
   (defvar *move-limb* :rarm)

   (defvar *prev-js*
     (instance sensor_msgs::jointstate
	       :init
	       :header
	       (instance std_msgs::Header :init
			 :stamp (ros::time 0.0)
			 :frame_id (format nil "/~A" *robot-origin*))
	       :position
	       (map float-vector
		    #'deg2rad
		    (send *robot* :angle-vector))
	       :name
	       (send-all (send *robot* :joint-list) :name)))

   (send *robot* :rleg :knee-p :min-angle 10)
   (send *robot* :lleg :knee-p :min-angle 10)

					;(send self :demo-mouse-controller)
   (send *robot* :angle-vector
	 (copy-object (send *real-robot* :angle-vector)))
   (send *real-robot*
	 :fix-leg-to-coords (make-coords) :both)
   (send *irtviewer* :draw-objects)
   (send *irtviewer* :look-all)

   (ros::rate 100)
   (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
   (send self :demo-pos-controller)

   )

  (:demo-mouse-controller
   nil
   (ros::rate 10)
   (do-until-key
    (if (not (ros::ok)) (return-from nil nil))
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)))

  (:demo-pos-controller
   (&key
    (target-limb
     (case *move-limb*
       (:arms '(:rarm :larm))
       (t (list *move-limb*))))
    (now
     (mapcar
      #'(lambda (k)
	  (send *robot* k
		:end-coords :copy-worldcoords))
      target-limb))
    (goal (copy-object now))
    (now-from-pelvis
     (mapcar
      #'(lambda (now goal)
	  (send (send *robot* :copy-worldcoords) :transformation now))
      now goal))
    (goal-from-pelvis
     (mapcar
      #'(lambda (now goal)
	  (send (send *robot* :copy-worldcoords) :transformation goal))
      now goal))
    (real? nil)
    (real-feedback? nil)
    (cnt 1000)
    (non-stop t)
    (return? nil)
    (revert-if-fail t)
    ;;
    (max-step 1000)
    (max-rstep (deg2rad 180))
    log-string
    )
   (if return?
       (setq return?
	     (length *success-state-collection*)))
   (let (ret move rot buf tmp str)
     (do-until-key
      (bench
       (if (and (not non-stop)
		(< (decf cnt) 0)) (return-from nil nil))
       (do-until-key
	(x::window-main-one)
	(ros::spin-once)
	(ros::sleep)
	(if (not *mouse-mode*) (return-from nil nil))
	)
       ;;
       ;;(send self :waist-fix :waist-key :waist-r :cnt 3)
       ;;(send self :waist-fix :waist-key :waist-p :cnt 3)
       ;;(send self :waist-fix :waist-key :waist-y :cnt 3)
       ;;(send self :hand-even-fix :cnt 1)
       ;;(dotimes (i 2)
       ;;(send self :call-ik-server :rotate? :x :log-stream nil))
       (cond
	(*ik-stop*
	 (format t "[demo-pos-controller] ik-stop")
	 (unix:usleep (round (* 1000 100))))
	(t
	 (setq
	  target-limb
	  (case *move-limb*
	    (:arms '(:rarm :larm))
	    (t (list *move-limb*))))
	 (setq
	  now
	  (mapcar
	   #'(lambda (k)
	       (send *robot* k
		     :end-coords :copy-worldcoords))
	   target-limb))
	 (setq
	  str
	  (mapcar
	   #'(lambda (k)
	       (send *real-robot* k
		     :end-coords :copy-worldcoords))
	   target-limb))
	 (setq
	  goal
	  (mapcar
	   #'(lambda (str)
	       (send *origin-coords* :transformation str))
	   str))
	 (send-all goal :transform *target-coords* :world)
	 (setq
	  now-from-pelvis
	  (mapcar
	   #'(lambda (now goal)
	       (send (send *robot* :copy-worldcoords) :transformation now))
	   now goal))
	 (setq
	  goal-from-pelvis
	  (mapcar
	   #'(lambda (now goal)
	       (send (send *robot* :copy-worldcoords) :transformation goal))
	   now goal))
	 (setq
	  move
	  (mapcar
	   #'(lambda (now-from-pelvis goal-from-pelvis)
	       (transform
		(send now-from-pelvis :worldrot)
		(send now-from-pelvis :difference-position
		      goal-from-pelvis)))
	   now-from-pelvis goal-from-pelvis))
	 (setq
	  rot
	  (mapcar
	   #'(lambda (now-from-pelvis goal-from-pelvis)
	       (transform
		(send now-from-pelvis :worldrot)
		(send now-from-pelvis :difference-rotation
		      goal-from-pelvis)))
	   now-from-pelvis goal-from-pelvis))
	 (setq log-string
	       (format nil
		       ":dif-pos ~4,2f :dif-rot ~4,2f"
		       (car (mapcar #'norm move))
		       (car (mapcar #'norm rot))))
	 (format t "[pos-controller] ~A" log-string)
	 (if (and (not non-stop)
		  (< (apply #'max (mapcar #'norm move)) 10)
		  (< (apply #'max (mapcar #'norm rot)) (deg2rad 5)))
	     (return-from nil t))
	 (setq move
	       (mapcar
		#'(lambda (move)
		    (scale
		     (min max-step (norm move))
		     (normalize-vector move)))
		move))
	 (setq rot
	       (mapcar
		#'(lambda (rot)
		    (scale (min max-rstep (norm rot))
			   (normalize-vector rot)))
		rot))
	 (mapcar
	  #'(lambda (limb move rot)
	      (setq tmp (send self :call-ik-server
			      :target-limb limb
			      :real? real?
			      :move-coords
			      (make-coords
			       :pos move
			       :rpy (reverse rot))
			      :rotate? :x
			      :revert-if-fail revert-if-fail
			      :log-stream nil
			      )))
	  target-limb move rot)
	 (cond
	  ((vectorp tmp)
	   (x::window-main-one)
	   (setq max-step
		 (min 1000 (* max-step 2)))
	   (setq max-rstep
		 (min (deg2rad 180)
		      (* max-rstep 2)))
	   (setq revert-if-fail t)
	   (send self :publish-touchit-status (format nil "~%IK SUCC / ~A" log-string))
	   (if *publish-ik*
	       (send self :publish-joint-states :real? nil)))
	  (t ;(evenp (random 2))
	   (setq max-step
		 (max 10 (/ max-step 2.0)))
	   (setq max-rstep
		 (max (deg2rad 10) (/ max-rstep 2.0)))
	   (setq revert-if-fail t)
	   (send self :publish-touchit-status (format nil "~%IK FAIL / ~A" log-string))
	   ;; (mapcar
	   ;;  #'(lambda (j)
	   ;;      (send j :joint-angle
	   ;; 	     (+ (send j :joint-angle)
	   ;; 		(* (- (* 2 (random 0.01)) 0.01)
	   ;; 		   (- (send j :max-angle)
	   ;; 		      (send j :min-angle))))))
	   ;;  (if (find-method *robot* *move-limb*)
	   ;;      (send *robot* *move-limb* :joint-list)))
	   ;; (send *viewer* :draw-objects)
	   ;; (setq tmp (send self :call-ik-server
	   ;; 	   :real? real?
	   ;; 	   :real-feedback? real-feedback?
	   ;; 				;:rotate? :x
	   ;; 	   :move-coords
	   ;; 	   (make-coords
	   ;; 	    :pos (random-vector 30.0)
	   ;; 	    :rpy (random-vector (deg2rad 10)))
	   ;; 	   :log-stream nil
	   ;; 	   ))
	   ))
	 )))))
   (if return?
       (subseq *success-state-collection*
	       0
	       (- (length *success-state-collection*)
		  return?))
     t)
   )

  ;; export ROS_MASTER_URI=http://10.10.72.112:11311
  ;; export ROS_IP=10.10.72.109 && export ROS_HOSTNAME=$ROS_IP
  ;; load "ssc-20131118.l"
  ;; load "ssc-20131119.l"
  ;; load "ssc-20131119-3.l"
  (:demo-collection-play
   (&key
    (real? nil)
    (data (reverse *success-state-collection*))
    (inital-sleep (if real? 10000 100))
    (cnt -1)
    d)
   ;; (cond
   ;;  (real?
   ;;   (init-grasp-pose)
   ;;   (model2real :sleep-time inital-sleep :wait t)))
   ;;
   (do-until-key
    (print (incf cnt))
    (if (null data) (return-from nil t))
    (setq d (car data))
    (setq data (cdr data))
    (send *robot* :angle-vector (cdr (assoc :angle-vector d)))
    (send *robot* :newcoords (cdr (assoc :worldcoords d)))
    (send *viewer* :draw-objects)
    (x::window-main-one)
    (cond
     (real?
      (model2real :sleep-time inital-sleep :wait t))
     (t (unix:usleep (round (* inital-sleep 1e+3)))))
    (setq inital-sleep 100))
   )
  )

(setq *im-controller* (instance im-controller :init))


#|

(ros::subscribe
 (format nil "~A/pose" server-nodename)
 jsk_interactive_marker::MarkerPose #'send self :pose-callback)

(ros::subscribe
 (format nil "~A/marker_menu" server-nodename)
 jsk_interactive_marker::MarkerMenu #'send self :move-callback)
