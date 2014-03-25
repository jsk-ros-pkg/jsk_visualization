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
(require "package://jsk_ik_server/euslisp/fullbody-ik-client.l")

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
   *lhand-frame*
   *rhand-frame*
   *fullbody-ik-client*
   *base-target-coords*
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
   (&rest
    args
    &key
    (frame-id (format nil "/~A" *robot-origin*))
    (move-limbs '(t t))
    (target-limbs '(:larm :rarm))
    (move-coords (make-coords))
    (move-target (mapcar #'(lambda (k) (send *real-robot* k :end-coords)) target-limbs))
    (initial-coords
     (mapcar #'(lambda (target-limb)
		 (send *real-robot* target-limb :end-coords :copy-worldcoords))
	     target-limbs))
    (target-coords
     (mapcar #'(lambda (move-limb initial-coord)
		 (send (send *real-robot* :copy-worldcoords) :transformation
		       (if move-limb
			   (make-coords
			    :pos (v+ (send initial-coord :worldpos)
				     (send move-coords :worldpos))
			    :rot (m* (send move-coords :worldrot)
				     (send initial-coord :worldrot)
				     ))
			 initial-coord))) move-limbs initial-coords))
    (prev-state
     (list (cons :angle-vector
		 (copy-seq (send *robot* :angle-vector)))
	   (cons :worldcoords
		 (copy-object (send *robot* :worldcoords)))))
    (log-stream t)

    (translation-axis
     (mapcar #'(lambda (move-limb) (if move-limb *translation-axis* t)) move-limbs))
    (rotation-axis
     (mapcar #'(lambda (move-limb) (if move-limb (if *tmp-rotation-axis* *rotation-axis* nil) t)) move-limbs))
    (revert-if-fail t)
    (update-convex t)
    ik-res
    &allow-other-keys
    )
   (print move-limbs)
   (print rotation-axis)
   (print "initial-coords")
   (print initial-coords)
   (print "target-coords")
   (print target-coords)
   (print "move-coords")
   (print move-coords)


   ;;send ik service
   (setq ik-res
	 (apply
	  #'send *fullbody-ik-client* :ik-server-call
	  (append
	   (list
	    :robot *robot*
	    :frame-id frame-id
	    :group-name ":inverse-kinematics"
	    :move-target move-target
	    :target-limbs target-limbs
	    :target-coords initial-coords
	    :root-link-virtual-joint-weight #F(1 1 1 1 1 1)
	    :stop 50
	    :use-toes '(nil nil)
	    :centroid-thre nil
	    :target-centroid-pos nil
	    :cog-gain 0
	    :rotation-axis rotation-axis
	    :translation-axis translation-axis
	    )
	   args)))
   (setq ik-res-tmp ik-res)

   (cond
    ((= (send ik-res :error_code :val)
	moveit_msgs::MoveItErrorCodes::*SUCCESS*)
     (format log-stream "successed!~%")
     (setq *prev-js* (send ik-res :solution :joint_state))
     (send *robot* :angle-vector
	   (map float-vector
		#'rad2deg
		(send ik-res :solution :joint_state :position)))
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
    (t (format log-stream "failed!~%") nil)))


  
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
	(let ((joint-method (intern (string-upcase joint-name) *keyword-package*)))
	  (when (find-method *robot* joint-method)
	    (if (subclassp (class (send *robot* joint-method)) rotational-joint)
		(setq joint-angle (rad2deg (elt joint-angles x)))
	      (setq joint-angle (* 1000.0 (elt joint-angles x))))
	    (send *robot* joint-method :joint-angle joint-angle))))
      (send *robot* :fix-leg-to-coords (make-coords) :both)
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
  #|
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
  |#
  #|
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

  |#
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
       (setq *orgin-key* :rarm)
       (send self :set-origin-to-hand :rarm)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_LARM*)
       (let ((message "move larm"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *move-limb* :larm)
       (setq *orgin-key* :larm)
       (send self :set-origin-to-hand :larm)
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

      ;;grasp
      ((eq menu jsk_interactive_marker::MarkerMenu::*START_GRASP*)
       (let ((message "start grasp"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (send self :start-grasp :arm *move-limb*)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*STOP_GRASP*)
       (let ((message "stop grasp"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (send self :stop-grasp :arm *move-limb*)
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
      ((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
       (format t "[marker-menu] move robot base!!~%")
       (send self :publish-base-target))
      ((eq menu jsk_interactive_marker::MarkerMenu::*JOINT_MOVE*)
       (format t "[marker-menu] move robot joint!!~%")
       (send self :publish-joint-states :real? t))
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
       (format t "[marker-menu] reset robot joint!!~%")
       (send self :realmodel2model)
       (send self :publish-joint-states :real? nil))
      ((eq menu 100)
       (format t "[marker-menu] stand-pose~%")
       (if (find-method *robot* :stand-pose)
	   (send *robot* :stand-pose))
       (send self :publish-joint-states :real? nil))
      ((eq menu 101)
       (format t "[marker-menu] reset-manip-pose~%")
       (if (find-method *robot* :reset-manip-pose)
	   (send *robot* :reset-manip-pose))
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
	  (to-frame (if (eq arm :rarm) *rhand-frame* *lhand-frame*))
	  (trans nil))
     (if to-frame
         (setq trans (send *tfl* :lookup-transform ;pelvis -> grasp_frame
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
   (let (res)
     (setq res (ros::service-call (format nil "~A/~A/get_joint_states" *im-nodename* *robot-name*) (instance jsk_interactive_marker::GetJointStateRequest :init)))

     (let* ((res-joint-state (send res :joint_state))
	    (joint-names (send res-joint-state :name))
	    (joint-angles (send res-joint-state :position))
	    joint-name joint-angle
	    )
       (dotimes (x (length joint-names))
	 (setq joint-name (elt joint-names x))
	 (let ((joint-method (intern (string-upcase joint-name) *keyword-package*)))
	   (when (find-method *real-robot* joint-method)
	     (if (subclassp (class (send *robot* joint-method)) rotational-joint)
		 (setq joint-angle (rad2deg (elt joint-angles x)))
	       (setq joint-angle (* 1000.0 (elt joint-angles x)))))
	   (send *real-robot* joint-method :joint-angle joint-angle)))
       )

     (send *robot*
	   :angle-vector
	   (copy-object
	    (send *real-robot* :angle-vector)))
     (send *robot* :fix-leg-to-coords
	   (make-coords) :both)
     (send self :fix-centroid-object)
     )
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

  (:publish-base-target
   ()
   (when *base-target-coords*
     (let ((base-msg (ros::coords->tf-pose-stamped *base-target-coords* (send *base-target-coords* :name))))
       (ros::publish (format nil "~A/~A/base_move" *im-nodename* *robot-name*)
		     base-msg)
       )
     )
   )

  (:base-target-callback
   (msg)
   (setq *base-target-coords* (ros::tf-pose-stamped->coords msg))
   (print "update target")
   )



  (:start-grasp
   (&key (arm :arms))
   (send self :send-grasp-msg "start-grasp" :arm arm)
   )

  (:stop-grasp
   (&key (arm :arms))
   (send self :send-grasp-msg "stop-grasp" :arm arm)
   )

  (:send-grasp-msg
   (pose-data &key (arm :arms))
   (let ((hand-msg (instance std_msgs::String :init)))
     (send hand-msg :data (format nil "~A_~A" pose-data arm))
     (ros::publish (format nil "~A/~A/hand_ri_move" *im-nodename* *robot-name*)
		   hand-msg)
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

   (defvar *rhand-frame* nil)
   (defvar *lhand-frame* nil)

   (defvar *start-loop* t)
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
   (setq *fullbody-ik-client* (instance fullbody-ik-client :init))
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
   
   ;;update real-robot joint state
   ;;   (ros::advertise
   ;;    (format nil "~A/~A/send_joint_states" *im-nodename* *robot-name*)
   ;;    sensor_msgs::JointState)

   (ros::subscribe
    (format nil "~A/~A/joint_states" *im-nodename* *robot-name*)
    sensor_msgs::JointState #'send self :joint-state-subscriber2)

   (ros::advertise
    (format nil "~A/~A/joint_states_ri_move" *im-nodename* *robot-name*)
    sensor_msgs::JointState)

   (ros::advertise
    (format nil "~A/~A/hand_ri_move" *im-nodename* *robot-name*)
    std_msgs::String)

   (ros::advertise (format nil "~A/~A/base_move" *im-nodename* *robot-name*)
		   geometry_msgs::PoseStamped)

   (ros::subscribe
    (format nil "~A/~A/base_pose" *im-nodename* *robot-name*)
    geometry_msgs::PoseStamped #'send self :base-target-callback)


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
   ;;update *real-robot* and joint marker pose
   (send self :realmodel2model)
   (send self :publish-joint-states :real? nil)

   (if *start-loop* (send self :demo-pos-controller))

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
   nil
   (let (ret move rot buf tmp str)
     (do-until-key
      (bench
       (do-until-key
	(x::window-main-one)
	(ros::spin-once)
	(ros::sleep)
	(if (not *mouse-mode*) (return-from nil nil))
	)
       (cond
	(*ik-stop*
	 ;;(format t "[demo-pos-controller] ik-stop")
	 (unix:usleep (round (* 1000 100))))
	(t
	 (let ((move-arms '(t t))
	       (use-torso (or *use-torso* *use-fullbody*))
	       target-coords)
	   (setq move-arms 
		 (case *move-limb*
		   (:rarm '(nil t))
		   (:larm '(t nil))
		   (:arms '(t t))))
	   ;;ik target
	   (setq target-coords
		 (mapcar #'(lambda (arm move-arm)
			     (if move-arm
				 (let* ((origin->hand (send (send *origin-coords* :transformation (send *real-robot* arm :end-coords)) :copy-worldcoords))
					(hand (send origin->hand :transform *target-coords* :world)))
				   hand)
			       (send *real-robot* arm :end-coords)))
			 '(:larm :rarm) move-arms))

	   (setq tmp
		 (send *robot* :inverse-kinematics
		       target-coords
		       :move-arm :arms
		       :use-torso use-torso
		       :debug-view nil)
		 ))

	 #|

	 (defvar *pr2-move-target*
	 (list (make-cascoords
	 :parent
	 (send (send (send *real-robot* :larm :end-coords :parent) :parent) :parent)
	 :coords
	 (copy-object
	 (send (send (send (send *real-robot* :larm :end-coords :parent) :parent) :parent)
	 :worldcoords))
	 :name :larm-pr2-end-coords)
	 (make-cascoords
	 :parent
	 (send (send (send *real-robot* :rarm :end-coords :parent) :parent) :parent)
	 :coords
	 (copy-object
	 (send (send (send (send *real-robot* :rarm :end-coords :parent) :parent) :parent)
	 :worldcoords))
	 :name :rarm-pr2-end-coords)))
	 (mapcar
	 #'(lambda (limb move rot)
	 (setq tmp (send self :call-ik-server
	 :move-limbs
	 (case *move-limb*
	 (:arms '(t t))
	 (:larm '(t nil))
	 (:rarm '(nil t)))
	 ;;:target-limbs limb
	 ;;			      :target-limbs 
	 ;;			      (case *move-limb*
	 ;;				(:arms '(:larm :rarm))
	 ;;				(t (list *move-limb*)))
	 :target-limbs '(:larm :rarm)
	 :real? real?
	 :move-target *pr2-move-target*
	 :move-coords (make-coords)
	 :origin-coords (make-coords)
	 ;; (make-coords
	 ;;  :pos move
	 ;;  :rpy (reverse rot))
	 :rotate? :x
	 :revert-if-fail revert-if-fail
	 :log-stream nil
	 )))
	 target-limb move rot)
	 |#
	 
	 (cond
	  ((vectorp tmp)
	   (x::window-main-one)
	   (setq revert-if-fail t)
	   ;;(send self :publish-touchit-status (format nil "~%IK SUCC / ~A" log-string))
	   (send self :publish-touchit-status (format nil "~%IK SUCC "))
	   (if *publish-ik*
	       (send self :publish-joint-states :real? nil)))
	  (t ;(evenp (random 2))
	   (setq revert-if-fail t)
	   ;;(send self :publish-touchit-status (format nil "~%IK FAIL / ~A" log-string))
	   (send self :publish-touchit-status (format nil "~%IK FAIL "))
	   ))
	 ))
       )
      ))
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

;;(setq *im-controller* (instance im-controller :init))


#|

(ros::subscribe
 (format nil "~A/pose" server-nodename)
 jsk_interactive_marker::MarkerPose #'send self :pose-callback)

(ros::subscribe
 (format nil "~A/marker_menu" server-nodename)
 jsk_interactive_marker::MarkerMenu #'send self :move-callback)
