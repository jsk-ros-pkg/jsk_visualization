#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")

(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_footstep_msgs")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_interactive_marker")
(ros::load-ros-manifest "jsk_rviz_plugins")


(ros::roseus "ik-controller")

(require "models/arrow-object.l")
(require "package://roseus/euslisp/actionlib.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://jsk_ik_server/euslisp/fullbody-ik-client.l")
(require "package://jsk_interactive_marker/euslisp/robot-joint-interface.l")

;;;;;;;; setup ;;;;;;;;;;;
(defclass im-controller
  :slots
  (*robot*
   *lhand-frame*
   *rhand-frame*
   *fullbody-ik-client*
   *base-target-coords*
   *movingp*
   *have-leg*
   *frame-id*
   *ik-update-times*
   *target-update-times*
   *rji*
   )
  )

(defmethod im-controller
  ;; initial pose
  (:init-pose
   nil
   (send *robot* :reset-manip-pose)
   (send self :fix-robot-coords)
   (if (and (boundp '*viewer*) *viewer*)
       (send *viewer* :draw-objects))
   )

  (:fix-robot-coords
   nil
   (send *robot* :fix-leg-to-coords (make-coords) :both)
   (send *real-robot* :fix-leg-to-coords (make-coords) :both)
   )

  (:robot-marker-joint-state-subscriber
   (msg)
   (format t "[robot-marker-joint-state-subscriber] called ")
   (apply-joint_state msg *robot*)
   (send self :fix-robot-coords)
   (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
   )

  (:joint-state-subscriber
   (msg)
   (format t "[joint-state-subscriber] called ")
   (bench
    (apply-joint_state msg *real-robot*)
    (send self :fix-robot-coords)

    (case *origin-key*
      (:free nil)
      (t
       (let ((from-frame *robot-origin*)
	     (to-frame (if (eq *origin-key* :rarm) *rhand-frame* *lhand-frame*))
	     (trans nil))
	 (if to-frame
	     (setq (send *tfl* :lookup-transform ;pelvis -> grasp_frame
			 from-frame to-frame (ros::time 0.0))))
	 (if trans
	     (progn
	       (send trans :transform
		     (send (make-coords) :transformation ;origin -> pelvis
			   (send *real-robot* :copy-worldcoords))
		     :world)
	       (send *origin-coords* :move-to trans :world))
	   (send *origin-coords* :move-to
		 (send *real-robot* *origin-key* :end-coords :worldcoords)
		 :world)
	   )))
      )
    
    (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    ))

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
     (setq prev-waist-angle waist-angle)
     (setq waist-angle
	   (* (if (> waist-angle 0) 1 -1)
	      (- (abs waist-angle) move-step)))
     (send waist-joint :joint-angle waist-angle)
     (setq ret (send self :call-ik-server ;:target-coords target-coords
		     :rotate? :x)))
   (if (not (vectorp ret))
       (send waist-joint :joint-angle prev-waist-angle)))

  ;;Interactive Marker Callback
  (:marker-menu-callback
   ( msg )
   (let ((menu (send msg :menu))
	 (type (send msg :type)))
     (cond
      ((eq menu jsk_interactive_marker::MarkerMenu::*MOVE*)
       (format t "[marker-menu] move robot!!~%")
       (send self :publish-joint-states :real? t)
       ;;test
       ;;(send self :publish-joint-trajectory (list (send *robot* :angle-vector)) 5000 :moveit t :real? t)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN*)
       (setq *origin-key* :free)
       (send self :set-origin)
       ;;(send self :publish-joint-states :real? nil)
       )
      ;;reset target coords
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_COORDS*)
       (send *target-coords* :move-to *old-coords* :world)
       (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
       )
      ;;reset joint and base
      ((eq menu jsk_interactive_marker::MarkerMenu::*RESET_JOINT*)
       (format t "[marker-menu] reset robot joint!!~%")
       (send self :realmodel2model)
       (send self :publish-joint-states :real? nil)
       ;;reset root pose
       (ros::publish (format nil "~A/~A/reset_root_pose" *im-nodename* *robot-topic-name*)
		     (instance std_msgs::Empty :init))
       (case *move-limb*
	 (:arms
	  )
	 (t
	  (send self :set-origin-to-hand *move-limb*)
	  )))
      ;;set origin to hand
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_RHAND*)
       (let ((message "set origin to rhand"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       ;;(setq *origin-key* :free)
       (setq *origin-key* :rarm)
       (send self :set-origin-to-hand :rarm)
       ;;(send self :publish-joint-states :real? nil)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_ORIGIN_LHAND*)
       (let ((message "set origin to lhand"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       ;;(setq *origin-key* :free)
       (setq *origin-key* :larm)
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
       (setq *origin-key* :rarm)
       (send self :set-origin-to-hand :rarm)
       )
      ((eq menu jsk_interactive_marker::MarkerMenu::*SET_MOVE_LARM*)
       (let ((message "move larm"))
	 (send self :publish-menu-select message)
	 (format t "[marker-menu] ~A~%" message)
	 )
       (setq *move-limb* :larm)
       (setq *origin-key* :larm)
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
       (send self :publish-ik-status (format nil " IK STOP") :status :WARNING)
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
   (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
   (send self :publish-arrow-tf)
   (send self :publish-status)
   )

  (:robot-marker-menu-callback
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
      ))
   (send self :publish-status)
   )

  ;;called when marker is moved
  (:pose-callback
   (msg)
   (let* ((pose (send msg :pose))
	  (cds (ros::tf-pose-stamped->coords pose))
	  map->marker
	  )
     (setq cds (send self :get-eus-coords cds (send pose :header)))
     (send *target-coords* :move-to (send (car (send *real-robot* :links)) :copy-worldcoords) :world)
     (send *target-coords* :transform cds :local)

     ;;set *tmp-rotation-axis*
     (cond
      ((equal (send msg :type) jsk_interactive_marker::MarkerPose::*SPHERE_MARKER*)
       (setq *tmp-rotation-axis* nil))
      (t
       (setq *tmp-rotation-axis* t)))
     (when (boundp '*irtviewer*)
       (send *irtviewer* :draw-objects))
     )
   (send self :update-target-coords)
   )

  (:update-target-coords
   ()
   (send self :publish-arrow-tf)
   (send self :publish-hand-marker-pose)
   (setq *target-update-times* (1+ *target-update-times*))
   )

  (:publish-hand-marker-pose
   ()
   (let ((ps-msg (instance geometry_msgs::PoseStamped :init))
         hand-coords)

     (setq hand-coords (send (send self :get-base-coords) :transformation *target-coords* :local))
     (setq ps-msg (ros::coords->tf-pose-stamped hand-coords *frame-id*))
     (ros::publish (format nil "~A/l_gripper/set_pose" *gripper-im-nodename*)
                   ps-msg)
     )
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
     (setq res (ros::service-call (format nil "~A/~A/get_joint_states" *im-nodename* *robot-topic-name*) (instance jsk_interactive_marker::GetJointStateRequest :init)))
     (apply-joint_state (send res :joint_state) *real-robot*)
     (send *robot* :angle-vector (send *real-robot* :angle-vector))
     (send self :fix-robot-coords)
     )
   )

  ;;fixed flame is foot?
  (:get-base-coords
   ()
   (let (coords torso->map)
     ;;(setq coords (send *real-robot* :copy-worldcoords))
     (setq coords (send (car (send *real-robot* :links)) :copy-worldcoords))
     (while t
       (setq torso->map
	     (send *tfl* :lookup-transform
		   *robot-origin* *frame-id* (ros::time 0)))
       (cond
	(torso->map
	 (send coords :transform torso->map)
	 (return)
	 )
	(t
	 (ros::ros-info "~A -> ~A cannot transform yet" *robot-origin* *frame-id*)
	 (unix:sleep 1)
	 )
	)
       )
     coords
     )
   )

  (:set-marker-pose
   (coords &key (marker-name) (frame *frame-id*) (markers) (server *server-nodename*))
   (let ((req (instance jsk_interactive_marker::MarkerSetPoseRequest :init))
	 (msg (ros::coords->tf-pose-stamped coords frame))
	 (ps (instance geometry_msgs::PoseStamped :init)))
     (send req :marker_name marker-name)
     (send req :pose msg)
     (send req :markers markers)
     (ros::service-call (format nil "~A/set_pose" server)
			req)

     ;;publish set pose
     (send ps :pose msg)
     (send ps :header :frame_id frame)
     (ros::publish (format nil "~A/set_pose" *server-nodename*)
		   msg)     
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
	 *frame-id* "/im_target_coords_arrow")
   (send *tfb* :send-transform
	 (send (send self :get-base-coords) :transformation
	       *origin-coords* :local)
	 *frame-id* "/im_origin_coords_arrow")
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
     (send marker :color :a 1)
     (send marker :scale :z scale)
     (send marker :type visualization_msgs::marker::*TEXT_VIEW_FACING*)
     (send marker :text text)
     (ros::publish (format nil "~A/marker" *server-nodename*) marker)
     )
   )

  (:overlay-text-color
   (status &key (a 0.8))
   (let ((color (instance std_msgs::ColorRGBA :init)))
     (case status
       (:INFO
        (send color :r (/ 25 255.0))
        (send color :g (/ 255 255.0))
        (send color :b (/ 255 255.0))
        )
       (:WARNING
        (send color :r (/ 255 255.0))
        (send color :g (/ 255 255.0))
        (send color :b (/ 0 255.0))
        )
       (:ERROR
        (send color :r (/ 255 255.0))
        (send color :g (/ 0 255.0))
        (send color :b (/ 0 255.0))
        )
       )
     (send color :a a)
     color
     ))

  (:publish-ik-status
   (text &key (delete nil) (status :INFO) (text_size 20))
   (let ((text_msg (instance jsk_rviz_plugins::OverlayText :init)))
     (if delete
         (send text_msg :action jsk_rviz_plugins::OverlayText::*DELETE*)
       (send text_msg :action jsk_rviz_plugins::OverlayText::*ADD*)
       )
     (send text_msg :width 640)
     (send text_msg :height 100)
     (send text_msg :fg_color (send self :overlay-text-color status))
     (send text_msg :bg_color :a 0.2)
     (send text_msg :line_width 2)
     (send text_msg :text_size 64)
     (send text_msg :font "DejaVu Sans Mono")
     (send text_msg :text text)
     (ros::publish (format nil "~A/ik_status" *nodename*) text_msg)
     ))

  (:publish-status
   (&key (delete nil) (status :INFO) (text_size 20))
   (let ((text_msg (instance jsk_rviz_plugins::OverlayText :init))
         text)
     (setq text (concatenate string
                             (format nil " ik:            ~A~%" (if *ik-stop* "stop" "running"))
                             (format nil " move-arm:      ~A~%" *move-limb*)
                             (format nil " target-coords: ~A~%" *origin-key*)
                             (format nil " ik-mode:       ~A~%" (if *rotation-axis* "6D" "3D"))
                             (format nil " move-link:     ~A~%" (cond (*use-torso* "torso")
                                                                      (*use-fullbody* "fullbody")
                                                                      (t "arm")
                                                                      ))
                             (format nil " ik-update:     ~A~%" *ik-update-times*)
                             (format nil " target-update: ~A~%" *target-update-times*)
                             ))
     (if delete
         (send text_msg :action jsk_rviz_plugins::OverlayText::*DELETE*)
       (send text_msg :action jsk_rviz_plugins::OverlayText::*ADD*)
       )
     (send text_msg :width 400)
     (send text_msg :height 240)
     (send text_msg :top 104)
     (send text_msg :fg_color (send self :overlay-text-color status))
     (send text_msg :bg_color :a 0.2)
     (send text_msg :line_width 2)
     (send text_msg :text_size 20)
     (send text_msg :font "DejaVu Sans Mono")
     (send text_msg :text text)
     (ros::publish (format nil "~A/status" *nodename*) text_msg)
     ))

  (:publish-joint-states
   (&key
    (real? nil))
   (let ((joint-list
          (append (send *robot* :joint-list)
                  ))
         (joint-state-msg
          (instance sensor_msgs::JointState :init
                    :header (instance std_msgs::header :init
                                      :stamp (ros::time-now))))
         (set-pose-msg
          (instance geometry_msgs::PoseStamped :init
                    :header (instance std_msgs::header :init
                                      :stamp (ros::time-now)
                                      :frame_id *robot-origin*)))
         )
     (send joint-state-msg :position
           (mapcar #'(lambda (joint) (send joint :ros-joint-angle)) joint-list))
     (send joint-state-msg :name (send-all joint-list :name))

     ;;set set-pose-msg
     (send set-pose-msg :pose
           (ros::coords->tf-pose
            (send (send (car (send *real-robot* :links)) :copy-worldcoords) :transformation
                  (send (car (send *robot* :links)) :copy-worldcoords)
                  :local)))

     (cond
      (real?
       (ros::publish (format nil "~A/~A/joint_states_ri_move" *im-nodename* *robot-topic-name*)
                     joint-state-msg)

       )
      (t
       (ros::publish (format nil "~A/~A/reset_joint_states" *im-nodename* *robot-topic-name*)
                     joint-state-msg)
       (ros::publish (format nil "~A/~A/set_pose" *im-nodename* *robot-topic-name*)
                     set-pose-msg)
       ))
     )
   )

  (:publish-joint-trajectory
   (avs &optional (tms (list 3000)) &key
	(real? nil)
	(moveit nil))
   (let (joint-list
	 (joint-traj-msg
	  (instance jsk_interactive_marker::JointTrajectoryWithType :init
		    :header (instance std_msgs::header :init
				      :stamp (ros::time-now))))
	 (now-av (send *robot* :angle-vector))
	 points
	 (time-total 0)
	 )

     ;;set time list
     (if (atom tms) (setq tms (make-list (length avs) :initial-element tms)))
     (if (< (length tms) (length avs))
	 (setq tms (append tms (make-list (- (length avs) (length tms)) :initial-element (car (last tms))))))

     (dotimes (i (length avs))
       (let ((jtp (instance jsk_interactive_marker::JointTrajectoryPointWithType :init))
	     time
	     )
	 (if moveit
	     (send jtp :type jsk_interactive_marker::JointTrajectoryPointWithType::*COLLISION_AVOIDANCE*)
	   (send jtp :type jsk_interactive_marker::JointTrajectoryPointWithType::*JOINT_INTERPOLATION*))
	 (send jtp :wait t)
	 (send *robot* :angle-vector (elt avs i))
	 (setq joint-list (send *robot* :joint-list))
	 (send jtp :positions (mapcar #'(lambda (joint) (send joint :ros-joint-angle)) joint-list))

	 (setq time (elt tms i))
	 (setq time-total (+ time-total time))
	 (send jtp :time_from_start (ros::time (/ time-total 1000.0)))
	 (push jtp points)
	 ))
     (setq points (reverse points))

     (send joint-traj-msg :joint_names (send-all joint-list :name))
     (send joint-traj-msg :points points)

     (cond
      (real?
       (ros::publish (format nil "~A/~A/joint_trajectory_with_type_ri_move" *im-nodename* *robot-topic-name*)
		     joint-traj-msg))
      ))
   )



  (:publish-base-target
   ()
   (when *base-target-coords*
     (let ((base-msg (ros::coords->tf-pose-stamped *base-target-coords* (send *base-target-coords* :name))))
       (ros::publish (format nil "~A/~A/base_move" *im-nodename* *robot-topic-name*)
		     base-msg)
       )
     )
   )

  (:base-target-callback
   (msg)
   (setq *base-target-coords* (ros::tf-pose-stamped->coords msg))
   (print "update target")
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
     (ros::publish (format nil "~A/~A/hand_ri_move" *im-nodename* *robot-topic-name*)
		   hand-msg)
     )
   )

  (:init 
   ()
   (defvar *target-coords* (instance arrow-object :init))
   (send *target-coords* :translate #f(500 0 0)) ;; initial pose
   (defvar *origin-coords* (instance arrow-object :init)) ;;arrow for origin of object
   (send *origin-coords* :translate #f(500 0 0))
   (defvar *old-coords* (send *target-coords* :copy-worldcoords))
   (defvar *use-fullbody* t)
   (defvar *use-torso* nil)
   (defvar *translation-axis* t)
   (defvar *rotation-axis* t)
   (defvar *tmp-rotation-axis* t) ;;turn nil when center sphere was moved
   ;;baxter
   ;;(defvar *publish-ik* t)
   (defvar *publish-ik* nil)
   (defvar *origin-key* :rarm)

   (defvar *robot-name* (ros::get-param "~robot" "PR2"))
   (defvar *robot-topic-name* (ros::get-param "~robot_topic_name" "robot"))
   (setq *use-ik-server* (ros::get-param "~use_ik_server" t))
   (setq *frame-id* (ros::get-param "~frame_id" "map"))


   (setq *publish-ik* (ros::get-param "~start_ik" t))
   (defvar *ik-stop* (not *publish-ik*))

   (defvar *rhand-frame* nil)
   (defvar *lhand-frame* nil)
   
   (setq *ik-update-times* 0)
   (setq *target-update-times* 0)

   (defvar *start-loop* t)
   ;;init robot
   (cond
    ((equal (string-upcase *robot-name*) "PR2")
     (setq *robot-name* "PR2")
     (setq *PR2* (pr2))
     (setq *robot* *PR2*)
     (setq *have-leg* nil)
     )
    ((equal (string-upcase *robot-name*) "BAXTER")
     (require "package://baxtereus/baxter-interface.l")
     (setq *robot-name* "BAXTER")
     (setq *baxter* (baxter))
     (setq *robot* *baxter*)
     (setq *have-leg* nil)
     )
    ((equal (string-upcase *robot-name*) "ATLAS")
     (ros::load-ros-manifest "hrpsys_gazebo_atlas")
     (ros::roseus-add-msgs "hrpsys_gazebo_atlas")
     (setq *robot-name* "atlas")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-end-coords.l")
     (atlas-with-hand)
     (setq *robot* *atlas*)
     (setq *have-leg* t)
     )
    ((equal (string-upcase *robot-name*) "HRP2JSKNT")
     (setq *robot-name* "hrp2jsknt")
     (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
     (setq *robot* (instance hrp2jsknt-robot :init))
     (setq *have-leg* t)
     )
    ((equal (string-upcase *robot-name*) "HRP2JSKNTS")
     (setq *robot-name* "hrp2jsknts")
     (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
     (setq *robot* (instance hrp2jsknts-robot :init))
     (setq *have-leg* t)
     )
    ((equal (string-upcase *robot-name*) "HRP2W")
     (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
     (setq *robot* (instance hrp2w-robot :init))
     (setq *have-leg* nil)
     )
    ((equal (string-upcase *robot-name*) "STARO")
     (load "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
     (setq *robot* (instance staro-robot :init))
     (setq *have-leg* t)
     )
    ((equal (string-upcase *robot-name*) "SAMPLEROBOT")
     (load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
     (setq *robot* (instance samplerobot-robot :init))
     (setq *have-leg* t)
     )
    (t
     (ros::ros-error "undefined robot: ~A" *robot-name*)
     (ros::ros-error "use PR2")
     (setq *PR2* (pr2))
     (setq *robot* *PR2*)
     (setq *have-leg* nil)
     )
    )
   (defvar *robot-origin* (send (car (send *robot* :links)) :name))

   (defvar *im-nodename* "jsk_model_marker_interface")
   (defvar *gripper-im-nodename* "gripper_marker_for_interactive_marker_interface")
   (defvar *server-nodename*
     "/jsk_interactive_marker_manipulation")

   (defvar *interactive-pc-nodename* "/interactive_point_cloud")
   (defvar *bounding-box-nodename* "/bounding_box_marker")
   (defvar *nodename* "ik_controller")

   ;;initialize joint-index-list
   ;;it is used to convert joint-name -> index
   ;;(joint-index-init)
   
   (ros::roseus *nodename*)
   (setq *tfb* (instance ros::transform-broadcaster :init))
   (setq *tfl* (instance ros::transform-listener :init))
   (setq *fullbody-ik-client* (instance fullbody-ik-client :init))

   ;;robot-joint-interface
   (setq *rji* (instance robot-joint-interface :init *robot*
			 (format nil "~A/~A/joint_trajectory_with_type_ri_move" *im-nodename* *robot-topic-name*)))


   (ros::roseus-add-msgs "sensor_msgs")

   (ros::advertise (format nil "~A/status" *nodename*)
		   jsk_rviz_plugins::OverlayText)
   (ros::advertise (format nil "~A/ik_status" *nodename*)
		   jsk_rviz_plugins::OverlayText)


   (ros::advertise (format nil "~A/marker" *server-nodename*)
		   visualization_msgs::Marker)
   (ros::advertise (format nil "~A/set_pose" *server-nodename*)
		   geometry_msgs::PoseStamped)
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
    jsk_interactive_marker::MarkerMenu #'send self :robot-marker-menu-callback 1)
   (ros::subscribe
    (format nil
	    "/~A/joint_states_decompressed" *robot-topic-name*)
    sensor_msgs::JointState
    #'send self :joint-state-subscriber 1)

   ;;Update Interactive Marker
   (ros::advertise
    (format nil "~A/~A/reset_joint_states" *im-nodename* *robot-topic-name*)
    sensor_msgs::JointState)
   ;; set Interactive Marker Root Pose
   (ros::advertise (format nil "~A/~A/set_pose" *im-nodename* *robot-topic-name*)
		   geometry_msgs::PoseStamped)

   ;; reset Interactive Marker Root Pose
   (ros::advertise (format nil "~A/~A/reset_root_pose" *im-nodename* *robot-topic-name*)
		   std_msgs::Empty)
   
   ;;update real-robot joint state
   (ros::subscribe
    (format nil "~A/~A/joint_states" *im-nodename* *robot-topic-name*)
    sensor_msgs::JointState #'send self :robot-marker-joint-state-subscriber)

   (ros::advertise
    (format nil "~A/~A/joint_states_ri_move" *im-nodename* *robot-topic-name*)
    sensor_msgs::JointState)

   (ros::advertise
    (format nil "~A/~A/joint_trajectory_with_type_ri_move" *im-nodename* *robot-topic-name*)
    jsk_interactive_marker::JointTrajectoryWithType)

   (ros::advertise
    (format nil "~A/~A/hand_ri_move" *im-nodename* *robot-topic-name*)
    std_msgs::String)

   (ros::advertise (format nil "~A/~A/base_move" *im-nodename* *robot-topic-name*)
		   geometry_msgs::PoseStamped)

   (ros::advertise (format nil "~A/l_gripper/set_pose" *gripper-im-nodename*)
		   geometry_msgs::PoseStamped)

   (ros::subscribe
    (format nil "~A/~A/base_pose" *im-nodename* *robot-topic-name*)
    geometry_msgs::PoseStamped #'send self :base-target-callback)

   (setq *real-robot* (copy-object *robot*))
   (gl::transparent *real-robot* 0.3)

   (send self :init-pose)
   (send self :set-origin-to-hand *origin-key*)

   (objects
    (list *robot* *real-robot* *origin-coords* *target-coords*))
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

   (when *have-leg*
     (send *robot* :rleg :knee-p :min-angle 10)
     (send *robot* :lleg :knee-p :min-angle 10))

   (send *robot* :angle-vector
	 (copy-object (send *real-robot* :angle-vector)))
   (send self :fix-robot-coords)
   (send *irtviewer* :draw-objects)
   (send *irtviewer* :look-all)

   (ros::rate 100)
   
   (send self :set-marker-pose-robot (send *target-coords* :worldcoords))
   ;;update *real-robot* and joint marker pose
   (send self :realmodel2model)
   (send self :publish-joint-states :real? nil)
   (send self :publish-status)
   (if *start-loop* (send self :main-loop))
   )

  (:main-loop
   nil
   (let (ret move rot buf tmp str)
     (do-until-key
      ;;(bench
       (do-until-key
	(x::window-main-one)
	(ros::spin-once)
	(if (not *mouse-mode*) (return-from nil nil))
	)
       (cond
	(*ik-stop*
	 (unix:usleep (round (* 1000 100))))
	(t
	 (let ((move-arms '(t t))
	       (use-torso (or *use-torso* *use-fullbody*))
	       rotation-axis
	       target-coords
	       move-target)

	   (setq move-arms
		 (case *move-limb*
		   (:rarm '(nil t))
		   (:larm '(t nil))
		   (:arms '(t t))))

	   (if *have-leg*
	       (progn
		 (setq move-arms (append move-arms '(nil nil)))
		 (setq target-limbs '(:larm :rarm :lleg :rleg))
		 )
	     (setq target-limbs '(:larm :rarm)))
	   (setq rotation-axis (mapcar #'(lambda (move-arm) (if move-arm (if *tmp-rotation-axis* *rotation-axis* nil) t)) move-arms))

	   (cond
	    ((or (equal (string-upcase *robot-name*) "HRP2JSKNT") (equal (string-upcase *robot-name*) "HRP2JSKNTS"))

	     (mapcar
	      #'(lambda (k)
		  (send *real-robot*
			:put (read-from-string (format nil "~A-ankle-end-coords" k))
			(make-cascoords
			 :name (read-from-string (format nil "~A-ankle-end-coords" k))
			 :parent (send (send *real-robot* k :end-coords :parent) :parent)
			 :coords (send (send *real-robot* k :end-coords) :copy-worldcoords))))
	      '(:lleg :rleg))

	     (setq move-target (list (send *real-robot* :larm :end-coords)
				     (send *real-robot* :rarm :end-coords)
				     (send *real-robot* :get :lleg-ankle-end-coords)
				     (send *real-robot* :get :rleg-ankle-end-coords)))

	     ;;ik target
	     (setq target-coords
		   (mapcar #'(lambda (arm move-arm)
			       (if move-arm
				   (let* ((origin->hand (send (send *origin-coords* :transformation arm) :copy-worldcoords))
					  (hand (send origin->hand :transform *target-coords* :world)))
				     hand)
				 (send arm :copy-worldcoords)))
			   move-target move-arms))
	     )
	    (t
	     ;;ik target
	     (setq target-coords
		   (mapcar #'(lambda (arm move-arm)
			       (if move-arm
				   (let* ((origin->hand (send (send *origin-coords* :transformation (send *real-robot* arm :end-coords)) :copy-worldcoords))
					  (hand (send origin->hand :transform *target-coords* :world)))
				     hand)
				 (send *real-robot* arm :end-coords :copy-worldcoords)))
			   target-limbs move-arms))))

	   (if *use-ik-server*
	       (cond
		((equal (string-upcase *robot-name*) "PR2")
		 (setq ik-server-res
		       (send *fullbody-ik-client* :ik-server-call
			     :robot *robot*
			     :target-limbs target-limbs
			     :target-coords target-coords
			     :stop 50
			     :rotation-axis rotation-axis
			     ;;for pr2?
			     :use-torso use-torso
			     :group-name ":inverse-kinematics"
			     :centroid-thre nil
			     :target-centroid-pos nil
			     :cog-gain 0
			     )
		       )
		 )
		((or (equal (string-upcase *robot-name*) "HRP2JSKNT") (equal (string-upcase *robot-name*) "HRP2JSKNTS"))
		 (cond
		  (*use-fullbody*
		   (setq ik-server-res
			 (send *fullbody-ik-client* :ik-server-call
			       :robot *robot*
			       :target-coords target-coords
			       :stop 50
			       :rotation-axis rotation-axis
			       :move-target move-target
			       )))
		  (*use-torso*
		   (send *robot* :fix-leg-to-coords (make-coords) :both)
		   (setq ik-server-res
			 (send *fullbody-ik-client* :ik-server-call
			       :robot *robot*
			       :target-coords (subseq target-coords 0 2)
			       :stop 50
			       :rotation-axis (subseq rotation-axis 0 2)
			       :move-target (subseq move-target 0 2)
			       ))
		   )
		  (t
		   (let ((mt (append (subseq move-target 0 2) (list (send *real-robot* :torso :end-coords))))
			 (tc (append (subseq target-coords 0 2) (list (send *real-robot* :torso :end-coords :copy-worldcoords))))
			 (rt (append (subseq rotation-axis 0 2) '(t))))
		     (setq ik-server-res
			   (send *fullbody-ik-client* :ik-server-call
				 :robot *robot*
				 :target-coords tc
				 :stop 50
				 :rotation-axis rt
				 :move-target mt
				 )))))
		 )
		(t
		 (format t "[ik-server]")
		 (bench
		  (setq ik-server-res
			(send *fullbody-ik-client* :ik-server-call
			      :robot *robot*
			      :target-limbs target-limbs
			      :target-coords target-coords
			      :stop 50
			      :rotation-axis rotation-axis
			      ;;:use-torso use-torso
			      ;;:group-name ":inverse-kinematics"
			      ;;:centroid-thre nil
			      ;;:target-centroid-pos nil
			      ;;:cog-gain 0
			      )))))

	     (let ((tc target-coords)
		   (mt (send *robot* :arms :end-coords))
		   (torso-link-num 0)
		   )
	       ;; use null space for aligning orientation
	       (if use-torso
		   (cond
		    ((equal (string-upcase *robot-name*) "PR2")
		     (setq torso-link-num 1)
		     ))
		 )
	       (setq ik-suc
		     (send *robot* :inverse-kinematics tc
			   :use-torso use-torso :rotation-axis rotation-axis :thre (list 10 10)
			   :stop 100
			   :null-space #'(lambda ()
					   (concatenate float-vector (make-array torso-link-num :element-type float-vector)
							(scale 0.1
							       (transform
								(send *robot* :calc-inverse-jacobian
								      (send *robot* :calc-jacobian-from-link-list
									    (mapcar #'(lambda (tmp-mt arm) (send *robot* :link-list (send tmp-mt :parent) (send *robot* arm :root-link)))
										    mt '(:larm :rarm))
									    :move-target mt
									    :translation-axis (list nil nil) :rotation-axis (list t t)))
								(apply #'concatenate float-vector
								       (mapcar #'(lambda (tmp-tc tmp-mt)
										   (send tmp-mt :difference-rotation tmp-tc))
									       tc mt))
								)))
					   )
			   ))
	       )
	     )
	   (cond
	    ((if *use-ik-server* (= (send ik-server-res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*) (vectorp ik-suc))
	     (setq revert-if-fail t)
	     (send self :publish-ik-status (format nil " IK SUCCESS"))
	     (print (send *robot* :angle-vector))
	     (if *publish-ik*
		 (send self :publish-joint-states :real? nil)))
	    (t ;(evenp (random 2))
	     (setq revert-if-fail t)
	     (send self :publish-ik-status (format nil " IK FAIL") :status :ERROR)
	     ))
	   (setq *ik-update-times* (1+ *ik-update-times*))
	   (send self :publish-status)
	   )))
       )))
  )
