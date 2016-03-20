(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")

;; set ;;;;;;;;;;;;;;;;;;;;
(defun insert-transformable-object
  (&key (shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*) (frame-id "odom")
        (name "default_name") (description "default_description") (resource "") (use-material t))
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type shape-type :action jsk_rviz_plugins::TransformableMarkerOperate::*INSERT* :frame_id frame-id
                    :name name :description description :mesh_resource resource :mesh_use_embedded_materials use-material))
    (ros::service-call "/transformable_interactive_server/request_marker_operate" req)
    ))

(defun erase-all-transformable-object
  ()
  (let* ((req (instance jsk_rviz_plugins::RequestMarkerOperateRequest :init)))
    (send req :operate
          (instance jsk_rviz_plugins::TransformableMarkerOperate :init
                    :type jsk_rviz_plugins::TransformableMarkerOperate::*BOX* :action jsk_rviz_plugins::TransformableMarkerOperate::*ERASEALL*))
    (ros::service-call "/transformable_interactive_server/request_marker_operate" req)
    ))

(defun set-transformable-object-dimensions
  (&key x y z radius small-radius name)
  (let* ((req (instance jsk_interactive_marker::SetMarkerDimensionsRequest :init))
         (req-dim (instance jsk_interactive_marker::MarkerDimensions :init)))
    (when x (send req-dim :x (* 0.001 x)))
    (when y (send req-dim :y (* 0.001 y)))
    (when z (send req-dim :z (* 0.001 z)))
    (when radius (send req-dim :radius (* 0.001 radius)))
    (when small-radius (send req-dim :small_radius (* 0.001 small-radius)))
    (when name (send req :target_name name))
    (send req :dimensions req-dim)
    (ros::service-call "/transformable_interactive_server/set_dimensions" req)
    ))

(defun set-transformable-object-pose
  (coords frame-id &key name)
  (let* ((req (instance jsk_interactive_marker::SetTransformableMarkerPoseRequest :init))
         (req-pose-st (instance geometry_msgs::PoseStamped :init)))
    (send req-pose-st :header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id frame-id))
    (send req-pose-st :pose (ros::coords->tf-pose coords))
    (when name (send req :target_name name))
    (send req :pose_stamped req-pose-st)
    (ros::service-call "/transformable_interactive_server/set_pose" req)
    ))

(defun set-transformable-object-color
  (&key (r 0) (b 0) (g 0) (a 0))
  (let* ((msg (instance std_msgs::ColorRGBA :init :r r :g g :b b :a a)))
    (ros::publish "/transformable_interactive_server/set_color" msg)))

(defun set-transformable-object-control-relative-pose
  (coords)
  (ros::publish "/transformable_interactive_server/set_control_relative_pose" (ros::coords->tf-pose coords)))

;; get ;;;;;;;;;;;;;;;;;;;;
(defun get-transformable-object-pose
  (&key name frame-id wait-duration)
  (let* ((req (instance jsk_interactive_marker::GetTransformableMarkerPoseRequest :init))
         pose-st coords)
    (when name (send req :target_name name))
    (setq pose-st (send (ros::service-call "/transformable_interactive_server/get_pose" req) :pose_stamped))
    (setq coords (ros::tf-pose->coords (send pose-st :pose)))
    ;; convert coords for signified frame-id
    (when frame-id
      (if (and wait-duration frame-id)
          (send *tf* :wait-for-transform (send pose-st :header :frame_id) frame-id
                (send pose-st :header :stamp) wait-duration))
      (let ((trans (send *tf* :lookup-transform frame-id
                         (send pose-st :header :frame_id) (send pose-st :header :stamp))))
        (unless trans
          (setq trans (send *tf* :lookup-transform frame-id
                            (send pose-st :header :frame_id) (ros::time 0))))
        (if trans (setq coords (send trans :transform coords)))
        ))
    coords))

(defun get-transformable-object-dimensions
  (&key name)
  (let* ((req (instance jsk_interactive_marker::GetMarkerDimensionsRequest :init))
         dim)
    (when name (send req :target_name name))
    (setq dim (send (ros::service-call "/transformable_interactive_server/get_dimensions" req) :dimensions))
    (list (* 1000 (send dim :x)) (* 1000 (send dim :y)) (* 1000 (send dim :z))
          (* 1000 (send dim :radius)) (* 1000 (send dim :small_radius)) (send dim :type))
    ))

(defun set-focus-marker-name
  (name)
  (let* ((req (instance jsk_interactive_marker::SetTransformableMarkerFocusRequest :init)))
    (send req :target_name name)
    (ros::service-call "/transformable_interactive_server/get_dimensions" req)
    ))

(defun get-marker-existence
  (name)
  (let* ((req (instance jsk_interactive_marker::GetTransformableMarkerExistenceRequest :init)) res)
    (send req :target_name name)
    (setq res (ros::service-call "/transformable_interactive_server/get_existence" req))
    (if res (send res :existence) nil)
    ))
