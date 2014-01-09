#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "view_controller_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "spacenav_controller")

(defvar *debug-view* nil)
(cond
 (*debug-view*
  (defvar *debug-object* (make-cube 100 100 100))
  (objects *debug-object*)))

(defun spacenav-controller-callback
  (msg)
  (if *debug-view* (format t "~A~%" (send msg :axes)))
  (let* ((bs (send msg :buttons))
         (rotscl (cond ((= (elt bs 0) 1) 0.05)
                       ((= (elt bs 1) 1) 0.02)
                       (t 0.1)))
         (posscl (cond ((= (elt bs 0) 1) 0)
                       ((= (elt bs 1) 1) 20.0)
                       (t 100.0)))
         )
    (cond
     ((< (norm (send msg :axes)) 1e-3)
      ;;(print 'skip)
      )
     (t
      ;;(print 'send)
      (move-controll-marker
       :coords (make-coords
                :pos
                (scale posscl (subseq (send msg :axes) 0 3))
                :rpy
                (reverse
                 (map cons
                      #'*
                      (scale rotscl (float-vector 1 1 1))
                      (subseq (send msg :axes) 3 6)))
                ;;(scale 1e-1 (subseq (send msg :axes) 3 6)) ;; ???
                ))))
    ))

(ros::subscribe
 "/spacenav/joy"
 sensor_msgs::Joy
 #'spacenav-controller-callback)

(ros::advertise
 "/rviz/camera_placement"
 view_controller_msgs::CameraPlacement)

(ros::advertise
 "/goal_marker/move_marker"
 geometry_msgs::poseStamped)

(defvar *controll-marker*
  (instance geometry_msgs::poseStamped :init))
(defvar *controll-twist* #F(0 0 0 0 0 0))
;;(defvar *controll-coords* (make-coords))
(defvar *camera-place*
  (instance view_controller_msgs::CameraPlacement :init))

(defun move-controll-marker
  (&key
   (coords (make-coords))
   (pos (send coords :worldpos))
   (rot (send coords :worldrot))
   (twist (concatenate float-vector
		       (scale 1e-3 pos)
		       (matrix-log rot))))
  (let* ((position (send (send *controll-marker* :pose) :position))
	 (orientation
	  (send (send *controll-marker* :pose) :orientation))
	 c
	 )
    (setq *controll-twist* (v+ twist *controll-twist*))
    (setq c
	  (make-coords
	   :pos (subseq *controll-twist* 0 3)
	   :rpy (reverse (subseq *controll-twist* 3 6))))
    (send *controll-marker* :header :frame_id "/map")
    (send *controll-marker* :header :stamp (ros::time 0.0))
    (map cons
	 #'(lambda (key val) (send position key val))
	 '(:x :y :z) (send c :worldpos))
    (map cons
	 #'(lambda (key val)
	     (send orientation key val))
	 '(:w :x :y :z)
	 (matrix2quaternion (send c :worldrot)))
    (ros::publish "/goal_marker/move_marker" *controll-marker*)))

(defclass view-property
  :super object
  :slots (yaw pitch distance focus ))
(defmethod view-property
  (:init
   (&key
    ((:yaw y) 0.0)
    ((:pitch p) 0.0)
    ((:distance d) 1.0)
    ((:focus f) (float-vector 0 0 0)))
   (setq yaw y)
   (setq pitch p)
   (setq distance d)
   (setq focus f))
  (:view-point
   nil
   (float-vector
    (+ (* distance (cos yaw) (cos pitch)) (aref focus 0))
    (+ (* distance (sin yaw) (cos pitch)) (aref focus 1))
    (+ (* distance (sin pitch)) (aref focus 2))))
  (:camera-orientation
   nil
   (let* ((OE (send self :view-point))
	  (f (normalize-vector (v- focus OE)))
	  (u (float-vector 0 0 1))
	  (r (normalize-vector (v* u f)))
	  (uy (normalize-vector (v* f r)))
	  (m (transpose (make-matrix 3 3 (list r uy f)))))
     m))
  (:camera-placement
   nil
   (let* ((TIME 0.1)
	  (view_point (send self :view-point))
	  (placement (instance view_controller_msgs::CameraPlacement :init)))
     (send placement :interpolation_mode
	   view_controller_msgs::CameraPlacement::*LINEAR*)
     (send placement :time_from_start (ros::time TIME))
     (send placement :eye :header :stamp (ros::time 0.0))
     (send placement :eye :header :frame_id "/map")
     (send placement :eye :point :x (aref view_point 0))
     (send placement :eye :point :y (aref view_point 1))
     (send placement :eye :point :z (aref view_point 2))
     (send placement :focus :header :stamp (ros::time 0.0))
     (send placement :focus :header :frame_id "/map")
     (send placement :focus :point :x (aref focus 0))
     (send placement :focus :point :y (aref focus 1))
     (send placement :focus :point :z (aref focus 2))
     (send placement :up :header :stamp (ros::time 0.0))
     (send placement :up :header :frame_id "/map")
     (send placement :up :vector :z 1.0)
     (send placement :mouse_interaction_mode
	   view_controller_msgs::CameraPlacement::*ORBIT*)
     placement)))

(unix:system ". `rospack find jsk_interactive_test`/spacenav/spacenav.sh &")
(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (cond
  (*debug-view*
   (send *debug-object* :newcoords
	 (make-coords
	  :rpy (reverse (subseq *controll-twist* 3 6))))
   (send *viewer* :draw-objects)
   (x::window-main-one)))
 (ros::sleep))


#|

(ros::publish
 "/rviz/camera_placement"
 (send (instance view-property :init) :camera-placement))

