#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "view_controller_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "spacenav_controller")

(defun spacenav-controller-callback
  (msg)
  ;;(format t "~A" (send msg :axes))
  (let* ((bs (send msg :buttons))
         (rotscl (cond ((= (elt bs 0) 1) 0.25)
                       ((= (elt bs 1) 1) 0.04)
                       (t 0.1)))
         (posscl (cond ((= (elt bs 0) 1) 250.0)
                       ((= (elt bs 1) 1) 40.0)
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
                      (float-vector 0 0 rotscl)
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
(defvar *controll-coords* (make-coords))
(defvar *camera-place*
  (instance view_controller_msgs::CameraPlacement :init))

(defun move-controll-marker
  (&key
   (coords (make-coords))
   (pos (send coords :worldpos))
   (rot (send coords :worldrot)))
  (let* ((p pos)
	 (q (matrix-log rot))
	 (new-p (v+ p (send *controll-coords* :worldpos)))
	 (new-q (v+ q (matrix-log (send *controll-coords* :worldrot))))
	 (position (send (send *controll-marker* :pose) :position))
	 (orientation
	  (send (send *controll-marker* :pose) :orientation))
	 )
    (setq *controll-coords*
	  (make-coords :pos new-p
		       :rpy
		       (coerce (reverse new-q) cons)))
    (send *controll-marker* :header :frame_id "/map")
    (send *controll-marker* :header :stamp (ros::time 0.0))
    (map cons
	 #'(lambda (key val) (send position key val))
	 '(:x :y :z) (scale 1e-3 (send *controll-coords* :worldpos)))
    (map cons
	 #'(lambda (key val)
	     (send orientation key val))
	 '(:w :x :y :z)
	 (matrix2quaternion (send *controll-coords* :worldrot)))
    (ros::publish "/goal_marker/move_marker" *controll-marker*)))

(defclass view-property
  :super object
  :slots (yaw pitch distance focus ))
(defmethod view-property
  (:init
   nil
   (setq yaw 0.0)
   (setq pitch 0.0)
   (setq distance 1.0)
   (setq focus (float-vector 0 0 0)))
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
     (send placement :time_from_start TIME)
     (send placement :.eye :header :stamp (ros::time 0.0))
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
 (ros::sleep))
