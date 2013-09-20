#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")

(require :robot-im "package://jsk_interactive/scripts/robot-im.l")

(defclass atlas-im
  :super robot-im
  )


(defmethod atlas-im
  (:init
   (&rest args)
;;   (my-init :set-reset-pose nil)
;;   (set-kp 4000)
   (send-super* :init args)
   )

  (:start-grasp
   (&optional (arm :rarm))
   (cond
    ((equal arm :arms) (send self :start-grasp :rarm) (send self :start-grasp :larm))
    (t
     (send robot :hand arm :angle-vector #f(0 90 40 0 90 40 0 90 40 0 45 45))
   ;;(hand-model2real :time 7000)
     (send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 7000)
     ))
   )
   
   

  (:stop-grasp
   (&optional (arm :rarm))
   (send robot :hand arm :angle-vector #f(0 0 0 0 0 0 0 0 0 0 0 0))
   (hand-model2real :time 5000))


  #|
  (:solve-ik 
   (target-coords &key (move-arm :rarm) (rotation-axis t) (debug-view nil) (use-torso nil))
   (cond 
    ((not use-torso)
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
       ))
     )
    (t
     (let (tc)
       (cond
	((equal move-arm :rarm)
	 (setq tc (list nil nil target-coords nil))
	 )
	((equal move-arm :larm)
	 (setq tc (list nil nil nil target-coords))
	 )
	((equal move-arm :arms)
	 (setq tc (list nil nil (car target-coords) (cadr target-coords)))
	 )
	)

       (with-move-target-link-list
	(mt ll robot '(:rleg :lleg :rarm :larm))
	(send robot :fullbody-inverse-kinematics
	      (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	      :link-list ll :move-target mt
	      :collision-avoidance-link-pair nil
	      :revert-if-fail t
	      :stop 1000
	      :translation-axis '(t t t t)
	      :rotation-axis '(t t rotation-axis rotation-axis)
	      :thre '(5 5 10 10)
	      :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	      :target-centroid-pos (send (send robot :foot-midcoords) :worldpos)
	      :centroid-thre 100)))
     ))
   )

  |#

  (:set-marker-pose-robot
   (coords &rest args)
   (setq a args)
   (if args
       (send* self :set-marker-pose (send (send self :get-base-coords) :transformation coords :local) args)
     (send self :set-marker-pose (send (send self :get-base-coords) :transformation coords :local))
   ))

  (:get-base-coords
   ()
   ;;(caddr (send robot :torso))
   

   ;;(send *tfl* :lookup-transform "utorso" "odom" (ros::time 0))
   (let (coords )
     (setq coords (send (send (caddr (send robot :torso)) :worldcoords) :copy-worldcoords))
     (send coords :transform (send *tfl* :lookup-transform "utorso" "odom" (ros::time 0)))
     
;;   (caddr (send robot :torso))
     coords
     )
   )


  

  (:move-to
   (coords)
   (let* ((pos (send coords :worldpos))
	  (x (elt pos 0))
	  (y (elt pos 1))
	  (deg (rad2deg (elt (car (rpy-angle (send coords :worldrot))) 0))))
     (ros::ros-warn "go-pos ~A[mm] ~A[mm] ~A[deg]" x y deg)
     (send ri :go-pos (/ x 1000) (/ y 1000) deg)
     )
   )

  (:move-callback
   (msg)
   (case (send msg :menu)
     (35
      (print "change manip mode")
      (real2model)
      (model2real :time 100 :wait t)
      (manip)
      (set-effort 0 :torso)
      (model2real :wait t)
      );;manipulation mode
     (t
      (send-super :move-callback msg)
      )
     )

)
