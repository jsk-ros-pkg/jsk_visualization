#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")

(require :robot-im "package://jsk_interactive/euslisp/robot-im.l")

(load "models/room73b2-hitachi-fiesta-refrigerator-object.l")

(defclass pr2-im
  :super robot-im
  )


(defmethod pr2-im
  (:init
   (&rest args)
   (send-super* :init args)
   )
  
  (:set-marker-pose-robot
   (coords &rest args)
   (if args
       (send* self :set-marker-pose (send (send self :get-base-coords) :transformation coords :local) args)
     (send self :set-marker-pose (send (send self :get-base-coords) :transformation coords :local))
     )
   )

  (:get-base-coords
   ()
   (send robot :worldcoords)
   )


  (:move-callback
   (msg)
   (case (send msg :menu)
     (35
      (print "overwrite callback")
      )
     
     (t
      (send-super :move-callback msg)
      )
     )
   )

  (:model-move-object-callback
   (msg)
   (let ((origin (ros::tf-pose-stamped->coords (send msg :origin)))
	 (goal (ros::tf-pose-stamped->coords (send msg :goal)))
	 (grasp (ros::tf-pose->coords (send msg :graspPose))))
     (setq tmp-msg msg)
     
     ;;(send grasp :rotate pi :z)

     (print (list "callback" origin))
     (print (list "callback" goal))
     (print (list "test" (send origin :worldcoords)))

     (setq base->origin (send *tfl* :lookup-transform "/base_link" (send origin :name)  (ros::time 0)))
     (unless base->origin
       (return-from :model-move-object-callback)
       )

     ;;when open door
     (send grasp :rotate -pi :z)
     (print grasp)

     (send *pr2* :locate #f(0 0 0) :world)
     (send origin :transform base->origin :world)
     (send goal :transform base->origin :world)
     (setq a origin)

     (setq origin-base (send (send robot :worldcoords) :copy-worldcoords))
     (print "origin")
     (print origin-base)


     ;; ;;test best place for base
     ;; (dotimes (i 10)
     ;;   (setq base-ip-num 10)
     ;;   (dotimes (j (+ base-ip-num 1))
     ;; 	 (setq grasp-coords (send grasp :copy-worldcoords))
     ;; 	 (send grasp-coords :transform (midcoords (/ (+ j 1) (float base-ip-num)) origin goal) :world)

     ;; 	 (send robot :inverse-kinematics grasp-coords
     ;; 	       :move-arm :rarm
     ;; 	       :debug-view nil
     ;; 	       :use-base 0.00001
     ;; 	       :base-range (list :min #f(-10000000000 -10000000000 -300)
     ;; 				 :max #f( 10000000000  10000000000  300)))
     ;; 	 (objects (list robot grasp-coords))
     ;; 	 )
     ;;   )

     (print "origin")
     (print (send robot :worldcoords))

     (setq goal-base (send origin-base :transformation (send robot :worldcoords)))
;;     (print origin-base)
;;     (print goal-base)

     ;;reset pose
     ;; (send robot :reset-pose)
     ;; (send ri :angle-vector (send robot :angle-vector) 5000)
     ;; (send ri :wait-interpolation)


     ;;(send ri :move-to goal-base :frame-id "/base_footprint")
     (setq goal-coords  goal-base)

     ;;grasp first
     (send self :stop-grasp :rarm)

     (setq grasp-coords (send grasp :copy-worldcoords))
     ;;(grasp-coords)
     (send grasp-coords :transform origin :world)
     (send robot :inverse-kinematics grasp-coords
	   :move-arm :rarm
	   :use-torso t
	   :debug-view nil
	   :revert-if-fail nil
	   )
     (send robot :rarm :move-end-pos #f(-100 0 0))
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;     (send self :solve-ik grasp-coords :move-arm :rarm)
     (send ri :angle-vector (send robot :angle-vector) 5000)
     (send ri :wait-interpolation)

     (send robot :rarm :move-end-pos #f(100 0 0))
     (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
     (send ri :angle-vector (send robot :angle-vector) 2000)
     (send ri :wait-interpolation)
     (send self :start-grasp :rarm)
     (send ri :wait-interpolation)
     (send ri :move-gripper :rarm 0 :effort 20)
     (send ri :wait-interpolation)

     (setq b grasp-coords)
     (setq e grasp)
     (setq c origin)
     (setq d goal)

;;     (setq *fridge* (room73b2-hitachi-fiesta-refrigerator))
;;     (send *fridge* :locate #f(1000 0 0))
;;     (send *fridge* :rotate pi :z)
     
;;     (objects (list goal origin grasp-coords *pr2* *fridge*))

     (send self :pose-move-object goal origin :arm :rarm)

     )
   )
  )
