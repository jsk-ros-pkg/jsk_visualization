#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")

(require :robot-im "package://jsk_interactive/scripts/robot-im.l")

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
     (print (list "callback" msg))
     (print (list "callback" origin))
     (print (list "callback" goal))
     (setq a origin)
     (print (list "test" (send origin :worldcoords)))

     (setq base->origin (send *tfl* :lookup-transform "base_link" (send origin :name)  (ros::time 0)))
     (unless base->origin
       (return-from :model-move-object-callback)
       )

     (send origin :transform base->origin :world)
     (send goal :transform base->origin :world)
     ;;(send grasp :transform origin :world)
     (setq a origin)
     ;;(send self :pose-move origin goal)

     (setq origin-base (send (send robot :worldcoords) :copy-worldcoords))
     (print "origin")
     (print origin-base)
     ;;test best place for base
     (dotimes (i 10)
       (setq base-ip-num 10)
       (dotimes (j (+ base-ip-num 1))
	 (setq grasp-coords (send grasp :copy-worldcoords))
	 (send grasp-coords :transform (midcoords (/ (+ i 1) (float base-ip-num)) origin goal) :world)

	 (send robot :inverse-kinematics grasp-coords
	       :move-arm :rarm
	       :debug-view nil
	       :use-base 0.00001
	       :base-range (list :min #f(-100000 -10000 -300)
				 :max #f( 100000  10000  300)))
	 (objects (list robot grasp-coords))
	 )
       )

     (print "origin")
     (print (send robot :worldcoords))

     (setq goal-base (send origin-base :transformation (send robot :worldcoords)))
     (print origin-base)
     (print goal-base)
     (send ri :move-to goal-base :frame-id "/base_footprint")
     (setq goal-coords  goal-base)

     ;;grasp first
     (setq grasp-coords (send grasp :copy-worldcoords))
     (send grasp-coords :transform origin :world)
     (send self :solve-ik grasp-coords :move-arm :rarm)
     (send ri :angle-vector (send robot :angle-vector) 5000)
     (send ri :wait-interpolation)

     (setq b grasp)

     ;;grasp

     ;;:rotation-axis ik-rotation-axis :use-torso use-torso)

     (send self :pose-move-object origin goal :arm :rarm)

     )
   )
  )
