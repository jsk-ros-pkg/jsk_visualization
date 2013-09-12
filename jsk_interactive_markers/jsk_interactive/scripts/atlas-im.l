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
   (send robot :hand arm :angle-vector #f(0 90 40 0 90 40 0 90 40 0 90 45))
   (hand-model2real :time 2000)
   )
   
   

  (:stop-grasp
   (&optional (arm :rarm))
   (send robot :hand arm :angle-vector #f(0 0 0 0 0 0 0 0 0 0 0 0))
   (hand-model2real :time 2000))


  (:set-marker-pose-robot
   (coords)
   (send self :set-marker-pose (send (send self :get-base-coords) :transformation coords :local))
   )

  (:get-base-coords
   ()
   (caddr (send robot :torso))
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
