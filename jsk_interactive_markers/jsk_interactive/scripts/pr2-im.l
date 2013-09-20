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
