#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_interactive")

(require :robot-im "package://jsk_interactive/euslisp/robot-im.l")

(defclass atlas-im
  :super robot-im
  )


(defmethod atlas-im
  (:init
   (&rest args)
   ;;   (my-init :set-reset-pose nil)
   ;;   (set-kp 4000)
   (send-super* :init args)
   (send robot :hand :rarm :angle-vector (send ri :hand :rarm :state :potentio-vector))
   (send robot :hand :larm :angle-vector (send ri :hand :larm :state :potentio-vector))

   )

  (:start-grasp
   (&optional (arm :rarm))
   (cond
    ((equal arm :arms) (send self :start-grasp :rarm) (send self :start-grasp :larm))
    (t
     ;;test for torque control
     (send ri :hand arm :send-command :effort 1.0)
     ;;(send robot :hand arm :angle-vector #f(0 90 40 0 90 40 0 90 40 0 45 45))
     ;;(send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 7000)
     ))
   )

  (:stop-grasp
   (&optional (arm :rarm))
   (cond
    ((equal arm :arms) (send self :start-grasp :rarm) (send self :start-grasp :larm))
    (t
     (send robot :hand arm :angle-vector #f(0 0 0 0 0 0 0 0 0 0 0 0))
     (send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 5000)
     ))

   )

  (:harf-grasp
   (&optional (arm :rarm))
   (cond
    ((equal arm :arms) (send self :start-grasp :rarm) (send self :start-grasp :larm))
    (t
     (send robot :hand arm :angle-vector #f(0 90 0 0 90 0 0 90 0 0 45 5))
     ;;(hand-model2real :time 7000)
     (send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 7000)
     ))
   )

  (:update-finger
   (arm)
   (send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 7000)
   (send self :set-hand-marker-pose)
   )
  
  (:reset-finger
   (arm)
   (send robot :hand arm :angle-vector (send ri :hand arm :state :potentio-vector))
   
   (send self :set-hand-marker-pose)
   (print (send ri :hand arm :state :potentio-vector))
   (print "reset-finger")
   )

  (:close-finger
   (arm rad)
   (let ((hand-av (send robot :hand arm :angle-vector))
	 (deg (rad2deg rad)))
     (setq hand-av (v+ hand-av (float-vector 0 deg deg 0 deg deg 0 deg deg 0 (/ deg 2) (/ deg 2))))
     (send robot :hand arm :angle-vector hand-av)
     (send self :set-hand-marker-pose)
     ;;(send ri :hand arm :angle-vector (send robot :hand arm :angle-vector) 2000)
     )
   )


  (:set-finger-marker-pose
   (arm link_name marker-name &key (move t))
   (cond
    (move
     (setq handle-r (make-cascoords :coords (send origin-coords :transformation (send robot :hand arm link_name))))
     (send handle-r :transform (send target-coords :copy-worldcoords) :world)
     )
    (t
     (setq handle-r (make-cascoords :coords (send robot :hand arm link_name)))
     )
    )
   (send handle-r :translate #f(-28 0 0) :world)
   (send self :set-marker-pose-robot handle-r :marker-name marker-name)
   )

  (:set-hand-marker-pose
   ()
   (case moving-arm
     (:rarm
      (send self :set-hand-marker-pose-rl :rarm)
      (send self :set-hand-marker-pose-rl :larm :move nil)
      )
     (:larm
      (send self :set-hand-marker-pose-rl :larm)
      (send self :set-hand-marker-pose-rl :rarm :move nil)
      )
     (t
      (send self :set-hand-marker-pose-rl :rarm)
      (send self :set-hand-marker-pose-rl :larm)
      )
     )

   )

  (:set-hand-marker-pose-rl
   (arm &key (move t))
   (let (link-name marker-name r_l)
     (cond
      ((equal arm :rarm)
       (setq r_l "right")
       )
      ((equal arm :larm)
       (setq r_l "left")
       )
      )
     (dotimes (finger 4)
       (dotimes (link 3)
	 (setq link-name (format nil "~A_f~A_~A_lk" r_l finger link))
	 (setq marker-name (format nil "~A_f~A_~AMarker" r_l finger link))
	 (send self :set-finger-marker-pose arm (intern (string-upcase link-name) (find-package "KEYWORD")) marker-name :move move)
	 ))
     )
   )

  (:solve-ik
   (target-coords &key (move-arm :rarm) (rotation-axis t) (debug-view nil) (use-torso nil) (use-fullbody nil))
   (cond
    (use-fullbody
     (send self :solve-fullbody-ik target-coords 
	   :move-arm move-arm :rotation-axis rotation-axis
	   :debug-view debug-view
	   )
     )
    (use-torso
     (send self :solve-torso-ik target-coords 
	   :move-arm move-arm :rotation-axis rotation-axis
	   :debug-view debug-view
	   )
     )
    (t
     (send self :solve-normal-ik target-coords 
	   :move-arm move-arm :rotation-axis rotation-axis
	   :debug-view debug-view
	   )
     )
    )
   )

  (:solve-fullbody-ik
   (target-coords 
    &key 
    (move-arm :rarm) (rotation-axis t) (debug-view nil)
    (rotation-axis-list '(t t t t))
    (target-limb '(:rarm :larm :rleg :lleg))
    (target-centroid-pos
     (scale
      0.5
      (apply #'v+
	     (mapcar
	      #'(lambda (k)
		  (send *robot* k :end-coords
			:worldpos))
	      '(:rleg :lleg)))))
    (additional-weight-list

     (mapcar
      #'(lambda (k gain)
	  (list (send robot :torso k :child-link)
		gain))
      (list :waist-r :waist-p :waist-y)
                                        ;(list 1e-3 1e-3 1e-3))
      (list 1e-3 1e-3 1e-2))
     )
    (collision-pair
     (apply
      #'append
      (mapcar
       #'(lambda (l)
	   (list (list l (send robot :utorso_lk))
		 (list l (send robot :r_uleg_lk))
		 (list l (send robot :l_uleg_lk))))
       (remove-if
	#'(lambda (l)
	    (or (eq (send l :parent)
		    (send robot :utorso_lk))
		(eq (send (send l :parent) :parent)
		    (send robot :utorso_lk))))
	(apply
	 #'append
	 (mapcar
	  #'(lambda (k) (send robot k :links))
	  (remove-if
	   #'(lambda (k) (find k '(:rleg :lleg)))
	   target-limb)))))))
    
    )

   (cond
    ;;arms
    ((equal move-arm :arms)
     (setq rotation-axis-list (list rotation-axis rotation-axis t t))
     )
    ((equal move-arm :rarm)
     (setq rotation-axis-list (list rotation-axis t t t))
     (setq target-coords (list target-coords (copy-object (send robot :larm :end-coords :worldcoords))))
     )
    ((equal move-arm :larm)
     (setq rotation-axis-list (list t rotation-axis t t))
     (setq target-coords (list (copy-object (send robot :rarm :end-coords :worldcoords)) target-coords))
     )
    )

   (send robot :fullbody-inverse-kinematics
	 (list (car target-coords)
	       (cadr target-coords)
	       (copy-object (send robot :rleg :end-coords :worldcoords))
	       (copy-object (send robot :lleg :end-coords :worldcoords)))
	 :move-target (mapcar
		       #'(lambda (k) (send robot k :end-coords))
		       (list :rarm :larm :rleg :lleg))
	 :target-centroid-pos target-centroid-pos
	 :additional-weight-list additional-weight-list
	 :root-link-virtual-joint-weight
	 #F(0.1 1.5 1 0.1 0.1 0.5)
	 ;;:min (float-vector 0 0 0 0 0 0)
	 ;;:max (float-vector 0 0 0 0 0 0)
	 :collision-avoidance-link-pair collision-pair
	 :avoid-collision-distance 10
	 :avoid-collision-null-gain 0.3
	 :avoid-collision-joint-gain 0.3

	 :link-list (mapcar
		     #'(lambda (k)
			 (send robot :link-list
			       (send (send robot k :end-coords) :parent)))
		     (list :rarm :larm :rleg :lleg))
	 :debug-view debug-view
	 :warnp t
	 :centroid-thre 100
	 :thre (make-list 4 :initial-element 5)
	 :rthre (make-list 4 :initial-element (deg2rad 3))
	 ;;	     :centroid-thre #'send self :check-centroid-with-convex
	 :rotation-axis rotation-axis-list)
   )

  (:check-centroid-with-convex
   (diff
    &optional
    (convex (convex-pos-list))
    (center
     (scale
      0.5
      (apply
       #'v+
       (mapcar
	#'(lambda (k)
	    (send robot k :end-coords :worldpos))
	'(:rleg :lleg))))))
   (let* ((diff (concatenate float-vector diff (list 0)))
	  (target (v+ diff center))
	  (t-vec (mapcar
		  #'(lambda (v) (v- target v))
		  convex))
	  (rotate
	   (mapcar
	    #'(lambda (v1 v2) (v* v1 v2))
	    t-vec (append (cdr t-vec) (list (car t-vec)))))
	  )
     (eq
      1
      (length
       (union
	nil
	(mapcar
	 #'(lambda (v) (minusp (aref v 2)))
	 rotate))))))

  

  (:solve-torso-ik
   (target-coords &key (move-arm :rarm) (rotation-axis t) (debug-view t))

   (cond
    ;;arms
    ((equal move-arm :arms)
     (setq rotation-axis-list (list rotation-axis rotation-axis t t))
     )
    ((equal move-arm :rarm)
     (setq rotation-axis-list (list rotation-axis t t t))
     (setq target-coords (list target-coords (copy-object (send robot :larm :end-coords :worldcoords))))
     )
    ((equal move-arm :larm)
     (setq rotation-axis-list (list t rotation-axis t t))
     (setq target-coords (list (copy-object (send robot :rarm :end-coords :worldcoords)) target-coords))
     )
    )

   (send robot :fullbody-inverse-kinematics
	 (list (car target-coords)
	       (cadr target-coords)
	       (copy-object (send robot :rleg :end-coords :worldcoords))
	       (copy-object (send robot :lleg :end-coords :worldcoords)))
	 :move-target (mapcar
		       #'(lambda (k) (send robot k :end-coords))
		       (list :rarm :larm :rleg :lleg))
	 :additional-weight-list
	 (mapcar
	  #'(lambda (k gain)
	      (list (send robot :torso k :child-link)
		    gain))
	  (list :waist-r :waist-p :waist-y)
                                        ;(list 1e-3 1e-3 1e-3))
	  (list 1e-3 1e-3 1e-3))
	 :root-link-virtual-joint-weight
	 (float-vector 0 0 0 0 0 0)
	 :min (float-vector 0 0 0 0 0 0)
	 :max (float-vector 0 0 0 0 0 0)
	 :link-list (mapcar
		     #'(lambda (k)
			 (send robot :link-list
			       (send (send robot k :end-coords) :parent)))
		     (list :rarm :larm :rleg :lleg))
	 :debug-view debug-view
	 :warnp t
	 :centroid-thre 100
	 :thre '(1 1 1 1)
	 :rotation-axis rotation-axis-list
	 )
   )


  (:solve-normal-ik
   (target-coords &key (move-arm :rarm) (rotation-axis t) (debug-view nil))
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
     )))


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
#|
  (:get-base-coords
   ()
   ;;(caddr (send robot :torso))
   ;;(send *tfl* :lookup-transform "utorso" "odom" (ros::time 0))
   (let (coords torso->odom)
     (setq coords (send (send (caddr (send robot :torso)) :worldcoords) :copy-worldcoords))
     (dotimes (i 5)
       (setq torso->odom (send *tfl* :lookup-transform "utorso" "odom" (ros::time 0)))
       (when torso->odom
	 (send coords :transform torso->odom)
	 (return))
       )
     ;;   (caddr (send robot :torso))
     coords
     )
   )
|#
  ;;fixed flamae is foot?
  (:get-base-coords
   ()
   ;;(caddr (send robot :torso))
   ;;(send *tfl* :lookup-transform "utorso" "odom" (ros::time 0))
   (let (coords torso->map)
     (setq coords (send (send robot :r_foot_lk :worldcoords) :copy-worldcoords))
     (while t
       (setq torso->map (send *tfl* :lookup-transform "r_foot" "map" (ros::time 0)))
       (cond
	(torso->map
	 (send coords :transform torso->map)
	 (return)
	 )
	(t
	 (ros::ros-info "r_foot -> map cannnot transform yet")
	 (ros::sleep)
	 )
       )
       )
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
     (start-walk)
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
  )