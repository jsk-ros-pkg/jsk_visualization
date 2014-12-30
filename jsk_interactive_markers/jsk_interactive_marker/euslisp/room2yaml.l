#!/usr/bin/env roseus
(load "models/room602-scene.l")
(load "models/room610-scene.l")
(load "models/room73a3-scene.l")
(load "models/room73b2-scene.l")
(load "models/room83b1-scene.l")

(defun room2yaml
  (room &key (frame-id (send room :name))
	(include-ground nil)
	(include-spot nil))
  (let ((urdf-file (format nil "~A/launch/models/~A.yaml" (ros::rospack-find "jsk_interactive_marker") (send room :name)))
	(converted-model nil)
	)
    (with-open-file
     (f urdf-file :direction :output)

     (dolist (model (send room :objects))
       (let ((name (substitute #\_ #\- (send model :name))) ;; replace "-" to "_"
	     obj-cnt
	     (pos (scale 0.001 (send model :worldpos)))
	     (qua (matrix2quaternion (send model :worldrot)))
	     (do-convert t))

	 (if (and (not include-ground) (equal (send model :name) (format nil "~A-~A-ground" (send room :name) (string-left-trim "room" (send room :name)))))
	     (setq do-convert nil))

	 (if (not (or include-spot (find-method model :link)))
	     (setq do-convert nil))

	 (when do-convert
	   (setq obj-cnt (count t (mapcar #'(lambda (x) (equal (send x :name) (send model :name))) converted-model)))
	   (push model converted-model)
	   (setq name (format nil "~A_~A" name obj-cnt))

	   (format t "converting ~A~%" name)
	   (format f "- name: \"~A\"~%" name)
	   (format f "  description: \"~A\"~%" (send model :name))
	   (format f "  model: \"package://eusurdf/models/~A/model.urdf\"~%" (send model :name))
	   (format f "  frame-id: \"~A\"~%" frame-id)
	   (format f "  robot: false~%")
	   (format f "  pose:~%")
	   (format f "    position: {x: ~A, y: ~A, z: ~A}~%"
		   (elt pos 0) (elt pos 1) (elt pos 2))
	   (format f "    orientation: {w: ~A, x: ~A, y: ~A, z: ~A}~%"
		   (elt qua 0) (elt qua 1) (elt qua 2) (elt qua 3))
	   (format f "~%")
	   )
	 )
       )
     )
    )
  )

(room2yaml (room602) :frame-id "eng8/6f/602")
(room2yaml (room610) :frame-id "eng8/6f/610")
(room2yaml (room73a3) :frame-id "eng2/7f/73A3")
(room2yaml (room73b2) :frame-id "eng2/7f/73B2")
(room2yaml (room83b1) :frame-id "eng2/8f/83B1")

;;(mapcar #'room2yaml (list (room602) (room610) (room73a3) (room73b2) (room83b1)))