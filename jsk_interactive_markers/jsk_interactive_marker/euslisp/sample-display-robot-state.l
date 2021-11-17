#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_interactive_marker/euslisp/display-robot-state.l")
(setq *robot* (instance pr2-robot :init))
(ros::roseus "foo")
(ros::advertise "/robot_state" moveit_msgs::DisplayRobotState)
(ros::publish "/robot_state"
              (angle-vector-to-display-robot-state
                *robot*
                (send (send *robot* :link "base_footprint") :copy-worldcoords)
                :multi-dof-link-name nil
                :global-frame "base_footprint"))
(objects (list *robot*))
(send *robot* :reset-manip-pose)
(while (ros::ok)
  (ros::publish "/robot_state"
              (angle-vector-to-display-robot-state
                *robot*
                (send (send *robot* :link "base_footprint") :copy-worldcoords)
                :multi-dof-link-name nil
                :global-frame "base_footprint"))
  (dotimes (i 360)
    (let* ((r 0.2)
           (x (+ (* r (cos (deg2rad i))) 0.4))
           (y (+ (* r (sin (deg2rad i))) 0.2))
           (z 1.0))
      (let ((c (make-coords :pos (scale 1000.0 (float-vector x y z)))))
        (send *robot* :larm :inverse-kinematics c :rotation-axis :z :min-stop 0)
        (ros::publish "/robot_state"
              (angle-vector-to-display-robot-state
                *robot*
                (send (send *robot* :link "base_footprint") :copy-worldcoords)
                :multi-dof-link-name nil
                :global-frame "base_footprint"))
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        ;;(unix:usleep 1000)
        )
      )))
