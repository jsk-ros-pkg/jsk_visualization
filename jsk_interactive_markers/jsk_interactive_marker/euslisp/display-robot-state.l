;; Utility API for moveit_msgs/DisplayRobotState
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "moveit_msgs")

(defun angle-vector-to-display-robot-state (robot rootlink-coords &key (global-frame "odom"))
  (let ((msg (instance moveit_msgs::DisplayRobotState :init)))
    (send msg :state :joint_state :name (send-all (send robot :joint-list) :name))
    (send msg :state :joint_state :position (mapcar #'deg2rad (coerce (send robot :angle-vector) cons)))
    (send msg :state :multi_dof_joint_state :header :frame_id global-frame)
    (send msg :state :multi_dof_joint_state :transforms (list (instance geometry_msgs::transform :init)))
    (send msg :state :multi_dof_joint_state :joint_names (list "world_joint"))
    (send (car (send msg :state :multi_dof_joint_state :transforms)) :translation
          (ros::pos->tf-translation (send rootlink-coords :worldpos)))
    (send (car (send msg :state :multi_dof_joint_state :transforms)) :rotation
          (ros::rot->tf-quaternion (send rootlink-coords :worldrot)))
    msg))

;; sample code
#|
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(setq *robot* (instance jaxon_red-robot :init))
(send *robot* :fix-leg-to-coords (make-coords))
(ros::roseus "foo")
(ros::advertise "/robot_state" moveit_msgs::DisplayRobotState)
(ros::publish "/robot_state" (angle-vector-to-display-robot-state *robot* (send (send *robot* :link "BODY") :copy-worldcoords)))
(progn
(send *robot* :reset-manip-pose)
(ros::publish "/robot_state" (angle-vector-to-display-robot-state *robot* (send (send *robot* :link "BODY") :copy-worldcoords)))
(let ((tc (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (float-vector 2000 0 0) :world)))
  (dotimes (i 100)
    (send *robot* :larm :inverse-kinematics tc :stop 2
          :revert-if-fail nil)
    (ros::publish "/robot_state" (angle-vector-to-display-robot-state *robot* (send (send *robot* :link "BODY") :copy-worldcoords)))
    (send *irtviewer* :draw-objects)
    (unix:usleep 10000)
    )))
|#
