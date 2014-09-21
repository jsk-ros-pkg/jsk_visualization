^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive_marker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.11 (2014-09-22)
-------------------
* jsk_interactive_marker does not depend on geometry
* Contributors: Ryohei Ueda

1.0.10 (2014-09-13)
-------------------
* add new executable to control CameraInfo with interactive marker
* Contributors: Ryohei Ueda

1.0.9 (2014-09-07)
------------------
* add ${PROJECT_NAME}_gencfg to all depends
* Contributors: Kei Okada

1.0.8 (2014-09-04)
------------------
* control marker with topic
* reset hand pose
* update catkin.cmake for urdf_control_marker
* root control marker for urdf marker
* rename config file
* use jsk_topic_tools::TimeAccumulator instead of jsk_pcl_ros::TimeAccumulator
* add include for catkin
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.7 (2014-08-06)
------------------
* add new program: pointcloud_cropper to crop pointcloud with interactive marker
* add config file for interactive point cloud
* update launch for pr2 gripper
* receive handle pose and publish it
* pick and place sample eus
* add reset root pose functions
* add reset marker callback
* rm empty line
* revert README.txt
* move .rviz.default to .rviz when making
* rename .rviz to .rviz.default
* use Eigen::Vector3f in footstep_marker because of the change of the api
  of jsk_pcl_ros
* depends on ${catkin_EXPORTED_TARGETS} to wait for message generation
* update footstep_marker to publish snapped pose to the planes
* support resuming the previous footstep on footstep_marker
* toggle 6dof marker via menu of footstep_marker
* toggle visualization of 6dof marker of footstep_marker via ~show_6dof_control parameter
* publish hand marker pose
* publish selected marker index
* snap the goal direction to the planes even with joy stick command
* do not use deprecated functions to convert tf and kdl instances to avoid
  compilation warning
* add 'Cancel Walk' menu to footstep marker
* Initialize the position of the marker to the frame if ~initial_reference_frame is specified
* register planDoneCB to the sendGoal function to the planner in footstep_marker
* asynchronously get the result of the footstep planning in footstep_marker
* add interactive_point_cloud.h
* add bounding box
* change paramater with dynamic reconfigure
* publish marker pose
* add interactive point cloud
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.6 (2014-07-14)
------------------
* add grasp method
* publish root pose when clicked
* launch file for pr2 gripper marker
* display multi marker
* add PR2 gripper xacro and setting file
* set initial joint state
* add class to set urdf marker config
* Contributors: Yusuke Furuta

1.0.5 (2014-06-29)
------------------
* add param to designate tf origin
* add new menu to call "estimate occlusion"
* skip planning until release the marker
* automatically snap the footstep marker to the plane if ~use_plane_snap
  is set to true
* publish the selected bounding box as BondingBoxArray for visualization
* publish the selected box as well as the index of the box
* add dummy camera launch file
* Contributors: Masaki Murooka, Ryohei Ueda, Yusuke Furuta

1.0.4 (2014-05-31)
------------------
* jsk_interactive_marker: fix for rosbuild, add mk/rosbuild to package.xml
* add "execute the plan" and "force to replan" mouse menu to footstep_marker
* add bounding_box_marker to select jsk_pcl_ros/BoundingBoxArray
* Contributors: Kei Okada, Ryohei Ueda

1.0.3 (2014-05-22)
------------------
* update depreceted functions
* add depend to roslib roscpp for ros::package

1.0.2 (2014-05-21)
------------------
* add interactive_markers and urdf

1.0.1 (2014-05-20)
------------------
* use geometry package to install orocos_kdl, since orocos_kdl is not installed via rosdep https://github.com/ros/rosdistro/pull/4336
* Contributors: Kei Okada

1.0.0 (2014-05-17)
------------------

0.0.3 (2014-05-15)
------------------

0.0.2 (2014-05-15)
------------------
* compile executables after message generation
* wait for service before making service client
* remove dependency on hrpsys_gazebo_atlas when using pr2
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into service-persistent-true
* use rotation-axis in inverse-kinematics
* set persistent true in dynamic_tf_publisher_client
* delete code using robot_state_publisher
* delete move_base_marker
* add jsk_pcl_ros message dependency
* change the location of catkin_package and generate_messages
* change marker frame id to /map
* Add method to set marker root link to robot root link
* Not use joint_state_publisher but use dynamic_tf_publisher when making
  robot marker
* add method to publish base pose
* add an interface to call footstep_controller from other programs to footstep_marker
* support foot offset parameters for initial feet placements
* use tf_conversions and eigen_conversions to convert tf::Transform to geometry_msgs::Pose
* use tf::Transform to store offset information
* implement readPoseParam
  a function to read geometry_msgs/Pose from a rosparameter
* add gitignore to jsk_interactive_marker
* add move base marker sample
* add controller to move base
* use tf_prefix instead of model name
* divide ik-controller.l into class and make instance
* use yaml for move base marker
* use `'tf_prefix_' instead of 'model_name_ + /'`
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix typo of jsk_interactive_marker of manifest.xml
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: reverted depend tags in manifest.xml of jsk_interactive_markers
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: add actionlib dependency to jsk_interactive_marker
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix catkin cmake syntax: CATKIN-DEPENDS -> CATKIN_DEPENDS
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: fix description of jsk_interactive_marker/manifest.xml
* use rosdep name for rviz and actionlib_msgs
* add urdf marker in order to move base link
* add method to get joint state from robot
* update footstep_marker in order to reset iniital pose
* not use ik-server
* trying to deal with new ik server
* fixing urdf_model_marker to link urdf_model_maker_main.cpp
* divide urdf_model_marker into class definition and main function
* fixing the position of the frame id
* use interactive_marker_helpers
* initialize feet position correctly
* add hand frame slot in ik-controller
* delete ros warining and make faster
* modify pass to pr2 ik server
* adding marker to visualize initial state
* adding method to estimate initial state of footstep from frame_id
* catkinize jsk_interactive_marker for hydro
* use joint state publisher when using pr2
* add mesh file path in linkMarkerMap
* fix bag in method to find ros package path from full path
* add method to move root link
* add ik controller and launch file
* update urdf model markers testfile
* adding marker_6dof, which is controllable via rostopic and rviz
* add launch file to controll robot with interactive marker
* add base_frame parameter in point_cloud_marker
* supporting z-direction
* calling SnapIt from outer program
* enable footstep planner in sample
* support to disable planner calling from footstep_marker
* add use_visible_color parameter to change color
* adding interactive marker for footstep planning
* adding footstep interactive marker
* set Use Link as Arm by default
* rotate hand in local coordinates
* add src to convert .world to .yaml
* rename Don't allow rotation / allow rotation, use 6D / 3D, 3D (positon) as default
* add subscriber to toggle rotation axis
* add subscriber to toggle start ik
* add center sphere marker to control position
* change door marker size
* show footsteps each 2
* remenmber previous door position
* fix previous step button
* supporting showing footstep list
* set foot step by rosparam
* update
* change resolution of knob color
* change control size to max size of box
* add color knob
* get scale from urdf
* clean up code and write dummy 0 joint-angle to Joint::PRISMATIC
* add wall in door_foot.cpp
* change foot position when open door
* use robot description in atlas-real
* add sphere and box marker in urdf model marker
* add sphere and box marker in urdf model marker
* be quiet
* updating rviz
* add look at menu and message
* add marker to visualize door and foot
* fix foot position of triangle
* add move it exec cancel button
* update defaultset
* fix bag of urdf_model_marker
* add Triangle Marker to visualize foot position
* add Touch It msg
* adding clear function for external program
* changing default value
* not publish joint state all time
* adding some external control
* updating for external programs
* untabify
* add change marker size menu
* stop ik by default
* fix bag and reset marker id when clear button is pressed
* add IM to get designated Point Cloud
* add menu to select using ik server
* reset when marker was reset
* fix to use joint_state_publisher and robot_state_publisher
* add joint_state_publisher.py
* add use_dynamic_tf to disable dynamic tf
* change marker size of urdf marker
* publishJointState on resetMarkerCB
* add special pose (fg manip pose)
* we can show and hide interactive marker
* add .rviz  for interactive_marker
* change frame-id from odom to map
* modify caliculation of tf from odom to marker
* add menu to cancel planned motion
* add visualizaion mode to visualize IK
* we can select Arm Ik , Torso Ik or Fullbody Ik
* add registration mode in urdf_model_marker
* added marker_array for viewing collision lines in rviz
* add .rviz for atlas_joint_marker
* Use package:// instead of file:// to designate mesh file name
* use jsk urdf model for atlas
* add launch file for moving joints for atlas
* update README.txt
* display parent link marker when fixed joint clicked
* add joint limit in joint robot marker
* add Function to set 1 Joint Angle
* reset robot marker to real robot
* add patch file for atlas.urdf to use RobotIM
* add Move Robot Joint Marker
* add cylinder marker when joint dont include mesh
* add yaml for Fridge model in 73b2
* add msg to designate marker movement
* attach Grasp Point to Model Marker
* change display of move marker when clicking
* use configuration yaml file to set models
* get full path of gazebo model
* set Move Marker based on Joint axis
* add dependancy on dynamic_tf_publisher
* making interactive marker based on urdf model
* add finger interactive marker
* add menu to change whether robot use torso
* add Marker Type in msgs
* add hand shape interactive marker
* add interactive operation sample of eus simulator
* add head marker and change msg
* add jsk_interactive_markers/ by yusuke furuta
* Contributors: Kei Okada, Ryohei Ueda, Yusuke Furuta, Masaki Murooka, Shintaro Noda, tarukosu, Youhei Kakiuchi
