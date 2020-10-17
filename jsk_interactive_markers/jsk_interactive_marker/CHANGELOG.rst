^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_interactive_marker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.7 (2020-10-17)
------------------

* set property for ccache if cmake version < 3.4 (`#780 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/780>`_)
* Remove meaningless lock (`#750 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/750>`_)
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * run 2to3 -f print
  * support noetic, use c++14, convert to package format 3

* Contributors: Kei Okada, Ryohei Ueda, Yuki Furuta

2.1.6 (2020-04-13)
------------------
* Remove unnecessary ROS_INFO (`#755 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/755>`_)
* Contributors: Yuto Uchimi

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* replace boost::shared_ptr by std::shared_ptr (`#710 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/710>`_)
  * manual urdf typedefs
  * add headers for urdf shared_ptr typedefs
  * use 'isnan' from 'std' namespace
  * use urdf typedefs for shared_ptr
  * enable C++11
  * replace boost pointers by std pointers

* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Call USE_SOURCE_PERMISSIONS before PATTERN
  * Update comment about installation
  * Install 'scripts' into SHARE_DESTINATION
  * Add comment for install destination
  * Fix installation destination

* fix typo in jsk_interactive_marker (`#718 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/718>`_)
* Remove newline and leading spaces from package.xml (`#706 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/706>`_)
* Explicitly resolve dependency for jsk_recognition_utils in package.xml (`#699 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/699>`_)
* Object 3d annotation using transformable interactive marker (`#668 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/668>`_)
  * do not compile transformable_marker_operator.cpp for jsk_recognition_msgs < 1.2.0
  * need 1.2.0 of jsk_recognition_msgs for Objects.msg
  * Update for updated Object.msg
  * Make object array topic editable and searched automatically
  * Insert mesh models dynamically by rviz panel
  * Change interactive_mode using dynamic reconfigure

* Contributors: Christian Rauch, Kei Okada, Kentaro Wada, Masaki Murooka, Shingo Kitagawa, Yuto Uchimi, Naoki Mizuno

2.1.3 (2017-10-26)
------------------

2.1.2 (2017-07-07)
------------------
* [jsk_rviz_plugins][classification_result_visualizer] minor bugfix (`#669 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/669>`_)
* [jsk_interactive_marker] Install scripts in jsk_interactive_marker (`#670 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/670>`_)
* Contributors: Kei Okada, Kentaro Wada

2.1.1 (2017-02-15)
------------------
* CMakeLists.txt: fix to support qt5
* package.xml: remove pr2eus_moveit from dependency
* Contributors: Yuki Furuta, Kei Okada

2.1.0 (2017-02-13)
------------------
* migration to kinetic, which uses qt5 wehre as indig/jade uses qt4 (`#662 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/662>`_ )
* Feature to transform markers in rviz (`#661 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/661>`_ )
  * Fix for same config in CMakeLists as jsk_rviz_plugins
    To fix error on hydro
  * Fix dependency of jsk_interactive_marker
  * Use throttle for ROS_ERROR
  * Move TransformableMarkerOperatorAction to jsk_interactive_marker
  * Update rviz for sample of transformable_markers
  * Use better marker size
  * Support transformation of dimension in transformable_markers_client
* Add client node for transformable markers (only boxes) (`#658 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/658>`_ )
  * Add TODO to extend transformable_markers_client
  * Rename transformable_boxes_client -> transformable_markers_client
  * Add client node for transformable boxes
* [maker_6dof] support mesh, publish pose topoic periodically ( `#657 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/657>`_ )
  * [jsk_interactive_marker/marker_6dof] add sample launch and doc.
  * [jsk_interactive_marker/src/marker_6dof] fix minor bug in line shape color setting.
  * [jsk_interactive_marker/src/marker_6dof] add option to select interactive marker size. default interactive marker size does not change.
  * [jsk_interactive_marker/src/marker_6dof] add option to publish pose periodically. default behavior does not change.
  * [jsk_interactive_marker/src/marker_6dof] supoort mesh as marker shape.
* Fix `#655 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/655>`_ : fix frame_id of tf published by marker 6dof( `#657 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/657>`_ )
  * Stop using fixed frame_id_ for simplicity
  * Transform pose in feedback with expected frame_id
  * Check frame_id of pose before publishing as TF
  * marker_6dof: publish tf after transformed to frame_id_, update is published with fixed frame of rviz
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Hiroto Mizohana

2.0.1 (2016-12-15)
------------------
* Fix jsk_recognition_msgs>=1.0.0 dep by jsk_interactive_marker
* Contributors: Kentaro Wada

2.0.0 (2016-12-14)
------------------
* ** Major Release** : Migrate srv files from jsk_pcl_ros to jsk_recognition_msgs ( `#644 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/644>`_ )
  see
  - https://github.com/jsk-ros-pkg/jsk_recognition/pull/1827
  - https://github.com/jsk-ros-pkg/jsk_recognition/pull/1914
  from this version, this package requires jsk_recognition higher than 1.0.0
* Remove dependency: jsk_interactive_marker on jsk_pcl_ros
* Stop using deprecated jsk_topic_tools/log_utils.h
  see
  - https://github.com/jsk-ros-pkg/jsk_common/pull/1462
  - https://github.com/jsk-ros-pkg/jsk_common/issues/1461
* Contributors: Kei Okada, Kentaro Wada

1.0.34 (2016-09-29)
-------------------

1.0.33 (2016-09-13)
-------------------
* Show description of only object which is selected `#633 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/633>`_
* [jsk_interactive_marker] Mode to display interactive manipultor only when selected `#626 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/626>`_

  * Refactor: remove variable which is not used
  * Add doc symlink for jsk_interactive_marker
  * Menu to enable/disable interactive manipulator
  * Mode to display interactive manipultor only when selected

* Validate object name is not empty to insert `#621 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/621>`_
* Set marker pose periodically for re-enabling on rviz `#618 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/618>`_

* Contributors: Kentaro Wada

1.0.32 (2016-07-20)
-------------------
* Fix missing cpp format string
* Contributors: Kentaro Wada

1.0.31 (2016-05-19)
-------------------

1.0.30 (2016-03-25)
-------------------

1.0.29 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_interactive_marker/euslisp] add transformable-object-util.
* [jsk_interactive_marker] Add document (JP) about how to use moveit_msgs/DisplayRobotState
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda

1.0.28 (2016-02-03)
-------------------
* [jsk_interactive_marker] Utility script and euslisp function to use moveit_msgs/DispalayRobotState
* [jsk_interactive_marker] Add ~use_2d parameter to remove z-axis translation
  and rotation around x-y axis
  Modified:
  - jsk_interactive_markers/jsk_interactive_marker/cfg/FootstepMarker.cfg
  - jsk_interactive_markers/jsk_interactive_marker/include/jsk_interactive_marker/footstep_marker.h
  - jsk_interactive_markers/jsk_interactive_marker/src/footstep_marker.cpp
* [jsk_interactive_marker]add parent and child interactive marker
* [out_of_body_marker.launch] odom_on_ground will be deprecated
* [jsk_interactive_marker] add some arguments to out_of_body_marker.launch
* [jsk_interactive_marker] add out_of_body_marker.launch
* Fix problem of genjava error caused by not listed message_generation
  Closes `#551 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/551>`_
  Modified:
  - jsk_interactive_markers/jsk_interactive_marker/package.xml
* [jsk_interactive_marker/urdf_model_marker] Check if urdf file exists
* [jsk_interactive_marker] Fix paren position of urdf_model_marker
* [jsk_interactive_marker] Untabify urdf_model_marker
* [jsk_interactive_marker] Add polygon marker to select one polygon
  out of multiple polygons
* Contributors: Eisoku Kuroiwa, Kentaro Wada, Ryohei Ueda, Yu Ohara

1.0.27 (2015-12-08)
-------------------
* Use ccache to make it faster to generate object file
* Contributors: Kentaro Wada

1.0.26 (2015-12-03)
-------------------
* [jsk_interactive_marker] add initialization for marker control
* [jsk_interactive_marker] Add ~force_to_replan service interface to
  footstep marker
* Contributors: Ryohei Ueda, Yu Ohara

1.0.25 (2015-10-10)
-------------------
* change service -> topic
* add right click config with yaml
* [jsk_interactive_markers] Ignore rvizconfig generated at build time
* Contributors: Kentaro Wada, Yu Ohara

1.0.24 (2015-09-08)
-------------------
* add jsk_recognition_msgs to catkin_package(CATKIN_DEPEND)
* add wait for trans to avoid tf nan error
* [jsk_interactive_markers/marker_6dof] Add ~initial_orientation parameter
* publish pose with focused marker name in transformable_interactive_server
* publish name of focused marker
* [jsk_interactive_markers/marker_6dof] Remove axis marker when remove
  6dof circle
* [jsk_interactive_marker/marker_6dof] Add ~initial_x,y,z parameters to
  specify initial position of the marker
* [jsk_interactive_marker/marker_6dof] Add ~publish_tf parameter to
  publish tf frame at the position of interactive marker.
* [jsk_interactive_marker/marker_6dof] Fix indent
* [jsk_interactive_marker] Add ~lock_joint_states and ~unlock_joint_states to furutaractive marker
  not to move joint angles of the model even if reset_joint_states is called
* [jsk_interactive_marker/footstep_marker] Publish current pose even if the marker is move via topic
* [jsk_interactive_marker/urdf_control_marker] Call dynamic_tf untili success if it failed
* [jsk_interactive_marker] Print error message if dynamic_tf is failed
  in urdf_control_marker
* [jsk_interactive_marker] Add hide/show empty service to transformable interactive server
* [jsk_interactive_marker/footstep_marker] Add dynamic reconfirue API to toggle projection to pointcloud
* [jsk_interactive_marker/footstep_marker] Always publish current pose of marker
* [jsk_interactive_marker/footstep_marker] Publish snapped pose even if service and topic API
  is used
* [jsk_interactive_marker/footstep_marker] Fix initialization of marker pose
* [jsk_interactive_marker] Use ros::ServiceClient::waitForExistence
  instead of ros::service::waitForService in order to wait
  dynamic_tf_publisher. Use different API leads potential bugs
* [jsk_interactive_marker/urdf_model_marker] Fix furutaractive when it fails to lookup tf transformation.
  1) use JSK_ROS_INFO, JSK_ROS_ERROR to output debug message
  2) Fix handling of tf lookup exception not to ignore addChildLinkNames
* [jsk_interactive_marker/urdf_control_marker] Visualize furutaractive marker without clicking urdf_control_marker by
  calling dynamic_tf in constructor
* [jsk_interactive_marker/footstep_marker] Set size of footsteps correctly
* [jsk_interactive_marker/footstep_marker] Fix projection of footprint
  when service interface is used
* Contributors: JSK Lab Member, Kei Okada, MasakiMurooka, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------
* [jsk_interactive_marker] Install include directory and library
* Contributors: Ryohei Ueda

1.0.22 (2015-06-24)
-------------------
* [jsk_interactive_marker] Install binaries
* [jsk_interactive_marker/footstep_marker] Enable ~footstep_margin parameter again
* always publish pose of transformable model
* Contributors: Ryohei Ueda, Masaki Murooka

1.0.21 (2015-06-11)
-------------------
* [jsk_interactive_marker] Add menu to select rleg or lleg as the first step
* [jsk_interactive_marker] Add menu to set heuristic
* [jsk_interactive_marker] Add ~always_planning parameter to footstep_marker
* [jsk_interactive_marker] Fix for terrain task
* [jsk_interactive_marker] Add topic interface to footstep_marker
* [jsk_interactive_marker] Decide footstep margin from robot name
* [jsk_interactive_marker] enable to change mesh marker control size
* [jsk_interactive_marker] add changing focus marker name line
* Contributors: Ryohei Ueda, Yu Ohara

1.0.20 (2015-05-04)
-------------------
* update furutaructive system. add load-ros-manifest. fix hand model path
* [jsk_interactive_marker/urdf_control_marker] Transform pose respected to fixed_frame_id
  when urdf_control_marker is moved by topic interface
* [jsk_interactive_marker] Supress messages while loading urdf mesh
* [jsk_interactive_marker] Add look ground menu to footstep_marker
* [jsk_interactive_marker] Fix timestamp handling in transformable object when ~strict_tf:=false
* Contributors: Masaki Murooka, Ryohei Ueda

1.0.19 (2015-04-09)
-------------------
* [jsk_interactive_marker/transformable_object] Add ~stric_tf parameter to
  relax timestamp soundness
* [jsk_interactive_marker] Add transformation of lleg and rleg when
  projecting goal footstep
* [jsk_interactive_marker/footstep_marker] Add ~use_projection_service to
  use other service to project footprint marker
* [jsk_interactive_marker] Fix projection of marker coordinates onto grid
  in footstep_marker
* [jsk_interactive_marker] Re-project projected coords if error is not
  small enough
* [jsk_interative_marker] Use SimpleOccupancyGridArray to project
  footprint marker
* [jsk_interactive_marker] Check tf2 exception in footstep_marker
* [jsk_interactive_marker] Check polygon to be snapped is valid or not
* add topic for control control
* use jsk_recognition_msgs instead of jsk_pcl_ros
* add offset topic for interactive control
* add configure for change control orientation mode
* add_mesh_model_in_transformable_marker
* suppress print in urdf_model_marker
* set radius instead of diameter when primitive shape is cylinder
* fix to set type correctly in getting dimenstions service
* Remove rosbuild files
* Contributors: Masaki Murooka, Ryohei Ueda, Yu Ohara, Yuki Furuta

1.0.18 (2015-01-30)
-------------------
* add link to boost

1.0.17 (2015-01-29)
-------------------
* [jsk_interactive_markers] Use jsk_recognition_msgs instead of jsk_pcl_ros
* add bounding box cropper example launch
* add bounding box marker version and line version
* revert target-cds-list to prevent ik failure
* modify robot-actions and pass the msg to box movement
* use move-object function in box-movement-callback
* add move base function and tuck arm function
* use base to carry object
* rename robot_actions_test to robot_actions_sample
* Contributors: Ryohei Ueda, Yusuke Furuta, Yuto Inagaki

1.0.16 (2015-01-04)
-------------------
* [jsk_interactive_marker] Add bsd license header to marker_6dof.cpp
* [jsk_interactive_marker] Keep pose of the marker even after toggling
  rotation circle in marker_6dof
* [jsk_interactive_marker] Add menu to toggle 6dof circle
* add README.md
* add rviz for robot_actions_test.launch
* add test for robot-actions.l
* change to load baxter-moveit instead of baxter-interface
* add moveit param to robot-actions.l
* use null space for aligning orientation
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into replace-self-to-ri
* replace self to *ri* in robot-actions.l
* check irtviewer
* Merge branch 'master' into split-ik-controller
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/euslisp/ik-controller.l
* change loading order
* add one click grasp parameter
* devide ik-controller.l into two scripts
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_visualization into refactoring-ik-control
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/euslisp/ik-controller.l
* use apply-joint_state
* refactoring ik controller deleted deprecated functions
* remove hoge from ik-control.l

1.0.15 (2014-12-13)
-------------------
* use robot-joint-interface in move bounding box
* add service request index to choose box from other program
* added config for show or not show controlls
* added spaces infront and behind equal
* update interactive marker controller for hrp2w
* changed dyn_reconfogure_effect_trigger_to_switch
* add interface to send joint trajectory
* add grasp hand method
* use moveit
* add JointTrajectoryPointWithType.msg
* Contributors: Ryohei Ueda, Yusuke Furuta, Yu Ohara, Yuto Inagaki

1.0.14 (2014-12-09)
-------------------
* added initializer if box_msg is empty
* added subscriber to set point_marker pose
* add show and hide marker method
* update pr2 spacenav sample
* load baxter-interface when using baxter
* refactoring ik-controller (delete functions and meanless comment out)
* add baxter to ik contollers
* Merge branch 'master' into irteus-to-urdf-interactive-marker
* add grasp offset
* fix generate msg
* added codes to pub relative_pose
* move and recognize handle
* Merge branch 'master' into irteus-to-urdf-interactive-marker
  Conflicts:
  jsk_interactive_markers/jsk_interactive_marker/catkin.cmake
* changed marker_type because BUTTON_CLICK was not driven
* update joint state and root
* check ptr is null
* change staro config
* add open door function
* added service to check marker existence. added copy to marker operation.
* added service to get/set color and focus.
* added service function and service files for GetTransformableMarkerPose and SetTransformableMarkerPose
* publish tf of transformable marker in timer callback
* update transformable marker when dimension is set.
* added topic to add pose to transformable mareker with relative coords
* publish marker dimensions as topic. added MarkerDimensions.msg and use this message in service files.
* publish tf of transformable object.
* publish tf at low frequency
* add new msg
* add marker description and publish marker model movement
* add black list to fasten converting ros package file style
* improve speed of creating marker
* fix bag
* fix link when robot mode
* add center marker to urdf control marker
* modified teleop source for hrp2jsknts
* set fixed link
* adding hrp2w interactive markers
* rotate marker in world coorlds
* merge origin/master and fixed conflicts
* added service to set/get marker dimensions
* use msg int value for object shape.
* move msg to jsk_rviz_plugins
* add transformable marker operator panel
* add RequestMarkerOperate
* Merging and update
* integrate files and arrange srv
* disable one click grasp by default
* add reconfigure to transformable server
* use Eigen for transfomation in addPose
* added new srvice for transformable object. added ros service to insert/erase marker dynamically.
* update marker pose when marker menu changed
* display ik controller info
* Merge remote-tracking branch 'tarukosu/hand-marker-for-im-control' into hand-marker-for-im-control
* add scale for urdf marker
* use robot description for hand marker
* update launch file
* add samplerobot yaml
* add samplerobot controller
* add staro hand
* update base pose
* extract hand urdf from all urdf
* do not use *robot-offset*
* Remove warning message of FootstepMarker
* Supress messages from footstep_marker
* delete old code
* load urdf to visualize hand pose
* move functions to interactive_marker_utils.cpp
* add hand marker for hrp2
* tiny debug to avoid compilation warning
* Contributors: Hiroaki Yaguchi, Kei Okada, Yuto Inagaki, Ryohei Ueda, Masaki Murooka, Eisoku Kuroiwa, Yusuke Furuta, Yu Ohara

1.0.13 (2014-10-10)
-------------------
* add tabletop launch file
* Solve inverse kinematics with use-torso t
* do not use ik server for pr2
* add staro interactive marker
* add staro launch file
* add parameter to set menu
* add staro
* add arm ik and torso ik
* set frame id
* check having legs
* interactive marker control for hrp2
* add dependency on jsk_rviz_plugins
* Speed up grasp-pose movement
* add parameter
* rename launch file
* publish first handle pose
* add callback to grasp object
* subscribe initial handle pose
* add bounding box marker name and remove description
* refactor launch file
* do not use old ik-server-function
* move to jsk_interactive_marker and modify spacenav rotate
  add GetPose.srv
* Does not set the name of interactive marker for bounding box, because
  the name is too annoying
* update urdf model with topic
* add launch file to make bounding box interactive marker
* Contributors: Ryohei Ueda, Eisoku Kuroiwa, Yusuke Furuta, Yuto Inagaki

1.0.12 (2014-09-23)
-------------------
* Generate srv files on catkin
* Contributors: Ryohei Ueda

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
* Contributors: Ryohei Ueda, Yusuke Furuta, Masaki Murooka

1.0.4 (2014-05-31)
------------------
* jsk_interactive_marker: fix for rosbuild, add mk/rosbuild to package.xml
* add "execute the plan" and "force to replan" mouse menu to footstep_marker
* add bounding_box_marker to select jsk_pcl_ros/BoundingBoxArray
* Contributors: Ryohei Ueda, Kei Okada

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
* Contributors: Youhei Kakiuchi, Kei Okada, Shintaro Noda, Masaki Murooka, Ryohei Ueda, Yusuke Furuta
