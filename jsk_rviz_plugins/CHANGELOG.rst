^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2014-05-22)
------------------
* add normals param and change skip_rate to set Percentage

1.0.2 (2014-05-21)
------------------
* Fixes a moc generation error with boost >= 1.48
* add color which will be deviced by curvature

1.0.1 (2014-05-20)
------------------
* add README and images, modify some fails
* Contributors: Yuto Inagaki

1.0.0 (2014-05-17)
------------------
* show border as default. add auto coloring option to show
  clusters efficiently.
* decrease the number of the error messages from NormalDispaly
* Contributors: Ryohei Ueda

0.0.3 (2014-05-15)
------------------
* supress erro message of NormalDisplay
* depends to hark_msgs is no longer needed
* Contributors: Kei Okada, Ryohei Ueda

0.0.2 (2014-05-15)
------------------
* overlay sample for groovy
* make NormalDisplay work on catkin.
  add normal_visual.cpp to jsk_rviz_plugins.so
* fix for using ambient_sound
* rename the name of plugin from PolygonArrayDisplay to PolygonArray
* add rviz_plugins icons
* change the color of the pie chart according to the absolute value
* smaller size for the font and add new line to the text of diagnostics display
* add a bool property to toggle auto scale for Plotter2DDisplay
* Merge remote-tracking branch 'refs/remotes/origin/master' into add-auto-color-changing-feature-to-plotters
  Conflicts:
  jsk_rviz_plugins/src/plotter_2d_display.cpp
  jsk_rviz_plugins/src/plotter_2d_display.h
* add auto color change boolean property and max color to change
  the color according to the value
* add sample for overlay rviz plugins
* support DELETE action to disable OvelrayText
* use qt to draw OverlayText
* does not call setSceneBlending twice
* add caption to 2d plotter
* add margin to plotter
* does not create QPainter without argument to supress the warning message of "painter not activate"
* initialize `orbit_theta_` and check overflow of the value
* add `update_interval_` to control the time to update the chart
* do not delete movable text in when the widget is disabled, delete it in deconstructor
* does not plot a chart if rviz is invoked with the plotter plugin disabled
* add DiagnosticsDisplay
* call hide in the destructor of overlay widgets
* add text to show caption and value.
  in order to toggle caption, added new check box.
  as caption, use the widget name.
* implement piechart on rviz using overlay technique
* add showborder property to 2d rviz plotter
* add plotter2d plugin
* use non-static and uniq string for overlay object
* implement OverlayText display plugin
* compile overlay text display
* add OverlayText.msg
* delete unneeded wrench files
* delete unneeded effort related files
* Merge pull request `#23 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/23>`_ from aginika/add-normal-diplay
  Add normal diplay
* add color channel and style property
* update to display in rviz
* update norml_display
* add normal_displays and normal_visuals
* delete point_display.cpp and point_visual.cpp
* Add the line to make the code in hydro
* ignore lib directory under jsk_rviz_plugins
* add gitignore for jsk_rviz_plugins
* do not create .so file under src directory
* depends on rviz using <depend> tag, because rviz failed to detect plugins from jsk_rviz_plugins without depend tag
* remove duplicated include line from polygon_array_display.h
  this duplication and quates in #include line happens compilation error about
  moc file of qt4
* `#7 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/7>`_: add wxwidgets dependency to jsk_rviz_plugins
* add dummy jsk-rviz-plugins.test
* use rosdep name for rviz and actionlib_msgs
* rendering backside face
* enabling alpha blending for PolygonArray
* fixing catkin cmake and dependency
* adding plugin to visualize PolygonArray
* add depends to jsk_footstep_msgs
* clear cache when toggle the check box of Footstep
* adding rviz plugin to visualize footstep
* paint point black if color is not available
* add select_point_cloud_publish_action for publish select points (no color)
* select action using combobox
* change msg type to actionlib_msgs
* add panel to cancel action
* add jsk_rviz_plugin::PublishTopic and remove Effort, wrenchStamped, PointStamped
* add rviz panel to send empty msg
* comment out SOURCE_FILES waiting for Issue `#246 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/246>`_
* use EXTRA_CMAKE_FLAGS to check to use ROSBUILD
* add dependencies to jsk_hark_msgs
* fix: validateFloats should be class method
* fix strequal ROS_DISTRO env
* use ROS_Distributions instead of ROS_DISTRO for electric
* add ambient_sound for groovy
* write libjsk_rviz_plugins under {PROJECT_SOURCE_DIR}/lib for and add export rviz to packages.xml, for groovy/catkin compile
* add debug message
* remove LIBRARY_OUTPUT_PATH and use catkin_package
* fix version
* fix to install plugin_descriptoin.xml and libjsk_rviz_plugins.so
* add comments
* fix for electric
* change msg:hark_msgs/HarkPower -> jsk_hark_msgs/HarkPower
* support groovy/cmake compile
* fix typo jsk_rviz_plugin -> jsk_rviz_plugins
* add test
* add package.xml
* add grad property
* added display ambient sound power
* add robot_description property
* add effort/max_effort property
* fix set sample color value for any scale value
* support enable button for each joint `#3597460 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3597460>`_
* remove color property
* fix when max_effort is zero, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* support scale for effort_plugin, `#3595106 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/3595106>`_
* update jsk_rviz_plugins
* add jsk_rviz_plugins
* Contributors: Kei Okada, Ryohei Ueda, Yuto Inagaki, Shohei Fujii, Yusuke Furuta, Satoshi Iwaishi, Youhei Kakiuchi
