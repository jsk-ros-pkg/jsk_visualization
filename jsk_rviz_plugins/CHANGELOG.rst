^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.10 (2024-12-11)
-------------------

2.1.9 (2024-12-11)
------------------
* enable to jsk_rviz_plugins, without rviz/ogre (`#848 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/848>`_)
  * jsk_rviz_plugins: find_package(Qt.. only when USE_VISUALIZATION
  * enable to compile without jsk_recognition_utils, if 'pcl/visualization' not found
  * enable to jsk_rviz_plugins, without rviz/ogre, i.e. use jsk_rviz_plugins message within embedded environment, compile with 'catkin bt -vi --cmake-args -DUSE_VISUALIZATION=ON'

* add more visualization options for BoundingBox and BoundingBoxArray `#844 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/844>`_
  * keep index to set box color
  * add value threshold filtering and fix unintended state behaviour for boxes violating size constraints
  * property naming consistency
  * hide min max properties in flat mode
  * add alpha min max value properties
  * add alpha = value option

* [ROS-O] patches (`#891 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/891>`_)

  * support Ogre 1.12
    OgreTechnique and Overlay are only forward declared in some headers now.
    `SimpleRenderable::setMaterial` requires a MaterialPtr from 1.12.
    The std::string overload was deprecated in 1.10.
  * drop ROS_DISTRO STRGREATER tests
    As the Debian packages use "Debian"
    https://salsa.debian.org/science-team/ros-ros-environment/-/blame/master/debian/rules?ref_type=heads#L6
    which is incompatible, does not trigger them
    c++14 is the default from melodic onward, so the additional statements
    there are not necessary

* [jsk_rviz_plugins] fix typo in test: boundigbox -> boundingbox (`#888 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/888>`_)
* [jsk_rviz_plugins][Plotter2D] add option to control text size in plot (`#878 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/878>`_)
* use qt key namespace (`#870 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/870>`_)
* Fix QPainterPath path has incomplete type and cannot be defined (`#869 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/869>`_)
* Fix simultaneous display of OverlayCamera and OverlayImage (`#863 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/863>`_)
* [jsk_rviz_plugins] allocate only valid bboxes (`#861 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/861>`_)
* Fixed linear gague display (`#860 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/860>`_)

* add test to check 'genpy.message.SerializationError: field width must be an integer type' (`#864 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/864>`_)
  * use int_t for width/height/top/left to be compatible with OverlayText.msg
  * add test to check 'genpy.message.SerializationError: field width must be an integer type'
  ```
  File "/home/k-okada/catkin_ws/jsk_recognition_ws/devel/lib/python3/dist-packages/jsk_rviz_plugins/msg/_OverlayText.py", line 127, in serialize
  except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
  File "/opt/ros/noetic/lib/python3/dist-packages/genpy/message.py", line 393, in _check_types
  check_type(n, t, getattr(self, n))
  File "/opt/ros/noetic/lib/python3/dist-packages/genpy/message.py", line 261, in check_type
  raise SerializationError('field %s must be an integer type' % field_name)
  genpy.message.SerializationError: field width must be an integer type
  ```

* Fixed typo of Software License Agreement. and/o2r to and/or (`#853 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/853>`_)
* Fixed typo of Software License Agreement. and/o2r to and/or
* [jsk_rviz_plugins] Fixed Camera Info visualization (`#851 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/851>`_)

  * [jsk_rviz_plugins/CameraInfo] Fixed scale depending on encodings
  * [jsk_rviz_plugins/CameraInfo] Fixed duplicated BGR encogind
  * [jsk_rviz_plugins/CameraInfo] Enable ROI size iamge input
  * [jsk_rviz_plugins/CameraInfo] Supprt for other image encodings (`#1 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/1>`_)
  * [jsk_rviz_plugins/CameraInfo] Avoid invalid image size case for visualization image
  * [jsk_rviz_plugins] Fixed Camera Info visualization

* integrate all config file to one, to make it easy to restart failed job (`#850 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/850>`_)

  * location? file? of 2017-06-20-12-00-00_people_images_in_lab.bag.tgz has changed ? but md5sum is same... strange

* Contributors: Aoi Nakane, Kei Okada, Martin Pecka, Shingo Kitagawa, Yoshiki Obinata, iory, joachim-p, ozzdemir, v4hn

2.1.8 (2022-01-11)
------------------
* Fix font name for pictgram (`#835 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/835>`_)

  * add launch/pictogram_sample.launch config/pictogram_sample.rviz
  * Add FontAwesome5: `#759 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/759>`_ need to update Font Name too
    PR `#579 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/579>`_ reported fa-mendeley is not displayed correctly. This because of the plugin uses system font, which is installed by fonts-font-awesome deb package, not font defined in fontawesome.dat. So if you remove fonts-font-awesome, you can not display most of font correctly. This PR update getFont function to return the correct font family

* Change pie chart color at your custom threshold (`#840 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/840>`_)
* [jsk_rviz_plugins] added new Linear Gauge rviz plugin (`#831 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/831>`_)
* add dependency.yml to check resolve circular dependency in jsk_rviz_plugins/package.xml (`#830 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/830>`_)

* add test code for jsk_rviz_plugins (`#825 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/825>`_)

  * fix `#799 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/799>`_
  * move normal.test and face_detecotr.test to xml because of missing bag files
  * skip if it failed to download bag file, due to 'could not download file. maximum download trial exceeded.'
  * add more info on rviz_config_check.py
    - fix bugs when waiting rviz service takes more than 5 sec
    - rviz_config_check.py: check subscribing topic of rviz while rosnode ping rviz
  * increase time-limit to 30
  * skip install test on indigo
  * jsk_rviz_plugins : enable add_rostest for all DISTRO
  * normal.test requires jsk_pcl/NormalConcatenater whcih needs jsk_pcl_ros on kinetic
  * on <meloidc, rviz/SendFilePath is not found, use std_srvs/Empty for rviz/reload_shaders
  * install gdown / jsk_pcl_ros_utils for install_sample_data
  * install_sample_data requries jsk_data
  * enable normal.test, face_detector.test
  * fix normal_sample.launch to suport launch_openni arg and use async
  * fix face_detector_sample.launch to work with 2017-06-20-12-00-00_people_images_in_lab.bag
  * fix typo t_start -> self.t_start
  * add install_sample_data.py to download 2017-06-20-12-00-00_people_images_in_lab.bag for test
  * add joint_state_publisher and robot_state_publisher to test_depend
  * add robot_command_interface_sample.rviz and service_call_panel.rviz
  * [test/segmen_array.test, config/segment_array_sample.rviz] fix wrong Topic /polygon_array_sample/output -> /segment_array_sample/output
  * jsk_rviz_plugins: add test codes
  * link_marker_publisher_sample.launch, contact_state_marker_sample.launch requires pr2_description
  * add rostest to package.xml, run test code from catkin
  * rviz_config_check.py : node to check if rviz arives

* [jsk_rviz_plugins] move rviz config from cfg/ to config/ and fix typo (`#823 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/823>`_)
* [jsk_rviz_plugins] add piechart sample (`#824 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/824>`_)
* Resize and repositioning of PieChart wasn't working, corrected. (`#822 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/822>`_)
* fix with '2to3 -w -f zip .' (`#817 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/817>`_)
* Use multiple billboard lines when exceeding max elements per line (`#808 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/808>`_)
* Updating the URLs of the jsk_rviz_plugins and jsk_rqt_plugins so the generated README points to working links (closes #805 ). (`#806 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/806>`_)
* added a regex for "color: XXX;" pattern to have a properly colored shadow (`#802 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/802>`_)

  * fix
    [jsk_rviz_plugins:make] /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:274:81: error: no matching function for call to ‘regex_replace(std::string&, std::regex&, const char [1])’
    [jsk_rviz_plugins:make]          std::string formatted_text\_ = std::regex_replace(text\_, color_tag_re, );
    [jsk_rviz_plugins:make]                                                                                  ^
    [jsk_rviz_plugins:make] /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:274:81: note: candidates are:
    [jsk_rviz_plugins:make] In file included from /usr/include/c++/4.8/regex:62:0,
    [jsk_rviz_plugins:make]                  from /home/user/ws_jsk_visualization/src/jsk_visualization/jsk_rviz_plugins/src/overlay_text_display.cpp:48:
    [jsk_rviz_plugins:make] /usr/include/c++/4.8/bits/regex.h:2162:5: note: template<class _Out_iter, class _Bi_iter, class _Rx_traits, class _Ch_type> _Out_iter std::regex_replace(_Out_iter, _Bi_iter, _Bi_iter, const std::basic_regex<_Ch_type, _Rx_traits>&, const std::basic_string<
    _Ch_type>&, std::regex_constants::match_flag_type)
    error, for compile on ubuntu 12.04

* [jsk_rviz_plugins] add warn for deprecated jsk_rviz_plugins/PoseArrayDisplay (`#786 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/786>`_)

* Contributors: Adi Vardi, Andres Kushnir, Francois Teyssere, Francois Teyssere, Jan Krieglstein, Kei Okada, Sam Pfeiffer, Shingo Kitagawa, Shumpei Wakabayashi

2.1.7 (2020-10-17)
------------------
* Fix programming issues where functions were not getting return values, and variables were not being declared for types (`#783 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/783>`_)
* Support custom color for OverlayMenu (`#775 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/775>`_)
* add human skeleton rviz visualization(`#740 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/740>`_)

  * jsk_recognition_msgs < 1.2.15 does not support human_skeleton_array_display.cpp
  * meloidc needs to include OGRE/OgreSceneManager.h
  * human_skeleton_array_display supports indigo build
  * add sphere at all edge ends
  * fix typo: skelton -> skeleton
  * add human skelton rviz visualization

* Add fg_color/bg_color to OverlayMenu.msg (`#776 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/776>`_)
* [jsk_rviz_plugins] Add StringDisplay as a new display plugin (`#728 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/728>`_)
* set property for ccache if cmake version < 3.4 (`#780 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/780>`_)
* [jsk_rviz_plugin/PieChart] add clock wise rotate option for pie chart (`#782 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/782>`_)
* Remove meaningless lock (`#750 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/750>`_)
* call processNormal when polygon points has more than 3 point (`#771 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/771>`_)
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * fix typo CV_VERSION_MAJOR -> CV_MAJOR_VERSION
  * run 2to3 -f except
  * run 2to3 -f print
  * support noetic, use c++14, convert to package format 3
  * Merge remote-tracking branch 'ruvu/fix/noetic' into add_noetic

* Add Rviz scene publisher (`#773 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/773>`_)
* Improve Overlay Visibility (`#769 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/769>`_)
* Contributors: Kei Okada, Patrick Beeson, Ramon Wijnands, Ryohei Ueda, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi, Iory Yanokura, Taichi Hagashide

2.1.6 (2020-04-13)
------------------
* [jsk_rviz_plugins] Add FontAwesome 5 (`#759 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/759>`_)

  * Display FontAwesome 5 icons from pictogram.py and pictogram_all.py
  * Add FontAwesome 5
  * Add property to set position of overlay menu

* Add option to specify width and height in VideoCapture plugin (`#748 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/748>`_)
* Add a script that convert String to OverlayText (`#753 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/753>`_)
* fix build failure of OgreSceneManager this block latest Melodic builds (`#766 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/766>`_)
* Add property to set position of overlay menu (`#758 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/758>`_)
* [jsk_rviz_plugins/OverlayImage] Add property to ignore alpha channel of the image (`#752 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/752>`_)
* [motor_states_temperature_decomposer.py] add queue_size (`#756 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/756>`_)
* support jsk_rviz_plugin to be loaded in indigo (`#739 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/739>`_)
* [jsk_rviz_plugins/OverlayImageDisplay] Use memcpy to copy image data (`#737 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/737>`_)

  * Use memcpy to copy image data from cv::Mat to QImagee instead of use QImage::setPixel() many times for optimization.

* Transport hint for camera info (`#736 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/736>`_)

  * Add field to select transport hint of CameraInfo display
    * Use ImageTransport to create subscriber to subscribe image topic in CameraInfoDisplay.
    * Use ImageTransportHintsProperty to choose image transport hints when subscribing image topic to visualize sensor_msgs/CameraInfo.
  * Use ImageTransportHintsProperty in OverlayImageDisplay class
  * Add ImageTransportHintsProperty class
    * ImageTransportHintsProperty is an rviz property class specialized for image_transport::TransportHints.

* Add transport hint field to OverlayImage display (`#730 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/730>`_)

  * Add transport hint field to OverlayImage display
    * Add an editable enum field to specify transport hint on OverlayImage display.
    * raw, compressed and theora are listed as pre-defined transport  hints.

* Unsubscribe image topic when "use image" is unchecked in CameraInfo display (`#732 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/732>`_)
* Fix format specifier (`#731 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/731>`_)

  * Use %u instead of %lu to print Ogre::Texture::getWidth() and Ogre::Texture::getHeight() because they return uint32.

* Do not subscribe image topic when rviz startups in OverlayImage display (`#733 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/733>`_)
  * Do not subscribe image topic when rviz startups in OverlayImage display
    * In order not to subscribe image topic when rviz startups with OverlayImage display disabled, always verify if the display is enabled before the display subscribes topic.
  * Unsubscribe image topic when "use image" is unchecked in CameraInfo display

* Support classification result visualization with approximate sync (`#725 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/725>`_)

  * classification_result_visualizer: add option to use approximate synchronizer

* Contributors: Yuki Furuta, Iki Yo, Naoki Mizuno, Naoki Hiraoka, Ryohei Ueda, Shingo Kitagawa, Yuto Uchimi, Iory Yanokura

2.1.5 (2019-02-18)
------------------
* [jsk_rviz_plugins] Add "Align Bottom" option to OverlayText (`#723 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/723>`_ )

  * Update config for easily understanding the effect of AlignBottom
  * Update overlay_sample.launch
  * Add rosparam to enable/disable reversing lines
  * Add "Align Bottom" option to overlay_text plugin

* Contributors: Yuto Uchimi

2.1.4 (2018-11-01)
------------------
* [jsk_rviz_plugins/target_visualize] Add visualizer\_ initilized flags (`#720 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/720>`_)
* replace boost::shared_ptr by std::shared_ptr (`#710 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/710>`_)
  * enable C++11
  * replace boost pointers by std pointers

* add error message to status (`#715 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/715>`_)
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Add comment for install destination
  * Fix path to headers for installation
  * Install missing test/ as well
  * Use source permission when installing executables
  * Fix installation destination

* [jsk_rviz_plugins/camera_info_display] Check fx and fy are not equal to zero. (`#1 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/1>`_)
* [jsk_rviz_plugins] Optimize camera info displaying (`#709 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/709>`_)
  * Split and merge image matrix channels instead of slow pixel-by-pixel copying while renderind camera info.

* [jsk_rviz_plugins] add segment_array_display (`#666 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/666>`_)
  * Add doc and sample of segment_array
  * add segment_array_display rviz plugin.

* [jsk_rviz_plugins] use QScreen::grabWindow() instead of QPixmap::grabWindow (`#700 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/700>`_)
  * [jsk_rviz_plugins] use QScreen::grabWindow() instead of QPixmap::grabWindow

* add enable lighitng property in polygon_array_display (`#686 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/686>`_)
* add jsk_rviz_plugins library to catkin_package LIBRARIES, use  instea… (`#696 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/696>`_)
  * add jsk_rviz_plugins library to catkin_package LIBRARIES
* Add #include <boost/format.hpp> (`#695 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/695>`_)

* jsk_rviz_plugins: warn on missing frame_id (`#698 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/698>`_)
* Suppress warnings of jsk_rviz_plugins for non-existent targets (`#693 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/693>`_)
  Support `catkin_make` also.
  Ref: https://github.com/jsk-ros-pkg/jsk_visualization/pull/692#issuecomment-390873758

* [jsk_rviz_plugins] fix std::isnan to make it compile under Ubuntu 16.04 / gcc 5 (`#687 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/687>`_)
  * fix std::isnan to make it compile under Ubuntu 16.04 / gcc 5
  * revert whitespace changes (adding trailing whitespace again)

* add enable lighitng property in polygon_array_display
* jsk_rviz_plugins: class_result_vis: add more types to vislalize (`#684 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/684>`_)
* jsk_rviz_plugins: add missing deps (`#683 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/683>`_)
* Add <url> to package.xml to add link to README (`#681 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/681>`_)
* Contributors: Aleksandr Rozhdestvenskii, Christian Rauch, Daniel Neumann, Yuki Furuta, Jan Carius, Kei Okada, Kentaro Wada, Laurenz, Masaki Murooka, Tamaki Nishino, Yuto Uchimi, Iori Yanokura

2.1.3 (2017-10-26)
------------------
* [jsk_rviz_plugins] Rviz default font is changed from Arial to LiberationSans (See: https://github.com/ros-visualization/rviz/pull/1141) (`#676 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/676>`_)
* Add exclude regex in rosconsole_overlay (`#675 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/675>`_)
* Contributors: Iori Kumagai, Kentaro Wada

2.1.2 (2017-07-07)
------------------
* [jsk_rviz_plugins][classification_result_visualizer] minor bugfix (`#669 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/669>`_ )
* [jsk_rviz_plugins] add marker publisher for classification result (`#667 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/667>`_)
  * [jsk_rviz_plugins] add visualizer for classification result

* Contributors: Yuki Furuta

2.1.1 (2017-02-15)
------------------
* remove depends to wxwidgets https://github.com/ros/rosdistro/pull/13886#issuecomment-279832181
* Contributors: Kei Okada

2.1.0 (2017-02-13)
------------------
* migration to kinetic, which uses qt5 wehre as indig/jade uses qt4 (`#662 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/662>`_ )
* Feature to transform markers in rviz (`#661 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/661>`_ )
  * Not to build transformable_marker_operator in jsk_rviz_plugins
  * Move TransformableMarkerOperatorAction to jsk_interactive_marker
  * Add feature to transform marker to rviz plugin
  * Add server_name for TransformableMarkerOperatorAction
* Contributors: Kentaro Wada, Hiroto Mizohana

2.0.1 (2016-12-15)
------------------

2.0.0 (2016-12-14)
------------------
* Stop using deprecated jsk_topic_tools/log_utils.h
  see
  - https://github.com/jsk-ros-pkg/jsk_common/pull/1462
  - https://github.com/jsk-ros-pkg/jsk_common/issues/1461
* [jsk_rviz_plugins/src/empty_service_call_interface.cpp] remove unused variables.
* Contributors: Kentaro Wada, MasakiMurooka

1.0.34 (2016-09-29)
-------------------
* Fix for Ogre >= 1.9, which build fail on Jade on 14.10/15.04
* [jsk_rviz_plugins] add offset to footstep_display.h
* [jsk_rviz_plugin] Add rviz button interface for yes/no service request
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi

1.0.33 (2016-09-13)
-------------------
* [doc/jsk_rviz_plugins/plugins/pie_chart.md, plotter_2d.md] add doc to how to change caption of overray text (Fix https://github.com/jsk-ros-pkg/jsk_visualization/issues/634)
* [jsk_rviz_plugins/CMakeLists.txt] Install samples dir that was missing for jsk_rviz_plugins (https://github.com/jsk-ros-pkg/jsk_visualization/issues/632)
* [jsk_rviz_plugins/samples/overlay_sample.py] Add queue_size arg for deprecated warning in overlay_sample.py (https://github.com/jsk-ros-pkg/jsk_visualization/issues/631)
* [jsk_rviz_plugins/src/overlay_text_display.cpp] Show available fonts using enum property (https://github.com/jsk-ros-pkg/jsk_visualization/issues/630)
* [jsk_rviz_plugins/src/overlay_picker_tool.cpp] handleDisplayClick was not going past first group  as after processing a group with no overlay item, it was still  returning true by default. It needed to return false to continue the
  seach (https://github.com/jsk-ros-pkg/jsk_visualization/issues/627)
* New rviz plugin to visualize jsk_recognition_msgs::BoundingBox (https://github.com/jsk-ros-pkg/jsk_visualization/issues/616)

  * [jsk_rviz_plugins/src/bounding_box_array_display.cpp] Show valid boxes even if invalid box is included

* Contributors: Jit Ray Chowdhury, Kei Okada, Kentaro Wada

1.0.32 (2016-07-20)
-------------------
* Show colorized ros logging on rviz overlay text
* Fix style of code of rosconsole_overlay_text.py
* Convert RGB to BGR precisely in video capturing
* Support multi legs in footstep_display
* Use small sized icons for faster adding display properties
  Fix https://github.com/jsk-ros-pkg/jsk_visualization/issues/603
* Cleanup jsk_rviz_plugins package.xml
* Fix moc generation errors with boost >= 1.57 (for OS X currently)
  Please refer to https://github.com/ros-visualization/rviz/pull/826
* Keep aspect ratio with only specified width for OverlayImage
* Contributors: Kentaro Wada, Eisoku Kuroiwa

1.0.31 (2016-05-19)
-------------------
* Stop passing -z flag to ld with Clang
* Contributors: Kentaro Wada

1.0.30 (2016-03-25)
-------------------
* use jsk_rviz_plugins::StringStamped instead of roseus::StringStamped, to remove roseus depends
* add rviz_DEFAULT_PLUGIN_LIBRARIES:  see https://github.com/ros-visualization/rviz/pull/979
* Contributors: Kei Okada

1.0.29 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_rviz_plugins] Do not show unnecessary properties of CameraInfo
* [jsk_rviz_plugins] Delete property in OverlayDiagnosticDisplay
* [jsk_rviz_plugins/OverlayDiagnostics] Add new style
* [jsk_rviz_plugins/OverlayPicker] Align to grid in pressing shift key
* Contributors: Kei Okada, Ryohei Ueda

1.0.28 (2016-02-03)
-------------------
* [jsk_rviz_plugins] Fix OverlayPicker for plugins gdouped by DisplayGroup.
* [jsk_rviz_plugins] Add keep aspect ratio option to OverlayImageDisplay.
  And ass overlay image sample to overlay_sample.launch
* [jsk_rviz_plugins] Add new tool OverlayPicker to move overlay plugin
  by mouse dragging
* [jsk_rviz_plugins] Add script to visualize static OverlayText
* [jsk_rviz_plugins] Support multiple Float32 in float32_to_overlay_text.py
* [jsk_rviz_plugins] Utility script to draw float32 as overlay text
  Added:
  - jsk_rviz_plugins/scripts/float32_to_overlay_text.py
* [jsk_rviz_plugins] Add utility python class to publish OverlayText
  Modified:
  - jsk_rviz_plugins/CMakeLists.txt
  Added:
  - jsk_rviz_plugins/cfg/OverlayTextInterface.cfg
  - jsk_rviz_plugins/python/jsk_rviz_plugins/__init_\_.py
  - jsk_rviz_plugins/python/jsk_rviz_plugins/overlay_text_interface.py
  - jsk_rviz_plugins/setup.py
* [jsk_rviz_plugins] Add regular expression interface to specify
  target node in rosconsole_overlay_text.py
  Modified:
  - jsk_rviz_plugins/scripts/rosconsole_overlay_text.py
* [jsk_rviz_plugins] Fix allocateShapes API of TorusArrayDisplay
  Modified:
  - jsk_rviz_plugins/src/torus_array_display.cpp
  - jsk_rviz_plugins/src/torus_array_display.h
* [jsk_rviz_plugins] Add script to visualize /rosout on rviz
  Added:
  - jsk_rviz_plugins/scripts/rosconsole_overlay_text.py
* Merge remote-tracking branch 'refs/remotes/garaemon/clear-torus-arrow' into many-prs
* [jsk_rviz_plugins] Support css to change text color and so on in OverlayText display.
* [jsk_rviz_plugins] A script to take screenshot of rviz when a topic is
  published: relay_screenshot.py
  It depends on ScreenshotListener tool of jsk_rviz_plugins.
  For example, `rosrun jsk_rviz_plugins relay_screenshot.py ~input:=/region_growing_multi_plane_segmentation/output/polygons`
* [jsk_rviz_plugins] Clear arrow of torus when it is disabled
  Modified:
  jsk_rviz_plugins/src/torus_array_display.cpp
* [jsk_rviz_plugins] Disable `show coords` in default in BoundingBoxDisplay
  Modified:
  jsk_rviz_plugins/src/bounding_box_array_display.cpp
* [jsk_rviz_plugins] Do not visualize failure=true toruses.
  It requires https://github.com/jsk-ros-pkg/jsk_recognition/pull/1379
* Contributors: Ryohei Ueda

1.0.27 (2015-12-08)
-------------------
* [jsk_rviz_plugins/BoundingBoxArray] Fix coords orientation.
  closes `#528 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/528>`_
* Use ccache to make it faster to generate object file
* [jsk_rviz_plugins] Empty function implementation for undefined methods
* [jsk_rviz_plugins] Use set_target_properties to set linker flags only
  for libjsk_rviz_plugins.so
* Use gcc -z defs to check undefined symbols in shared objects
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.26 (2015-12-03)
-------------------
* [jsk_rviz_plugins] Install icons
* [jsk_rviz_plugins] add landing_time_detector to display early landing/taking-off
* [jsk_rviz_plugins/motor_states_temparature_decomposer] Decrease cpu load
  by queue_size=1.
  Fix for joints which does not have limit attribute.
* [jsk_rviz_plugins] Add ~parent_link parameter for contact_state_publisher
* [jsk_rviz_plugins] Add dynamic_reconfigure API to ContactStateMarker
* [jsk_rviz_plugins] Check size of likelihood and labels of PolygonArray
* [jsk_rviz_plugins/contact_state_marker.py] Support origin attribute of
  visual tag
* [jsk_rviz_plugins] update ambient sound visual paramter
* [jsk_rviz_plugins] contact_state_marker.py to visualize hrpsys_ros_bridge/ContactStatesStamped
* [jsk_rviz_plugins] Add script to publish marker of a robot link with
  specified color
* Contributors: Eisoku Kuroiwa, Kentaro Wada, Ryohei Ueda, Yuto Inagaki

1.0.25 (2015-10-10)
-------------------
* [jsk_rviz_plugins] Fix font size of PeoplePositionMeasurementArray
* [jsk_rviz_plugins] Add script for diagnostics sample
* [jsk_rviz_plugins] Compile PeoplePositionMeasurementArrayDisplay
* [jsk_rviz_plugins/VideoCapture] Check file permission to write correctly
* [jsk_rviz_plugins] Use readthedocs to document
* [jsk_rviz_plugins] Add index page for sphinx + readthedocs
* [jsk_rviz_plugins] Use jsk_recognition_utils instead of jsk_pcl_ros to
  speed up compilation
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.24 (2015-09-08)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Fix compilation error because of
  the latest jsk_recongition_utils changes
* [jsk_rqt_plugins/TwistStamped] Fix duplicated delete
* [jsk_rviz_plugins] Allow width/height 0 image (fix segfault)
* [jsk_rviz_plugins/PolygonArray] Coloring by labels and likelihood fields
  of jsk_recognition_msgs/PolygonArray
* [jsk_rviz_plugins/TwistStamped] Decide circle thickness according to
  radius of circle
* [jsk_rviz_plugins/BoundingBoxArray] Normalize value color gradation
* [jsk_rviz_plugins/BoundingBoxArray] Update coloring method to support
  coloring by values and labels.
* [jsk_rviz_plugins] Remove footstep texts from rviz when reset the plugin
* [jsk_rqt_plugins] Add sample launch for PolygonArray
* [jsk_rviz_plugins/PolygonArray] Use enum property to choose coloring method
* [jsk_rviz_plugins/TfTrajectory] Use status property to show error rather than
  ROS_ERROR
* [jsk_rviz_plugins/RobotCommandInterface] Use smaller icon size
* [jsk_rviz_plugins] Use ~robot_command_buttons parameter to configure RobotCommandInterfaceAction
* [jsk_rviz_plugins/TFTrajectory] Initialize line width
* [jsk_rviz_plugins/TFTrajectory] Add movie link to README
* [jsk_rviz_plugins] A rviz plugin to visualize tf trajectory as path
* [jsk_rviz_plugins][OverlayImage] Automatically setup size with negative val
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------
* [jsk_rviz_plugins/PoseArray] Clear pose array if checkbox is unchecked
* fix coords bug
* Contributors: Ryohei Ueda, Yu Ohara

1.0.22 (2015-06-24)
-------------------
* [jsk_rviz_plugins/OverlayImage] Support alpha channel if image_encoding
  is BGRA8 or RGBA8
* Contributors: Ryohei Ueda

1.0.21 (2015-06-11)
-------------------
* [jsk_rviz_plugins/PolygonArrayDisplay] Cleanup codes to be within 80 columns
* [jsk_rviz_plugins/BoundingBoxArray] Immediately apply change of attributes
* [jsk_rviz_plugins/BoundingBoxArray] Refactor codes by splitting processMessages into several functions
* [jsk_rviz_plugins/BoundingBoxArray] Use symmetrical radius for coordinates arrow
* [jsk_rviz_plugins/BoundingBoxArray] Fix coding style around if/else/for
* [jsk_rviz_plugins/BoundingBoxArray] Check if the size of box is nan
* [jsk_rviz_plugins/BoundingBoxArray] Fix indent to be within 80 columns
* Contributors: Ryohei Ueda

1.0.20 (2015-05-04)
-------------------
* [jsk_rviz_plugins] add rotate speed to pictogram
* [jsk_rviz_plugins] add String PopupMode for Pictogram
* [jsk_rviz_plugins] Make arrow nodes invisible as default in PolygonArrayDisplay not to show normal if no needed
* [jsk_rviz_plugins] Check size of BoundingBox
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.19 (2015-04-09)
-------------------
* [jsk_rviz_plugins] Fix initialization order in Plotter2DDisplay in order  to avoid call std::vector::resize with uninitialized length
* [jsk_rviz_plugins] Obsolate SparseOccupancyGridArray, it's replaced by SimpleOccupancyGridArray
* [jsk_rviz_plugins] Use jsk_pcl_ros/geo_util to reconstruct 3d
  information in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add image of SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support auto coloring in SimpleOccupancyGridArray
* [jsk_rviz_plugins] Support 4th parameter of plane coefficients in SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] Add SimpleOccupancyGridArrayDisplay
* [jsk_rviz_plugins] add tmp pose array display
* [jsk_rviz_plugins] Change plotter color from 30%
* add_mesh_model_in_transformable_marker
* [jsk_rviz_plugins] Do not update min/max value when re-enabling Plotter2D
* [jsk_rviz_plugins] Change color of plotter from 50 percent of max value
* [jsk_rviz_plugins] add showing coords option for bounding box array display
* [jsk_rviz_plugins] Add utility script to visualize difference between to tf frame on rviz
* [jsk_rviz_plugins] Check direction vector is non-nan in PolygonArrayDisplay
* [jsk_pcl_ros] Fix license: WillowGarage -> JSK Lab
* [jsk_pcl_ros] Fix install path and install headers
* [jsk_rviz_plugins] Do not show disabled properties of OverlayText, Plotter2D and PieChart
* [jsk_pcl_ros] Make overlay sample more faster
* [jsk_rviz_plugins] Change color from 60 percent of maximum value in PieChartDisplay and Plotter2DDisplay
* [jsk_rviz_plugins] Draw PieChart at the first time
* Remove rosbuild files
* [jsk_rviz_plugins] Update PieChartDisplay only if value changed
* [jsk_rviz_plugins] Do not change texture size and position in processMessage
* [jsk_rviz_plugins] Optimize PieChartDisplay, draw image in update() method instead of processMessage
* Contributors: Ryohei Ueda, Yu Ohara, Yuto Inagaki

1.0.18 (2015-01-30)
-------------------
* add depends to cv_bridge instaed of opencv2

1.0.17 (2015-01-29)
-------------------
* [jsk_rviz_plugins] Add TwistStampedDisplay
* [jsk_rviz_plugins] Use jsk_recognition_msgs
* update README file for mainly panels
* [jsk_rviz_plugins] Add document of PolygonArray display
* add publishing pointcloud information as overlay text
* add record action panel
* remove unused QLineEdit variable
* add normal option for torus display
* [jsk_rviz_plugins] Refactor PolygonArrayDisplay class
* [jsk_rviz_plugins] Add "Show Normal" to PolygonArrayDisplay
* add object fit operator panel
* Make torus more smooth and add beatiful parameter
* add torus array display
* Contributors: Ryohei Ueda, JSK Lab member, Yuto Inagaki

1.0.16 (2015-01-04)
-------------------
* [jsk_rviz_plugins] Fix namespace of TabletViewController
* [jsk_rviz_plugins] Fix namespace jsk_rviz_plugin -> jsk_rviz_plugins
* [jsk_rviz_plugins] Utility script to draw the number of samples during
  capturing data
* [jsk_rviz_plugins] Remove invalid codes of ScreenshotListenerTool
* [jsk_rviz_plugins] VideoCaptureDisplay Display to capture rviz as movie
* [jsk_rviz_plugins] ScreenshotListenerTool: A simple tool to listen to
  a service and save screenshot to specified file
* [jsk_rviz_plugins] Avoid Segmentation Fault when size 0 texture is
  specified

1.0.15 (2014-12-13)
-------------------
* Add new plugin and message to display array of pictograms
* Remove pictogram when the display is disabled
* Fix policy to move head using rviz: Do not consider movement of mouse,
  just use the position of the mouse. Because we cannot ignore
  network latency
* Fix several parameters suitable for surface
* Add panel for tablet demonstration
* Add view_controller_msgs
* Compute difference to mouse position
* Add TabletViewController to control robot from tablet using rviz
* Check texture is available or not when initializing CameraInfo
* Paster image on the bottom of the camera parameter pyramid
* Contributors: Ryohei Ueda

1.0.14 (2014-12-09)
-------------------
* Add more action to pictogram
* Add documentation about pictogram
* Do not rewrite texture if no need
* Add sample to visualize all the pictograms
* Add FontAwesome fonts and several improvements about font drawing:
  1) decide size of font according to font metrics
  2) do not re-write pictogram texture if no need
* Support deletion of pictogram
* Add color field to Pictogram.msg
* Add sample script for pictogram
* Add display to visualize pictogram
* fixed parameter namespace mismatch.
* set the components to align left
* added button for start_impedance_for_drill
* added service to check marker existence. added copy to marker operation.
* fix quatation signiture for function name in robot_command_interface.cpp
* refact and delete some unneeded includes
* add empty_service_call_interface
* add robot_command_interface
* Change the size of menu according to the change of title and fix
  position of the popup window if the window is larger than the rviz
* Use name for decomposed topic of motor_states_temperature_decomposer.py
* Change color of text according to the foreground color of PieChart
* Show value as string on Plotter2DDisplay
* Decompose joint_state's effort value and read the max value from robot_description
* Fix motor_state_decomposer.py
* Take title into account to decide the size of OverlayMenu
* compacting the panel with using tab
* move msg to jsk_rviz_plugins
* add depend on jsk_interactive_marker
* add transformable marker operator panel
* Coloring footstep by jsk_footstep_msgs::Footstep::footstep_group
* Show text on footstep to display left or right
* Separate 'OvertakeProperties' into 'Overtake Color Properties' and
  'Overtake Position Properties'
* Script to decompose MotorStates/temperature into std_msgs/Float32
* Contributors: Ryohei Ueda, Masaki Murooka, Yuto Inagaki

1.0.13 (2014-10-10)
-------------------
* Add "overtake properties" property to OverlayTextDisplay
* Call queueRender after opening/closing properties in Open/CloseAllTool
* Contributors: Ryohei Ueda

1.0.12 (2014-09-23)
-------------------

1.0.11 (2014-09-22)
-------------------
* Do not ues deprecated PLUGINLIB_DECLARE_CLASS
* Draw polygon as 'face' on PolygonArrayDisplay
* Use jsk_topic_tools::colorCategory20 to colorize automatically
* Add tool plugin to close/open all the displays on rviz
* Contributors: Ryohei Ueda

1.0.10 (2014-09-13)
-------------------
* Fix color of people visualizer by initializing color to sky blue
* Fix texture color of camera info by filling color value of texture image
* Fix caching of overlay textures of OverlayMenuDisplay to support
  changing menus
* add relay camera info node
* Add new plugin to visualize sensor_msgs/CameraInfo
* Ignore first message means CLOSE in OverlayMenuDisplay
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.9 (2014-09-07)
------------------

1.0.8 (2014-09-04)
------------------
* add enum menu to TargetVisualizer and PeoplePositionMeasurementDisplay
  to select the style of the visualizer
* do not depends on people_msgs on groovy
* add SimpeCircleFacingVisualizer class
* separate a code to draw visualizer into facing_visualizer.cpp
* add rviz plugin for face_detector
* cleanup package.xml of jsk_rviz_plugins
* Contributors: Ryohei Ueda

1.0.7 (2014-08-06)
------------------
* show "stalled" if no diagnostic message received in OverlayDiagnosticDisplay
* add utility class for Overlay: OverlayObject and ScopedPixelBuffer in overlay_utils.cpp
* spcify max/min values for the properties of Plotter2D
* fix color error when changing the size of the window of Plotter2D
* add offset to compute the absolute position of the grid
* Remove non-used color property in OverlayDiagnosticsDisplay
* Remove OverlayDiagnostic correctly (not remaining overlay texture).
* under line of the caption should be longer than the length of the
  caption in TargetVisualizer
* align the position of the text of TargetVisualizer to left
* add CancelAction and PublishTopic plugin to hydro of jsk_rviz_plugin
* add visualizer to visualize pose stamped with target mark
* Contributors: Ryohei Ueda

1.0.6 (2014-07-14)
------------------
* add new plugin to visualize diagnostic status on ovrelay layer
* hide movable text of DiagnosticDisplay at first
* support font size field in DiagnosticDisplay
* diagnostics namespace and frame_id fields of DiagnosticsDisplay is now
  selectable according to the current ROS topics
* support axis color to colorize SparseOccupancyGridMap
* use rviz::PointCloud to render jsk_pcl_ros::SparseOccupancyGridArray to optimize
* hotfix to fix the position of overlay text
* does not update scale if the dimension is same to the previous data in OccupancyGridDisplay
* implement rviz plugin to visualize jsk_pcl_ros::SparseOccupancyGridArray
* add QuietInteractiveMarker
* Contributors: Ryohei Ueda

1.0.5 (2014-06-29)
------------------
* add overlay camera display
* close overlay menu firmly
* add new rviz plugin: OverlayImage
  visualize sensor_msgs::Image as HUD on rviz 3D rendering window
* add new plugin: OverlayMenu
* Contributors: Ryohei Ueda

1.0.4 (2014-05-31)
------------------
* jsk_rviz_plugins: use depend tag add mk/rosbuild to build_depend
* update the initial parameter of FootstepDisplay
* add line width property to BoundingBoxArrayDisplay
* add new plugin: BoundingBoxArray for jsk_pcl_ros/BoundingBoxArray
* Contributors: Ryohei Ueda, Kei Okada

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
* Contributors: Ryohei Ueda, Kei Okada

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
* Contributors: Shohei Fujii, Youhei Kakiuchi, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Ryohei Ueda, Yusuke Furuta
