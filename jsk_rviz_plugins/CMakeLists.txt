# $ENV{ROS_VERSION} is set by ros_environment, which is missing on
# pre-ros_environment distros (e.g. indigo) and on ROS 2 builds that do not
# depend on it (e.g. the buildfarm's binary jobs). Guess it then: ament sets
# AMENT_PREFIX_PATH (bloom's debian/rules passes it to cmake, setup.sh and
# colcon set the environment variable), catkin never does.
if("$ENV{ROS_VERSION}" STREQUAL "")
  if(DEFINED AMENT_PREFIX_PATH OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_rviz_plugins)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
  endif()

  # The sources are shared with ROS 1 and tell the two apart with
  # `#if ROS_VERSION_MAJOR != 1` (src/rviz_compat.h). On ROS 2 they are
  # built against rviz_common, rviz_rendering and rviz_default_plugins.
  #
  # The following plugins are provided by upstream packages on ROS 2 and
  # therefore not built here:
  #   camera_info_display -> rviz_default_plugins/CameraInfo (https://github.com/ros2/rviz)
  #   pose_array_display  -> rviz_default_plugins/PoseArray, which already provides the
  #                          Arrow/Axes shapes this plugin added on ROS 1
  #
  # The following plugins wait for their dependencies to be released for
  # ROS 2 and stay ROS 1 only until then:
  #   src/ambient_sound_display.{h,cpp}                     -> jsk_hark_msgs (not built on
  #                                                            ROS 1 either, see ambient_sound_display_groovy)
  #   src/polygon_array_display.{h,cpp}                     -> jsk_recognition_msgs, jsk_recognition_utils
  #   src/simple_occupancy_grid_array_display.{h,cpp}       -> jsk_recognition_msgs, jsk_recognition_utils
  #   src/rviz_util.h                                       -> only used by simple_occupancy_grid_array_display
  #   src/people_position_measurement_array_display.{h,cpp} -> people_msgs
  #   src/view_controller/tablet_view_controller.{h,cpp}    -> view_controller_msgs
  #
  # src/overlay_camera_display.{h,cpp} is blocked on rviz itself: it reads the rendered
  # scene back with Ogre::RenderTarget::copyContentsToMemory(), but rviz_rendering does
  # not expose the Ogre render target behind rviz_common::RenderPanel::getRenderWindow()
  # (see rviz_rendering::RenderWindowOgreAdapter). Port it once rviz_rendering exposes it.
  #
  # The dynamic_reconfigure configs (cfg/), the python module (python/,
  # dynamic_reconfigure based) and the scripts/ are ROS 1 only as well.
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(jsk_hark_msgs REQUIRED)
  find_package(jsk_footstep_msgs REQUIRED)
  find_package(jsk_gui_msgs REQUIRED)
  find_package(jsk_ros1_ros2_compat REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(resource_retriever REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(rviz_default_plugins REQUIRED)
  find_package(rviz_ogre_vendor REQUIRED)
  find_package(rviz_rendering REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  # cv_bridge only exports opencv_core/opencv_imgproc; video_capture_display
  # uses cv::VideoWriter of opencv_videoio
  find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ObjectFitCommand.msg
    msg/OverlayMenu.msg
    msg/OverlayText.msg
    msg/Pictogram.msg
    msg/PictogramArray.msg
    msg/RecordCommand.msg
    msg/StringStamped.msg
    msg/TransformableMarkerOperate.msg
    srv/EusCommand.srv
    srv/RequestMarkerOperate.srv
    srv/Screenshot.srv
    DEPENDENCIES std_msgs geometry_msgs
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

  set(HEADER_FILES
    src/ambient_sound_display_groovy.h
    src/ambient_sound_visual.h
    src/cancel_action.h
    src/close_all_tool.h
    src/diagnostics_display.h
    src/empty_service_call_interface.h
    src/facing_visualizer.h
    src/footstep_display.h
    src/image_transport_hints_property.h
    src/linear_gauge_display.h
    src/normal_display.h
    src/normal_visual.h
    src/object_fit_operator.h
    src/open_all_tool.h
    src/overlay_diagnostic_display.h
    src/overlay_image_display.h
    src/overlay_menu_display.h
    src/overlay_picker_tool.h
    src/overlay_text_display.h
    src/overlay_utils.h
    src/pictogram_array_display.h
    src/pictogram_display.h
    src/pie_chart_display.h
    src/plotter_2d_display.h
    src/publish_topic.h
    src/quiet_interactive_marker_display.h
    src/record_action.h
    src/robot_command_interface.h
    src/rviz_scene_publisher.h
    src/screenshot_listener_tool.h
    src/select_point_cloud_publish_action.h
    src/string_display.h
    src/tablet_controller_panel.h
    src/target_visualizer_display.h
    src/tf_trajectory_display.h
    src/twist_stamped_display.h
    src/video_capture_display.h
    src/yes_no_button_interface.h
  )
  set(SOURCE_FILES
    src/ambient_sound_display_groovy.cpp
    src/ambient_sound_visual.cpp
    src/cancel_action.cpp
    src/close_all_tool.cpp
    src/diagnostics_display.cpp
    src/empty_service_call_interface.cpp
    src/facing_visualizer.cpp
    src/footstep_display.cpp
    src/image_transport_hints_property.cpp
    src/linear_gauge_display.cpp
    src/normal_display.cpp
    src/normal_visual.cpp
    src/object_fit_operator.cpp
    src/open_all_tool.cpp
    src/overlay_diagnostic_display.cpp
    src/overlay_image_display.cpp
    src/overlay_menu_display.cpp
    src/overlay_picker_tool.cpp
    src/overlay_text_display.cpp
    src/overlay_utils.cpp
    src/pictogram_array_display.cpp
    src/pictogram_display.cpp
    src/pie_chart_display.cpp
    src/plotter_2d_display.cpp
    src/publish_topic.cpp
    src/quiet_interactive_marker_display.cpp
    src/record_action.cpp
    src/robot_command_interface.cpp
    src/rviz_scene_publisher.cpp
    src/screenshot_listener_tool.cpp
    src/select_point_cloud_publish_action.cpp
    src/string_display.cpp
    src/tablet_controller_panel.cpp
    src/target_visualizer_display.cpp
    src/tf_trajectory_display.cpp
    src/twist_stamped_display.cpp
    src/video_capture_display.cpp
    src/yes_no_button_interface.cpp
  )

  # jsk_recognition_msgs is not released for ROS 2 yet, so its displays are
  # built only when it is found (e.g. built from source in the workspace, as
  # the ROS 2 CI does) and exported with their own plugin description file.
  # PolygonArray and SimpleOccupancyGridArray need jsk_recognition_utils as
  # well (polygon triangulation, planes, PCL conversions), which is not ported
  # to ROS 2, so they stay ROS 1 only.
  find_package(jsk_recognition_msgs QUIET)
  if(jsk_recognition_msgs_FOUND)
    list(APPEND HEADER_FILES
      src/bounding_box_array_display.h
      src/bounding_box_display.h
      src/bounding_box_display_common.h
      src/human_skeleton_array_display.h
      src/segment_array_display.h
      src/torus_array_display.h
    )
    list(APPEND SOURCE_FILES
      src/bounding_box_array_display.cpp
      src/bounding_box_display.cpp
      src/human_skeleton_array_display.cpp
      src/segment_array_display.cpp
      src/torus_array_display.cpp
    )
  else()
    message(STATUS "BoundingBox, BoundingBoxArray, TorusArray, SegmentArray and HumanSkeletonArray "
      "displays skipped: jsk_recognition_msgs not found")
  endif()

  set(CMAKE_AUTOMOC ON)
  # rosidl_generate_interfaces() owns the ${PROJECT_NAME} target, so the
  # library target is named differently and renamed back on output, to keep
  # the libjsk_rviz_plugins.so that rviz_common_plugins.xml loads.
  add_library(${PROJECT_NAME}_plugins SHARED ${HEADER_FILES} ${SOURCE_FILES})
  set_target_properties(${PROJECT_NAME}_plugins PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
  target_compile_features(${PROJECT_NAME}_plugins PUBLIC cxx_std_17)
  # The sources include the Ogre headers as <OGRE/...>, as on ROS 1, while
  # rviz_ogre_vendor exports .../include/OGRE itself.
  target_include_directories(${PROJECT_NAME}_plugins PRIVATE
    src
    ${jsk_ros1_ros2_compat_INCLUDE_DIRS}
  )
  target_include_directories(${PROJECT_NAME}_plugins SYSTEM PRIVATE
    ${rviz_ogre_vendor_DIR}/../../../opt/rviz_ogre_vendor/include
  )
  target_link_libraries(${PROJECT_NAME}_plugins
    ${cpp_typesupport_target}
    ${jsk_gui_msgs_TARGETS}
    ${jsk_footstep_msgs_TARGETS}
    ${jsk_hark_msgs_TARGETS}
    ${action_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    image_transport::image_transport
    pluginlib::pluginlib
    rclcpp::rclcpp
    resource_retriever::resource_retriever
    rviz_common::rviz_common
    rviz_default_plugins::rviz_default_plugins
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
    rviz_rendering::rviz_rendering
    Qt5::Widgets
    opencv_videoio
  )
  if(jsk_recognition_msgs_FOUND)
    target_link_libraries(${PROJECT_NAME}_plugins ${jsk_recognition_msgs_TARGETS})
  endif()
  install(TARGETS ${PROJECT_NAME}_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
  if(jsk_recognition_msgs_FOUND)
    pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins_jsk_recognition_msgs.xml)
  endif()

  install(DIRECTORY config icons launch
    DESTINATION share/${PROJECT_NAME}
  )

  # The samples of launch/*.launch.py. They keep `#!/usr/bin/env python` for
  # ROS 1 (python2 on indigo..melodic); ROS 2 base images ship no bare
  # `python`, so the installed ROS 2 copies get a python3 shebang.
  jsk_ros1_ros2_compat_install_python(PROGRAMS
    samples/overlay_menu_sample.py
    samples/overlay_sample.py
    samples/pictogram.py
    samples/pictogram_all.py
    samples/piechart_sample.py
  )

  if(BUILD_TESTING)
    # rviz2 with the sample configs; the tests need an X display and skip
    # themselves without one (CI runs them under Xvfb)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/ros2/test_overlay.py TIMEOUT 120)
    add_launch_test(test/ros2/test_linear_gauge.py TIMEOUT 120)
    add_launch_test(test/ros2/test_pictogram.py TIMEOUT 120)
    add_launch_test(test/ros2/test_piechart.py TIMEOUT 120)
    add_launch_test(test/ros2/test_service_call_panel.py TIMEOUT 120)
    add_launch_test(test/ros2/test_robot_command_interface.py TIMEOUT 120)
  endif()

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
# these plugins should be uploaded to upstream repository
# https://github.com/ros-visualization/rviz/pull/634

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_rviz_plugins)
if(("$ENV{ROS_DISTRO}" STREQUAL "kinetic") OR ("$ENV{ROS_DISTRO}" STREQUAL "indigo"))
  # catch special case ROS kinetic where c++11 is not the default yet
  add_compile_options(-std=c++11)
endif()

# support environment does not have visualization such as VTK or Rviz (i.e embedded environment)
set(USE_VISUALIZATION ON CACHE BOOL "compile with rviz package")
if(USE_VISUALIZATION)
  set(RVIZ "rviz")
  set(JSK_RECOGNITION_UTILS "jsk_recognition_utils") # jsk_recognition_utils depends on pcl/visualization (https://github.com/jsk-ros-pkg/jsk_recognition/blob/09cbce5f0ce7102331f6e48b9f9befc2b1fda869/jsk_recognition_utils/src/pcl_conversion_util.cpp#L37)
else()
  set(RVIZ "")
  set(JSK_RECOGNITION_UTILS "")
  message(WARNING "Compile without RVIZ/jsk_recognition_utils package")
endif()

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS ${RVIZ} jsk_hark_msgs jsk_footstep_msgs ${JSK_RECOGNITION_UTILS}
  people_msgs
  message_generation std_msgs diagnostic_msgs cv_bridge
  jsk_topic_tools
  image_geometry
  view_controller_msgs
  geometry_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
  jsk_gui_msgs
  message_filters)

# Use ccache if installed to make it fast to generate object files
if (CMAKE_VERSION VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

add_message_files(FILES
  OverlayText.msg OverlayMenu.msg TransformableMarkerOperate.msg ObjectFitCommand.msg
  Pictogram.msg PictogramArray.msg RecordCommand.msg StringStamped.msg)
add_service_files(FILES
  RequestMarkerOperate.srv EusCommand.srv
  Screenshot.srv
  )
catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/ClassificationResultVisualizer.cfg
  cfg/ContactStateMarker.cfg
  cfg/OverlayTextInterface.cfg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

if(USE_VISUALIZATION)
  set(${LIBRARIES_PROJECT_NAME} ${PROJECT_NAME})
else()
  set(${LIBRARIES_PROJECT_NAME} "")
endif()
catkin_package(
    DEPENDS ${RVIZ}
    CATKIN_DEPENDS jsk_hark_msgs jsk_footstep_msgs
    ${JSK_RECOGNITION_UTILS} cv_bridge people_msgs image_geometry
    INCLUDE_DIRS # TODO include
    LIBRARIES ${LIBRARIES_PROJECT_NAME}
)


set(ROS_BUILD_TYPE Release)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

if(USE_VISUALIZATION AND rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
elseif(USE_VISUALIZATION)
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

set(HEADER_FILES
  src/ambient_sound_display_groovy.h
  src/select_point_cloud_publish_action.h
  src/footstep_display.h
  src/publish_topic.h
  src/cancel_action.h
  src/record_action.h
  src/polygon_array_display.h
  src/normal_display.h
  src/overlay_text_display.h
  src/overlay_menu_display.h
  src/overlay_image_display.h
  src/overlay_camera_display.h
  src/plotter_2d_display.h
  src/pie_chart_display.h
  src/linear_gauge_display.h
  src/diagnostics_display.h
  src/quiet_interactive_marker_display.h
  src/bounding_box_display.h
  src/bounding_box_array_display.h
  src/torus_array_display.h
  src/overlay_diagnostic_display.h
  src/target_visualizer_display.h
  src/people_position_measurement_array_display.h
  src/simple_occupancy_grid_array_display.h
  src/camera_info_display.h
  src/close_all_tool.h
  src/open_all_tool.h
  src/overlay_picker_tool.h
  src/object_fit_operator.h
  src/robot_command_interface.h
  src/empty_service_call_interface.h
  src/pictogram_display.h
  src/pictogram_array_display.h
  src/view_controller/tablet_view_controller.h
  src/tablet_controller_panel.h
  src/video_capture_display.h
  src/rviz_scene_publisher.h
  src/twist_stamped_display.h
  src/pose_array_display.h
  src/tf_trajectory_display.h
  src/yes_no_button_interface.h
  src/segment_array_display.h
  src/image_transport_hints_property.h
  src/string_display.h
  src/human_skeleton_array_display.h
)

message(STATUS "Check rviz_FOUND: [${rviz_FOUND}], rviz_QT_VERSION: [${rviz_QT_VERSION}]")
if("${rviz_QT_VERSION}" STREQUAL "")
  message(WARNING "Compile without RVIZ package : skip qt_wrap_cpp")
elseif(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  src/ambient_sound_display_groovy.cpp
  src/ambient_sound_visual.cpp
  src/footstep_display.cpp
  src/publish_topic.cpp
  src/cancel_action.cpp
  src/record_action.cpp
  src/select_point_cloud_publish_action.cpp
  src/polygon_array_display.cpp
  src/normal_display.cpp
  src/normal_visual.cpp
  src/overlay_text_display.cpp
  src/overlay_menu_display.cpp
  src/overlay_image_display.cpp
  src/overlay_camera_display.cpp
  src/plotter_2d_display.cpp
  src/pie_chart_display.cpp
  src/linear_gauge_display.cpp
  src/diagnostics_display.cpp
  src/bounding_box_display.cpp
  src/bounding_box_array_display.cpp
  src/torus_array_display.cpp
  src/quiet_interactive_marker_display.cpp
  src/target_visualizer_display.cpp
  src/overlay_diagnostic_display.cpp
  src/simple_occupancy_grid_array_display.cpp
  src/people_position_measurement_array_display.cpp
  src/overlay_utils.cpp
  src/facing_visualizer.cpp
  src/camera_info_display.cpp
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/overlay_picker_tool.cpp
  src/screenshot_listener_tool.cpp
  src/object_fit_operator.cpp
  src/robot_command_interface.cpp
  src/empty_service_call_interface.cpp
  src/pictogram_display.cpp
  src/pictogram_array_display.cpp
  src/view_controller/tablet_view_controller.cpp
  src/tablet_controller_panel.cpp
  src/video_capture_display.cpp
  src/rviz_scene_publisher.cpp
  src/twist_stamped_display.cpp
  src/pose_array_display.cpp
  src/tf_trajectory_display.cpp
  src/yes_no_button_interface.cpp
  src/segment_array_display.cpp
  src/image_transport_hints_property.cpp
  src/string_display.cpp
  src/human_skeleton_array_display.cpp
  ${MOC_FILES}
)
if(jsk_recognition_msgs_VERSION VERSION_LESS 1.2.15)
  message(WARNING "jsk_recognition_msgs ${jsk_recognition_msgs_VERSION} < 1.2.15 does not support human_skeleton_array_display.cpp")
  list(REMOVE_ITEM HEADER_FILES src/human_skeleton_array_display.h)
  list(REMOVE_ITEM SOURCE_FILES src/human_skeleton_array_display.cpp)
  list(REMOVE_ITEM SOURCE_FILES  ${PROJECT_BINARY_DIR}/src/moc_human_skeleton_array_display.cxx)
endif()

if(NOT USE_VISUALIZATION)
  message(WARNING "polygon_array_display, simple_occupancy_grid_array_display requries jsk_recognition_utils")
  list(REMOVE_ITEM HEADER_FILES src/polygon_array_display.h)
  list(REMOVE_ITEM SOURCE_FILES src/polygon_array_display.cpp)
  list(REMOVE_ITEM HEADER_FILES src/simple_occupancy_grid_array_display.h)
  list(REMOVE_ITEM SOURCE_FILES src/simple_occupancy_grid_array_display.cpp)
endif()

if(rviz_FOUND)
add_library(jsk_rviz_plugins ${SOURCE_FILES} ${UIC_FILES})
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(jsk_rviz_plugins Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()
add_dependencies(jsk_rviz_plugins
  jsk_footstep_msgs_generate_messages_cpp
  jsk_gui_msgs_generate_messages_cpp
  jsk_hark_msgs_generate_messages_cpp
  jsk_recognition_msgs_generate_messages_cpp
  people_msgs_generate_messages_cpp
  view_controller_msgs_generate_messages_cpp
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS})

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(jsk_rviz_plugins PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(TARGETS jsk_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
endif(rviz_FOUND)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cfg config icons launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
file(GLOB PYTHON_SAMPLES samples/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SAMPLES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/samples
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if (CATKIN_ENABLE_TESTING)
  # download and install sample data
  add_custom_target(${PROJECT_NAME}_install_sample_data COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/test/install_sample_data.py)
  add_dependencies(tests ${PROJECT_NAME}_install_sample_data)
  # run rostest for melodic+
  find_package(rostest REQUIRED)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo") # support from kinetic
    file(GLOB TEST_FILES test/*.test test/*.xml)
    foreach(TEST_FILE ${TEST_FILES})
      # if we failed to install sample data, skip testing face_detect and normal
      if((NOT EXISTS "${PROJECT_SOURCE_DIR}/test/data/2017-06-20-12-00-00_people_images_in_lab.bag") AND
	  ((${TEST_FILE} STREQUAL "${PROJECT_SOURCE_DIR}/test/face_detector.xml") OR
	    (${TEST_FILE} STREQUAL "${PROJECT_SOURCE_DIR}/test/normal.xml")))
	message(STATUS "Skip test for ${TEST_FILE} due to missing bag file")
	continue()
      endif()
      message(STATUS "Run test for ${TEST_FILE}")
      add_rostest(${TEST_FILE})
    endforeach()
  endif()
endif()
endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
