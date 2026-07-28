cmake_minimum_required(VERSION 3.12)
project(jsk_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# In ROS 2, the following plugins are already provided by upstream packages and
# are therefore not ported here:
#   overlay_text_display, pie_chart_display
#     -> rviz_2d_overlay_plugins (https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins)
#   camera_info_display -> rviz_default_plugins/CameraInfo (https://github.com/ros2/rviz)
#   pose_array_display  -> rviz_default_plugins/PoseArray, which already provides the
#                          Arrow/Axes shapes this plugin added on ROS 1
#
# The plugins below are still waiting for an upstream dependency to be released for
# ROS 2. Port them once the listed package is available:
#   src/ambient_sound_display.{h,cpp}                     -> jsk_hark_msgs
#   src/ambient_sound_display_groovy.{h,cpp}              -> jsk_hark_msgs
#   src/ambient_sound_visual.{h,cpp}                      -> jsk_hark_msgs
#   src/footstep_display.{h,cpp}                          -> jsk_footstep_msgs
#   src/bounding_box_display.{h,cpp}                      -> jsk_recognition_msgs, jsk_topic_tools
#   src/bounding_box_array_display.{h,cpp}                -> jsk_recognition_msgs, jsk_topic_tools
#   src/bounding_box_display_common.h                     -> jsk_recognition_msgs, jsk_topic_tools
#   src/torus_array_display.{h,cpp}                       -> jsk_recognition_msgs
#   src/segment_array_display.{h,cpp}                     -> jsk_recognition_msgs
#   src/human_skeleton_array_display.{h,cpp}              -> jsk_recognition_msgs
#   src/polygon_array_display.{h,cpp}                     -> jsk_recognition_msgs, jsk_recognition_utils
#   src/simple_occupancy_grid_array_display.{h,cpp}       -> jsk_recognition_msgs, jsk_recognition_utils
#   src/rviz_util.h                                       -> only used by simple_occupancy_grid_array_display
#   src/people_position_measurement_array_display.{h,cpp} -> people_msgs
#   src/view_controller/tablet_view_controller.{h,cpp}    -> view_controller_msgs
#   src/yes_no_button_interface.{h,cpp}                   -> jsk_gui_msgs
#
# src/overlay_camera_display.{h,cpp} is blocked on rviz itself: it reads the rendered
# scene back with Ogre::RenderTarget::copyContentsToMemory(), but rviz_rendering does
# not expose the Ogre render target behind rviz_common::RenderPanel::getRenderWindow()
# (see rviz_rendering::RenderWindowOgreAdapter). Port it once rviz_rendering exposes it.
set(HEADER_FILES
  include/empty_service_call_interface.h
  include/overlay_utils.h
  include/publish_topic.h
  include/string_display.h
  include/close_all_tool.h
  include/open_all_tool.h
  include/twist_stamped_display.h
  include/plotter_2d_display.h
  include/linear_gauge_display.h
  include/diagnostics_display.h
  include/normal_display.h
  include/cancel_action.h
  include/facing_visualizer.h
  include/image_transport_hints_property.h
  include/overlay_image_display.h
  include/overlay_menu_display.h
  include/overlay_diagnostic_display.h
  include/overlay_picker_tool.h
  include/target_visualizer_display.h
  include/tf_trajectory_display.h
  include/pictogram_display.h
  include/pictogram_array_display.h
  include/screenshot_listener_tool.h
  include/quiet_interactive_marker_display.h
  include/video_capture_display.h
  include/rviz_scene_publisher.h
  include/record_action.h
  include/object_fit_operator.h
  include/robot_command_interface.h
  include/select_point_cloud_publish_action.h
  include/tablet_controller_panel.h
)
set(SOURCE_FILES
  src/empty_service_call_interface.cpp
  src/overlay_utils.cpp
  src/publish_topic.cpp
  src/string_display.cpp
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/twist_stamped_display.cpp
  src/plotter_2d_display.cpp
  src/linear_gauge_display.cpp
  src/diagnostics_display.cpp
  src/normal_display.cpp
  src/normal_visual.cpp
  src/cancel_action.cpp
  src/facing_visualizer.cpp
  src/image_transport_hints_property.cpp
  src/overlay_image_display.cpp
  src/overlay_menu_display.cpp
  src/overlay_diagnostic_display.cpp
  src/overlay_picker_tool.cpp
  src/target_visualizer_display.cpp
  src/tf_trajectory_display.cpp
  src/pictogram_display.cpp
  src/pictogram_array_display.cpp
  src/screenshot_listener_tool.cpp
  src/quiet_interactive_marker_display.cpp
  src/video_capture_display.cpp
  src/rviz_scene_publisher.cpp
  src/record_action.cpp
  src/object_fit_operator.cpp
  src/robot_command_interface.cpp
  src/select_point_cloud_publish_action.cpp
  src/tablet_controller_panel.cpp
)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

# rosidl_auto_generate_interfaces() reserves the ${PROJECT_NAME} target name for
# the interface generation, so the plugin library target is named differently and
# renamed back on output to keep libjsk_rviz_plugins.so as rviz_common_plugins.xml expects.
ament_auto_add_library(
  ${PROJECT_NAME}_plugins SHARED
  ${HEADER_FILES}
  ${SOURCE_FILES}
  ${MOC_FILES}
)
set_target_properties(${PROJECT_NAME}_plugins PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# cv_bridge only exports opencv_core/opencv_imgproc, so link opencv_videoio explicitly
# for the cv::VideoWriter used by video_capture_display.
find_package(OpenCV REQUIRED COMPONENTS videoio)
target_link_libraries(${PROJECT_NAME}_plugins opencv_videoio)

rosidl_auto_generate_interfaces(TARGETS ${PROJECT_NAME}_plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_auto_package()
# # support environment does not have visualization such as VTK or Rviz (i.e embedded environment)
# set(USE_VISUALIZATION ON CACHE BOOL "compile with rviz package")
# if(USE_VISUALIZATION)
#   set(RVIZ "rviz")
#   set(JSK_RECOGNITION_UTILS "jsk_recognition_utils") # jsk_recognition_utils depends on pcl/visualization (https://github.com/jsk-ros-pkg/jsk_recognition/blob/09cbce5f0ce7102331f6e48b9f9befc2b1fda869/jsk_recognition_utils/src/pcl_conversion_util.cpp#L37)
# else()
#   set(RVIZ "")
#   set(JSK_RECOGNITION_UTILS "")
#   message(WARNING "Compile without RVIZ/jsk_recognition_utils package")
# endif()

# # Load catkin and all dependencies required for this package
# # TODO: remove all from COMPONENTS that are not catkin packages.
# find_package(catkin REQUIRED COMPONENTS ${RVIZ} jsk_hark_msgs jsk_footstep_msgs ${JSK_RECOGNITION_UTILS}
#   people_msgs
#   message_generation std_msgs diagnostic_msgs cv_bridge
#   jsk_topic_tools
#   image_geometry
#   view_controller_msgs
#   geometry_msgs
#   dynamic_reconfigure
#   jsk_recognition_msgs
#   jsk_gui_msgs
#   message_filters)

# add_message_files(FILES
#   OverlayText.msg OverlayMenu.msg TransformableMarkerOperate.msg ObjectFitCommand.msg
#   Pictogram.msg PictogramArray.msg RecordCommand.msg StringStamped.msg)
# add_service_files(FILES
#   RequestMarkerOperate.srv EusCommand.srv
#   Screenshot.srv
#   )
# catkin_python_setup()
# generate_dynamic_reconfigure_options(
#   cfg/ClassificationResultVisualizer.cfg
#   cfg/ContactStateMarker.cfg
#   cfg/OverlayTextInterface.cfg)
# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# if(USE_VISUALIZATION)
#   set(${LIBRARIES_PROJECT_NAME} ${PROJECT_NAME})
# else()
#   set(${LIBRARIES_PROJECT_NAME} "")
# endif()
# catkin_package(
#     DEPENDS ${RVIZ}
#     CATKIN_DEPENDS jsk_hark_msgs jsk_footstep_msgs
#     ${JSK_RECOGNITION_UTILS} cv_bridge people_msgs image_geometry
#     INCLUDE_DIRS # TODO include
#     LIBRARIES ${LIBRARIES_PROJECT_NAME}
# )

# # TODO: fill in what other packages will need to use this package
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need

# if(USE_VISUALIZATION AND rviz_QT_VERSION VERSION_LESS "5")
#   find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#   include(${QT_USE_FILE})
# elseif(USE_VISUALIZATION)
#   find_package(Qt5Widgets REQUIRED)
# endif()
# add_definitions(-DQT_NO_KEYWORDS -g)

# #set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

# if(jsk_recognition_msgs_VERSION VERSION_LESS 1.2.15)
#   message(WARNING "jsk_recognition_msgs ${jsk_recognition_msgs_VERSION} < 1.2.15 does not support human_skeleton_array_display.cpp")
#   list(REMOVE_ITEM HEADER_FILES src/human_skeleton_array_display.h)
#   list(REMOVE_ITEM SOURCE_FILES src/human_skeleton_array_display.cpp)
#   list(REMOVE_ITEM SOURCE_FILES  ${PROJECT_BINARY_DIR}/src/moc_human_skeleton_array_display.cxx)
# endif()

# if(NOT USE_VISUALIZATION)
#   message(WARNING "polygon_array_display, simple_occupancy_grid_array_display requries jsk_recognition_utils")
#   list(REMOVE_ITEM HEADER_FILES src/polygon_array_display.h)
#   list(REMOVE_ITEM SOURCE_FILES src/polygon_array_display.cpp)
#   list(REMOVE_ITEM HEADER_FILES src/simple_occupancy_grid_array_display.h)
#   list(REMOVE_ITEM SOURCE_FILES src/simple_occupancy_grid_array_display.cpp)
# endif()

# if(rviz_FOUND)
# add_library(jsk_rviz_plugins ${SOURCE_FILES} ${UIC_FILES})
# if(rviz_QT_VERSION VERSION_LESS "5")
#   target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
# else()
#   target_link_libraries(jsk_rviz_plugins Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
# endif()
# add_dependencies(jsk_rviz_plugins
#   jsk_footstep_msgs_generate_messages_cpp
#   jsk_gui_msgs_generate_messages_cpp
#   jsk_hark_msgs_generate_messages_cpp
#   jsk_recognition_msgs_generate_messages_cpp
#   people_msgs_generate_messages_cpp
#   view_controller_msgs_generate_messages_cpp
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${catkin_EXPORTED_TARGETS})

# if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
#   set_target_properties(jsk_rviz_plugins PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
# endif()

# endif(rviz_FOUND)

# file(GLOB PYTHON_SCRIPTS scripts/*.py)
# catkin_install_python(
#   PROGRAMS ${PYTHON_SCRIPTS}
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY cfg config icons launch test
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   USE_SOURCE_PERMISSIONS
# )
# file(GLOB PYTHON_SAMPLES samples/*.py)
# catkin_install_python(
#   PROGRAMS ${PYTHON_SAMPLES}
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/samples
# )

# install(DIRECTORY src/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# if (CATKIN_ENABLE_TESTING)
#   # download and install sample data
#   add_custom_target(${PROJECT_NAME}_install_sample_data COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/test/install_sample_data.py)
#   add_dependencies(tests ${PROJECT_NAME}_install_sample_data)
#   # run rostest for melodic+
#   find_package(rostest REQUIRED)
#   if("$ENV{ROS_DISTRO}" STRGREATER "indigo") # support from kinetic
#     file(GLOB TEST_FILES test/*.test test/*.xml)
#     foreach(TEST_FILE ${TEST_FILES})
#       # if we failed to install sample data, skip testing face_detect and normal
#       if((NOT EXISTS "${PROJECT_SOURCE_DIR}/test/data/2017-06-20-12-00-00_people_images_in_lab.bag") AND
# 	  ((${TEST_FILE} STREQUAL "${PROJECT_SOURCE_DIR}/test/face_detector.xml") OR
# 	    (${TEST_FILE} STREQUAL "${PROJECT_SOURCE_DIR}/test/normal.xml")))
# 	message(STATUS "Skip test for ${TEST_FILE} due to missing bag file")
# 	continue()
#       endif()
#       message(STATUS "Run test for ${TEST_FILE}")
#       add_rostest(${TEST_FILE})
#     endforeach()
#   endif()
# endif()
