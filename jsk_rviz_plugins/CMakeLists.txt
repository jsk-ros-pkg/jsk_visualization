cmake_minimum_required(VERSION 3.28.3)
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

# rosidl_auto_generate_interfaces() reserves the ${PROJECT_NAME} target name for
# the interface generation, so the plugin library target is named differently and
# renamed back on output to keep libjsk_rviz_plugins.so as rviz_common_plugins.xml expects.
ament_auto_add_library(
  ${PROJECT_NAME}_plugins SHARED
  ${HEADER_FILES}
  ${SOURCE_FILES}
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
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_overlay.py TIMEOUT 120)
  add_launch_test(test/test_linear_gauge.py TIMEOUT 120)
  add_launch_test(test/test_pictogram.py TIMEOUT 120)
  add_launch_test(test/test_service_call_panel.py TIMEOUT 120)
  add_launch_test(test/test_robot_command_interface.py TIMEOUT 120)
endif()

install(
  FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config icons launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    samples/overlay_menu_sample.py
    samples/overlay_sample.py
    samples/piechart_sample.py
    samples/pictogram_sample.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_auto_package()
