cmake_minimum_required(VERSION 3.5)
project(jsk_rviz_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(jsk_footstep_msgs REQUIRED)
find_package(jsk_gui_msgs REQUIRED)
find_package(jsk_hark_msgs REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)
find_package(jsk_rviz_plugin_msgs REQUIRED)
find_package(jsk_topic_tools REQUIRED)

# for Ogre include files
include_directories(${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(jsk_rviz_plugins SHARED
  ### Display
  src/overlay_utils.cpp
  src/plotter_2d_display.cpp
  src/pie_chart_display.cpp
  src/string_display.cpp
  src/overlay_diagnostic_display.cpp
  src/pictogram_display.cpp
  src/twist_stamped_display.cpp
  src/diagnostics_display.cpp
  src/facing_visualizer.cpp
  src/overlay_text_display.cpp
  src/torus_array_display.cpp
  src/overlay_image_display.cpp
  src/bounding_box_display.cpp
  src/bounding_box_array_display.cpp
  src/footstep_display.cpp
  src/human_skeleton_array_display.cpp
  src/camera_info_display.cpp
  src/normal_visual.cpp
  src/normal_display.cpp
  src/mesh_shape.cpp
  src/pictogram_array_display.cpp
  src/segment_array_display.cpp
  src/tf_trajectory_display.cpp
  src/target_visualizer_display.cpp
  src/polygon_array_display.cpp
  src/overlay_menu_display.cpp
  # src/overlay_camera_display.cpp
  # src/ambient_sound_display_groovy.cpp
  # src/ambient_sound_visual.cpp
  src/linear_gauge_display.cpp
  # src/quiet_interactive_marker_display.cpp
  # src/simple_occupancy_grid_array_display.cpp
  # src/people_position_measurement_array_display.cpp
  # src/camera_info_display.cpp
  # src/video_capture_display.cpp
  # src/rviz_scene_publisher.cpp
  # src/pose_array_display.cpp

  ### Panel
  src/object_fit_operator.cpp
  src/publish_topic.cpp
  src/cancel_action.cpp
  src/record_action.cpp
  src/yes_no_button_interface.cpp
  # src/tablet_controller_panel.cpp
  # src/select_point_cloud_publish_action.cpp
  # src/robot_command_interface.cpp
  # src/empty_service_call_interface.cpp

  ### Tool
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  src/screenshot_listener_tool.cpp
  # src/overlay_picker_tool.cpp

  ### ViewController
  # src/view_controller/tablet_view_controller.cpp
)

# Link ament packages
ament_target_dependencies(jsk_rviz_plugins 
  rclcpp 
  rviz_common
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  jsk_footstep_msgs
  jsk_gui_msgs
  jsk_hark_msgs
  jsk_recognition_msgs
  jsk_rviz_plugin_msgs
  jsk_topic_tools
  )

target_include_directories(jsk_rviz_plugins 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

# Link non ament packages
target_link_libraries(jsk_rviz_plugins Qt5::Widgets)

# prevent pluginlib from using boost
# target_compile_definitions(jsk_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# replaces catkin_package(LIBRARIES jsk_rviz_plugins) 
ament_export_libraries(jsk_rviz_plugins)

ament_export_include_directories(include)
ament_export_targets(export_jsk_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  cv_bridge
  image_transport
  geometry_msgs
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_ogre_vendor
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  diagnostic_msgs
  jsk_footstep_msgs
  jsk_gui_msgs
  jsk_hark_msgs
  jsk_recognition_msgs
  jsk_rviz_plugin_msgs
  jsk_topic_tools
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  jsk_rviz_plugins
  EXPORT export_jsk_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  icons
  DESTINATION share/jsk_rviz_plugins
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
