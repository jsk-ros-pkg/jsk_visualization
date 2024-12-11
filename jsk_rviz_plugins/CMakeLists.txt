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
