# does not depends on people_msgs on groovy
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  list(APPEND _files ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
  list(APPEND _files ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml)
  foreach(_file ${_files})
    message("remove people_msgs dependency from ${_file}")
    execute_process(COMMAND sed -i "/people_msgs/d" ${_file})
  endforeach()
else($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(people_position_measurement_array_header
    "src/people_position_measurement_array_display.h")
  set(people_position_measurement_array_source
    "src/people_position_measurement_array_display.cpp")
  set(people_msgs "people_msgs")
endif($ENV{ROS_DISTRO} STREQUAL "groovy")

if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} src )

qt4_wrap_cpp(MOC_FILES
  src/publish_topic.h
  src/cancel_action.h
  src/select_point_cloud_publish_action.h
  src/footstep_display.h
  src/polygon_array_display.h
  src/normal_display.h
  src/overlay_text_display.h
  src/overlay_menu_display.h
  src/overlay_image_display.h
  src/overlay_camera_display.h
  src/plotter_2d_display.h
  src/pie_chart_display.h
  src/diagnostics_display.h
  src/ambient_sound_display_groovy.h
  src/quiet_interactive_marker_display.h
  src/bounding_box_array_display.h
  src/overlay_diagnostic_display.h
  src/target_visualizer_display.h
#  ${people_position_measurement_array_header}
  src/sparse_occupancy_grid_array_display.h
  src/camera_info_display.h
  src/close_all_tool.h
  src/open_all_tool.h
)

set(SOURCE_FILES
  src/footstep_display.cpp
  src/cancel_action.cpp
  src/publish_topic.cpp
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
  src/diagnostics_display.cpp
  src/ambient_sound_display_groovy.cpp
  src/ambient_sound_visual.cpp
  src/quiet_interactive_marker_display.cpp
  src/bounding_box_array_display.cpp
  src/overlay_diagnostic_display.cpp
  src/sparse_occupancy_grid_array_display.cpp
  src/target_visualizer_display.cpp
#  ${people_position_measurement_array_source}
  src/overlay_utils.cpp
  src/facing_visualizer.cpp
  src/camera_info_display.cpp
  src/close_all_tool.cpp
  src/open_all_tool.cpp
  ${MOC_FILES})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_library(jsk_rviz_plugins ${SOURCE_FILES})
target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES})
