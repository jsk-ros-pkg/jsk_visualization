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
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services

rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_executable(camera_info_publisher src/camera_info_publisher.cpp src/interactive_marker_utils.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(interactive_marker_interface src/interactive_marker_interface.cpp src/interactive_marker_utils.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(urdf_model_marker src/urdf_model_marker_main.cpp src/urdf_model_marker.cpp src/interactive_marker_utils.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(point_cloud_config_marker src/point_cloud_config_marker.cpp)

rosbuild_add_executable(triangle_foot src/triangle_foot.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(door_foot src/door_foot.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(world2yaml src/world2yaml)
rosbuild_add_executable(footstep_marker src/footstep_marker.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(marker_6dof src/marker_6dof.cpp)
rosbuild_add_executable(bounding_box_marker src/bounding_box_marker.cpp)
rosbuild_add_executable(pointcloud_cropper
  src/pointcloud_cropper.cpp
  src/interactive_marker_helpers.cpp)

rosbuild_add_executable(interactive_point_cloud src/interactive_point_cloud_node.cpp src/interactive_point_cloud.cpp src/interactive_marker_helpers.cpp)
rosbuild_add_executable(urdf_control_marker src/urdf_control_marker.cpp)

target_link_libraries(world2yaml ${TinyXML_LIBRARIES})
#rosbuild_add_executable(interactive_marker_interface_both_arm src/interactive_marker_interface_both_arm.cpp)
#rosbuild_add_executable(button_interactive_marker_interface src/button_interactive_marker_interface.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# copy *.rviz.default to *.rviz

file(GLOB _rviz_default_files "${PROJECT_SOURCE_DIR}/launch/*.rviz.default")
foreach(_rviz_default_file ${_rviz_default_files})
  string(REGEX REPLACE "\\.default$" "" _rviz_file "${_rviz_default_file}")
  if(EXISTS "${_rviz_file}")
    message("${_rviz_file} exists")
  else()
    execute_process(
      COMMAND cmake -E copy "${_rviz_default_file}" "${_rviz_file}")
    message("copy ${_rviz_default_file} to ${_rviz_file}")
  endif()
endforeach()
