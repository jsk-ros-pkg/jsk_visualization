cmake_minimum_required(VERSION 2.8.3)
project(jsk_interactive_marker)

if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
  add_compile_options(-std=c++14)
else()
  add_compile_options(-std=c++11)
endif()

# Use ccache if installed to make it fast to generate object files
if (CMAKE_VERSION VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(PCL_MSGS pcl)
else()
  set(PCL_MSGS pcl_msgs) ## hydro and later
endif()


find_package(catkin REQUIRED COMPONENTS
  message_generation
  cmake_modules jsk_footstep_msgs geometry_msgs visualization_msgs
  dynamic_reconfigure jsk_rviz_plugins
  interactive_markers dynamic_tf_publisher tf_conversions eigen_conversions
  actionlib roscpp roslib urdf
  pcl_conversions
  jsk_recognition_msgs
  ${PCL_MSGS}
  jsk_topic_tools
  rviz
  )
message("jsk_recognition_msgs : ${jsk_recognition_msgs_VERSION}")
message("jsk_footstep_msgs    : ${jsk_footstep_msgs_VERSION}")
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  DIRECTORY msg
  FILES JointTrajectoryPointWithType.msg JointTrajectoryWithType.msg MarkerDimensions.msg MarkerMenu.msg MarkerPose.msg MoveModel.msg MoveObject.msg SnapFootPrintInput.msg PoseStampedWithName.msg
)
add_service_files(DIRECTORY srv
  FILES MarkerSetPose.srv SetPose.srv GetJointState.srv GetType.srv SetMarkerDimensions.srv GetMarkerDimensions.srv GetTransformableMarkerPose.srv SetTransformableMarkerPose.srv GetTransformableMarkerColor.srv SetTransformableMarkerColor.srv GetTransformableMarkerFocus.srv SetTransformableMarkerFocus.srv GetTransformableMarkerExistence.srv IndexRequest.srv SetParentMarker.srv RemoveParentMarker.srv
  SnapFootPrint.srv
  SetHeuristic.srv)

generate_dynamic_reconfigure_options(
  cfg/FootstepMarker.cfg
  cfg/InteractivePointCloud.cfg
  cfg/PointCloudCropper.cfg
  cfg/CameraInfoPublisher.cfg
  cfg/InteractiveSetting.cfg
  )

add_definitions("-g")

include_directories(include ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# include Qt
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

if(jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0")
  set(TRANSFORMABLE_MARKER_OPERATOR_SRC_FILES)
  message(WARNING "transformable_marker_operator.h requires jsk_recognition_msgs >= 1.2.0")
else(jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0")
  set(TRANSFORMABLE_MARKER_OPERATOR_SRC_FILES
     src/rviz_plugins/transformable_marker_operator.cpp)
if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES
    include/jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h
    )
else()
  qt5_wrap_cpp(MOC_FILES
    include/jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h
    )
endif()
endif(jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0")

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(jsk_interactive_marker
  src/interactive_marker_helpers.cpp
  ${TRANSFORMABLE_MARKER_OPERATOR_SRC_FILES}
  ${MOC_FILES}
  ${UIC_FILES}
)
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(jsk_interactive_marker ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(jsk_interactive_marker Qt5::Widgets ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})
endif()
add_dependencies(jsk_interactive_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(interactive_marker_interface src/interactive_marker_interface.cpp src/interactive_marker_utils.cpp)
target_link_libraries(interactive_marker_interface ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(interactive_marker_interface ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(camera_info_publisher src/camera_info_publisher.cpp src/interactive_marker_utils.cpp)
target_link_libraries(camera_info_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(camera_info_publisher ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(urdf_model_marker src/urdf_model_marker.cpp src/urdf_model_marker_main.cpp src/interactive_marker_utils.cpp)
target_link_libraries(urdf_model_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(urdf_model_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(urdf_control_marker src/urdf_control_marker.cpp src/interactive_marker_utils.cpp)
target_link_libraries(urdf_control_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(urdf_control_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(point_cloud_config_marker src/point_cloud_config_marker.cpp)
target_link_libraries(point_cloud_config_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(point_cloud_config_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(triangle_foot src/triangle_foot.cpp)
target_link_libraries(triangle_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(triangle_foot ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(door_foot src/door_foot.cpp)
target_link_libraries(door_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(door_foot ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(footstep_marker src/footstep_marker.cpp)
target_link_libraries(footstep_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(footstep_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(marker_6dof src/marker_6dof.cpp)
target_link_libraries(marker_6dof ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(marker_6dof ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(world2yaml src/world2yaml)
target_link_libraries(world2yaml ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES}  ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(world2yaml ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(bounding_box_marker src/bounding_box_marker.cpp)
target_link_libraries(bounding_box_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(bounding_box_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(polygon_marker src/polygon_marker.cpp)
target_link_libraries(polygon_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(polygon_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(interactive_point_cloud src/interactive_point_cloud_node.cpp src/interactive_point_cloud.cpp src/parent_and_child_interactive_marker_server.cpp)
target_link_libraries(interactive_point_cloud ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(interactive_point_cloud ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(pointcloud_cropper src/pointcloud_cropper.cpp
 )
target_link_libraries(pointcloud_cropper ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(pointcloud_cropper ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(transformable_server_sample
  src/transformable_object.cpp
  src/yaml_menu_handler.cpp
  src/parent_and_child_interactive_marker_server.cpp
  src/transformable_interactive_server.cpp
  src/transformable_server_sample.cpp
)

target_link_libraries(transformable_server_sample ${catkin_LIBRARIES} jsk_interactive_marker yaml-cpp)
add_dependencies(transformable_server_sample ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

generate_messages(
  DEPENDENCIES geometry_msgs jsk_footstep_msgs visualization_msgs jsk_recognition_msgs
)

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS  geometry_msgs jsk_footstep_msgs tf_conversions actionlib jsk_rviz_plugins jsk_recognition_msgs message_runtime
    INCLUDE_DIRS include # TODO include
    LIBRARIES jsk_interactive_marker # TODO
)

find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
if(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
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
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS jsk_interactive_marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config euslisp launch models scripts urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  jsk_interactive_marker
  interactive_marker_interface
  camera_info_publisher
  urdf_model_marker
  urdf_control_marker
  point_cloud_config_marker
  triangle_foot
  door_foot
  footstep_marker
  marker_6dof
  world2yaml
  bounding_box_marker
  polygon_marker
  interactive_point_cloud
  pointcloud_cropper
  transformable_server_sample
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# copy *.rviz.default to *.rviz
macro(copy_rviz_files _rviz_files)
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
endmacro(copy_rviz_files _rviz_files)

file(GLOB _rviz_default_files "${PROJECT_SOURCE_DIR}/launch/*.rviz.default")
copy_rviz_files(_rviz_default_files)
file(GLOB _rviz_default_files "${PROJECT_SOURCE_DIR}/config/*.rviz.default")
copy_rviz_files(_rviz_default_files)
