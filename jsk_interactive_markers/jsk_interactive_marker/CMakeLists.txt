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
  project(jsk_interactive_marker)

  # Only what does not need the packages missing on ROS 2 is built here.
  # ROS 1 only, with the packages they need:
  # - footstep_marker, polygon_marker, bounding_box_marker,
  #   interactive_point_cloud, urdf_model_marker (jsk_recognition_msgs,
  #   jsk_recognition_utils, jsk_footstep_msgs)
  # - transformable_server_sample and the TransformableMarkerOperator rviz
  #   panel (jsk_rviz_plugins messages, jsk_recognition_msgs, rviz)
  # - interactive_marker_interface, urdf_control_marker (dynamic_tf_publisher,
  #   urdf/kdl/tf_conversions of interactive_marker_utils.cpp; built with
  #   jsk_recognition_utils on ROS 1)
  # - pointcloud_cropper (pcl_ros)
  # - srv/IndexRequest.srv (jsk_recognition_msgs/Int32Stamped)
  # - euslisp/ (roseus)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  # The ROS 1 messages as they are, except for what rosidl does not accept:
  # `duration` is builtin_interfaces/Duration, and field names are
  # lower_snake_case (MoveObject's graspPose is grasp_pose).
  set(_interfaces
    msg/JointTrajectoryPointWithType.msg
    msg/JointTrajectoryWithType.msg
    msg/MarkerDimensions.msg
    msg/MarkerMenu.msg
    msg/MarkerPose.msg
    msg/MoveModel.msg
    msg/MoveObject.msg
    msg/PoseStampedWithName.msg
    msg/SnapFootPrintInput.msg
    srv/GetJointState.srv
    srv/GetMarkerDimensions.srv
    srv/GetTransformableMarkerColor.srv
    srv/GetTransformableMarkerExistence.srv
    srv/GetTransformableMarkerFocus.srv
    srv/GetTransformableMarkerPose.srv
    srv/GetType.srv
    srv/MarkerSetPose.srv
    srv/RemoveParentMarker.srv
    srv/SetHeuristic.srv
    srv/SetMarkerDimensions.srv
    srv/SetParentMarker.srv
    srv/SetPose.srv
    srv/SetTransformableMarkerColor.srv
    srv/SetTransformableMarkerFocus.srv
    srv/SetTransformableMarkerPose.srv
    srv/SnapFootPrint.srv
  )
  set(_ros2_interfaces)
  foreach(_interface ${_interfaces})
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/${_interface} _content)
    string(REGEX REPLACE "(^|\n)duration " "\\1builtin_interfaces/Duration " _content "${_content}")
    string(REGEX REPLACE "(^|\n)(geometry_msgs/Pose) graspPose" "\\1\\2 grasp_pose" _content "${_content}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces/${_interface}.tmp "${_content}")
    # copied only when changed, not to regenerate the messages every time
    configure_file(${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces/${_interface}.tmp
      ${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces/${_interface} COPYONLY)
    list(APPEND _ros2_interfaces "${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces:${_interface}")
  endforeach()
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_ros2_interfaces}
    DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs visualization_msgs
  )
  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(jsk_interactive_marker)

if(("$ENV{ROS_DISTRO}" STREQUAL "kinetic") OR ("$ENV{ROS_DISTRO}" STREQUAL "indigo"))
  # catch special case ROS kinetic where c++11 is not the default yet
  add_compile_options(-std=c++11)
endif()

# support environment does not have visualization such as VTK or Rviz (i.e embedded environment)
set(USE_VISUALIZATION ON CACHE BOOL "compile with rviz package")
if(USE_VISUALIZATION)
  set(RVIZ "rviz")
  set(JSK_RECOGNITION_UTILS "jsk_recognition_utils")
else()
  set(RVIZ "")
  set(JSK_RECOGNITION_UTILS "")
  message(WARNING "Compile without RVIZ/jsk_recognition_utils package")
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
  jsk_topic_tools image_transport
  ${RVIZ} ${JSK_RECOGNITION_UTILS}
  )
message("rviz                  : VERSION: ${rviz_VERSION}, FOUND: ${rviz_FOUND}")
message("jsk_recognition_utils : VERSION: ${jsk_recognition_utils_VERSION}, FOUND ${jsk_recognition_utils_FOUND}")
message("jsk_recognition_msgs  : VERSION: ${jsk_recognition_msgs_VERSION}, FOUND: ${jsk_recognition_msgs_FOUND}")
message("jsk_footstep_msgs     : VERSION: ${jsk_footstep_msgs_VERSION}, FOUND: ${jsk_footstep_msgs_FOUND}")
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
if(rviz_FOUND AND (rviz_QT_VERSION VERSION_LESS "5"))
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
elseif(rviz_FOUND)
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

if((NOT rviz_FOUND) OR (jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0"))
  set(TRANSFORMABLE_MARKER_OPERATOR_SRC_FILES)
  message(WARNING "transformable_marker_operator.h requires jsk_recognition_msgs >= 1.2.0")
else(jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0")
  set(TRANSFORMABLE_MARKER_OPERATOR_SRC_FILES
     src/rviz_plugins/transformable_marker_operator.cpp)
if(rviz_FOUND AND (rviz_QT_VERSION VERSION_LESS "5"))
  qt4_wrap_cpp(MOC_FILES
    include/jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h
    )
elseif(rviz_FOUND)
  qt5_wrap_cpp(MOC_FILES
    include/jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h
    )
endif()
endif((NOT rviz_FOUND) OR (jsk_recognition_msgs_VERSION VERSION_LESS "1.2.0"))

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
if(rviz_FOUND AND (rviz_QT_VERSION VERSION_LESS "5"))
  target_link_libraries(jsk_interactive_marker ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})
elseif(rviz_FOUND)
  target_link_libraries(jsk_interactive_marker Qt5::Widgets ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES})
endif()
if(rviz_FOUND)
add_dependencies(jsk_interactive_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
endif()

set(INSTALL_TARGET)
if(jsk_recognition_utils_FOUND)
add_executable(interactive_marker_interface src/interactive_marker_interface.cpp src/interactive_marker_utils.cpp)
target_link_libraries(interactive_marker_interface ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(interactive_marker_interface ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
list(APPEND INSTALL_TARGET "interactive_marker_interface")

add_executable(camera_info_publisher src/camera_info_publisher.cpp src/interactive_marker_utils.cpp)
target_link_libraries(camera_info_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker yaml-cpp)
add_dependencies(camera_info_publisher ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
list(APPEND INSTALL_TARGET "camera_info_publisher")

add_executable(urdf_model_marker src/urdf_model_marker.cpp src/urdf_model_marker_main.cpp src/interactive_marker_utils.cpp)
target_link_libraries(urdf_model_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(urdf_model_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
list(APPEND INSTALL_TARGET "urdf_model_marker")

add_executable(urdf_control_marker src/urdf_control_marker.cpp src/interactive_marker_utils.cpp)
target_link_libraries(urdf_control_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(urdf_control_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
list(APPEND INSTALL_TARGET "urdf_control_marker")
endif()

add_executable(point_cloud_config_marker src/point_cloud_config_marker.cpp)
target_link_libraries(point_cloud_config_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(point_cloud_config_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(triangle_foot src/triangle_foot.cpp)
target_link_libraries(triangle_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(triangle_foot ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(door_foot src/door_foot.cpp)
target_link_libraries(door_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(door_foot ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

if(jsk_recognition_utils_FOUND)
  add_executable(footstep_marker src/footstep_marker.cpp)
  target_link_libraries(footstep_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
  add_dependencies(footstep_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
  list(APPEND INSTALL_TARGET "footstep_marker")
endif()

add_executable(marker_6dof src/marker_6dof.cpp)
target_link_libraries(marker_6dof ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(marker_6dof ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(world2yaml src/world2yaml)
target_link_libraries(world2yaml ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES}  ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(world2yaml ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(bounding_box_marker src/bounding_box_marker.cpp)
target_link_libraries(bounding_box_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
add_dependencies(bounding_box_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

if(jsk_recognition_utils_FOUND)
  add_executable(polygon_marker src/polygon_marker.cpp)
  target_link_libraries(polygon_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
  add_dependencies(polygon_marker ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
  list(APPEND INSTALL_TARGET "polygon_marker")

  add_executable(interactive_point_cloud src/interactive_point_cloud_node.cpp src/interactive_point_cloud.cpp src/parent_and_child_interactive_marker_server.cpp)
  target_link_libraries(interactive_point_cloud ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} jsk_interactive_marker)
  add_dependencies(interactive_point_cloud ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
  list(APPEND INSTALL_TARGET "interactive_point_cloud")
endif()

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

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config euslisp launch models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  jsk_interactive_marker
  point_cloud_config_marker
  triangle_foot
  door_foot
  marker_6dof
  world2yaml
  bounding_box_marker
  pointcloud_cropper
  transformable_server_sample
  ${INSTALL_TARGET}
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
endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
