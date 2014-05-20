cmake_minimum_required(VERSION 2.8.3)
project(jsk_interactive_marker)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules jsk_pcl_ros jsk_footstep_msgs geometry_msgs visualization_msgs interactive_markers dynamic_tf_publisher tf_conversions eigen_conversions actionlib
  urdf)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES MarkerMenu.msg MarkerPose.msg MoveObject.msg
)
add_service_files(DIRECTORY srv
  FILES MarkerSetPose.srv SetPose.srv)

include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(interactive_marker_interface src/interactive_marker_interface.cpp src/interactive_marker_utils.cpp src/interactive_marker_helpers.cpp)
target_link_libraries(interactive_marker_interface ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(interactive_marker_interface ${PROJECT_NAME}_gencpp)

add_executable(urdf_model_marker src/urdf_model_marker.cpp src/urdf_model_marker_main.cpp src/interactive_marker_utils.cpp src/interactive_marker_helpers.cpp)
target_link_libraries(urdf_model_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(urdf_model_marker ${PROJECT_NAME}_gencpp)

add_executable(point_cloud_config_marker src/point_cloud_config_marker.cpp)
target_link_libraries(point_cloud_config_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(point_cloud_config_marker ${PROJECT_NAME}_gencpp)

add_executable(triangle_foot src/triangle_foot.cpp src/interactive_marker_helpers.cpp)
target_link_libraries(triangle_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(triangle_foot ${PROJECT_NAME}_gencpp)

add_executable(door_foot src/door_foot.cpp src/interactive_marker_helpers.cpp)
target_link_libraries(door_foot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(door_foot ${PROJECT_NAME}_gencpp)

add_executable(footstep_marker src/footstep_marker.cpp src/interactive_marker_helpers.cpp)
target_link_libraries(footstep_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(footstep_marker ${PROJECT_NAME}_gencpp)

add_executable(marker_6dof src/marker_6dof.cpp)
target_link_libraries(marker_6dof ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(marker_6dof ${PROJECT_NAME}_gencpp)


add_executable(world2yaml src/world2yaml)
target_link_libraries(world2yaml ${TinyXML_LIBRARIES})
add_dependencies(world2yaml ${PROJECT_NAME}_gencpp)


generate_messages(
  DEPENDENCIES geometry_msgs jsk_footstep_msgs visualization_msgs jsk_pcl_ros
)

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS  geometry_msgs jsk_footstep_msgs tf_conversions actionlib
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


install(DIRECTORY launch euslisp urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(TARGETS footstep_marker door_foot triangle_foot point_cloud_config_marker urdf_model_marker interactive_marker_interface
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})





