cmake_minimum_required(VERSION 2.8.3)
project(jsk_transformable_interactive_marker)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  std_msgs
  geometry_msgs
  tf
  visualization_msgs
)

add_service_files(
  FILES
  GetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jsk_transformable_interactive_marker
  CATKIN_DEPENDS interactive_markers roscpp std_msgs tf visualization_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(jsk_transformable_interactive_marker
  src/transformable_object.cpp
  src/transformable_box.cpp
  src/transformable_cylinder.cpp
  src/transformable_torus.cpp
  src/transformable_interactive_server.cpp
)

target_link_libraries(jsk_transformable_interactive_marker
  ${catkin_LIBRARIES}
)

add_executable(transformable_server_sample src/transformable_server_sample.cpp)
target_link_libraries(transformable_server_sample
  ${catkin_LIBRARIES}
  jsk_transformable_interactive_marker
)
