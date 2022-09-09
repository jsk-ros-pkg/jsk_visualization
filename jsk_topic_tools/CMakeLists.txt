# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(jsk_topic_tools)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TopicInfo.msg

  srv/List.srv
  srv/Update.srv
  srv/ChangeTopic.srv
  srv/PassthroughDuration.srv

  DEPENDENCIES std_msgs builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

add_library(jsk_topic_tools_node SHARED
  src/color_utils.cpp)

ament_target_dependencies(jsk_topic_tools_node
  rclcpp
  std_msgs)

target_include_directories(jsk_topic_tools_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if (NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

ament_export_targets(export_jsk_topic_tools_node HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_libraries(jsk_topic_tools_node)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  jsk_topic_tools_node
  EXPORT export_jsk_topic_tools_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


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
