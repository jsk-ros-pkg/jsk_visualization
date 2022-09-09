cmake_minimum_required(VERSION 3.5)
project(jsk_rviz_plugin_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(jsk_rviz_plugin_msgs
  msg/ObjectFitCommand.msg
  msg/OverlayMenu.msg
  msg/OverlayText.msg
  msg/Pictogram.msg
  msg/PictogramArray.msg
  msg/RecordCommand.msg
  msg/StringStamped.msg
  msg/TransformableMarkerOperate.msg

  srv/EusCommand.srv
  srv/RequestMarkerOperate.srv
  srv/Screenshot.srv

  DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

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
