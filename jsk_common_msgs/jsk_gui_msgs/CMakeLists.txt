# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(jsk_gui_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Action.msg
  msg/AndroidSensor.msg
  msg/DeviceSensor.msg
  msg/Gravity.msg
  msg/MagneticField.msg
  msg/MultiTouch.msg
  msg/Tablet.msg
  msg/Touch.msg
  msg/TouchEvent.msg
  msg/VoiceMessage.msg

  srv/Query.srv
  srv/YesNo.srv

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

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
