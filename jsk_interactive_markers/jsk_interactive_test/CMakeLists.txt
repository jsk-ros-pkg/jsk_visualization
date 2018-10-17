# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_interactive_test)

find_package(catkin REQUIRED COMPONENTS visualization_msgs jsk_interactive jsk_interactive_marker)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS visualization_msgs jsk_interactive jsk_interactive_marker
    INCLUDE_DIRS
    LIBRARIES
)

install(DIRECTORY scripts/ spacenav/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
