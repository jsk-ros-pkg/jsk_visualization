# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_interactive)

find_package(catkin REQUIRED COMPONENTS jsk_interactive_marker)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS jsk_interactive_marker
    INCLUDE_DIRS
    LIBRARIES
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
