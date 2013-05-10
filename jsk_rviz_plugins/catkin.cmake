# these plugins should be uploaded to upstream repository
# https://github.com/ros-visualization/rviz/pull/634

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_rviz_plugins)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rviz)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories( ${wxWidgets_INCLUDE_DIRS} )

#set(SOURCE_FILES src/effort_display.cpp src/effort_visual.cpp src/wrench_display.cpp src/wrench_visual.cpp src/point_display.cpp src/point_visual.cpp src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)
qt4_wrap_cpp(MOC_FILES
  src/wrench_display_groovy.h
  src/point_display_groovy.h
  src/effort_display_groovy.h
)

set(SOURCE_FILES
  src/wrench_display_groovy.cpp
  src/wrench_visual.cpp
  src/point_display_groovy.cpp
  src/point_visual.cpp
  src/effort_display_groovy.cpp
  src/effort_visual.cpp
  ${MOC_FILES}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(jsk_rviz_plugins ${SOURCE_FILES})
target_link_libraries(jsk_rviz_plugins ${QT_LIBRARIES})

catkin_package(
    DEPENDS rviz
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS jsk_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
