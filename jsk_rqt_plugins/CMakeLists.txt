cmake_minimum_required(VERSION 2.8.3)
project(jsk_rqt_plugins)

find_package(catkin REQUIRED
  message_generation
)

catkin_python_setup()
catkin_package()

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
file(GLOB PYTHON_SCRIPTS bin/*)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch resource sample sample_scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
file(GLOB PYTHON_SAMPLES sample_scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SAMPLES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sample_scripts
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.launch"
)
file(GLOB PYTHON_TEST test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_TEST}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "indigo")
    catkin_add_nosetests(test)
    add_rostest(test/test_rqt_plugins.test)
  endif()
endif()
