cmake_minimum_required(VERSION 2.8.3)
project(jsk_rqt_plugins)

find_package(catkin REQUIRED
  message_generation
)

catkin_python_setup()
catkin_package()

install(DIRECTORY bin/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch resource sample sample_scripts test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
  add_rostest(test/test_rqt_plugins.test)
endif()
