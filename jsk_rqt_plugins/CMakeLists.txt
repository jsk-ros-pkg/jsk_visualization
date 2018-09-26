cmake_minimum_required(VERSION 2.8.3)
project(jsk_rqt_plugins)

find_package(catkin REQUIRED
  message_generation
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS bin/rqt_3d_plot bin/rqt_histogram_plot bin/rqt_yn_btn
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
  add_rostest(test/test_rqt_plugins.test)
endif()
