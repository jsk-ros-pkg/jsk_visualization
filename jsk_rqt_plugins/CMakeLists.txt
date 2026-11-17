# $ENV{ROS_VERSION} is set by ros_environment, which is missing on
# pre-ros_environment distros (e.g. indigo) and on ROS 2 builds that do not
# depend on it (e.g. the buildfarm's binary jobs). Guess it then: ament sets
# AMENT_PREFIX_PATH (bloom's debian/rules passes it to cmake, setup.sh and
# colcon set the environment variable), catkin never does.
if("$ENV{ROS_VERSION}" STREQUAL "")
  if(DEFINED AMENT_PREFIX_PATH OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_rqt_plugins)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(jsk_ros1_ros2_compat REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  # The scripts keep `#!/usr/bin/env python` for ROS 1 (python2 on
  # indigo..melodic); ROS 2 base images ship no bare `python`, so the
  # installed ROS 2 copies get a python3 shebang.
  jsk_ros1_ros2_compat_install_python(PROGRAMS
    bin/rqt_3d_plot
    bin/rqt_drc_mini_maxwell
    bin/rqt_image_view2
    bin/rqt_service_buttons
    bin/rqt_status_light
    bin/rqt_string_label
    bin/rqt_tabbed_buttons
    bin/rqt_yn_btn
    sample_scripts/sample_3d_plot.py
    sample_scripts/sample_drc_mini_maxwell.py
    sample_scripts/sample_service_buttons.py
    sample_scripts/sample_service_radio_buttons.py
  )

  # The plugins of hist.py (HistogramPlot) and plot_2d.py (Plot2D) need
  # jsk_recognition_msgs, which is not released for ROS 2 yet, so their bin/
  # entry points (rqt_histogram_plot, rqt_2d_plot) are installed only when
  # it is found (e.g. built from source in the workspace, as the ROS 2 CI
  # does); their samples are ROS 1 only.
  find_package(jsk_recognition_msgs QUIET)
  if(jsk_recognition_msgs_FOUND)
    jsk_ros1_ros2_compat_install_python(PROGRAMS
      bin/rqt_2d_plot
      bin/rqt_histogram_plot
    )
  else()
    message(STATUS "rqt_histogram_plot and rqt_2d_plot skipped: jsk_recognition_msgs not found")
  endif()

  install(FILES plugin.xml
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}
  )
  # the ROS 2 versions of the sample launch files
  install(DIRECTORY sample
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch.py"
  )

  # The ROS 2 tests are in test/ros2/, where ROS 1's
  # catkin_add_nosetests(test) does not look.
  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    ament_add_pytest_test(test_util test/ros2/test_util.py)
    add_launch_test(test/ros2/test_rqt_plugins.py TIMEOUT 180)
  endif()

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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
endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
