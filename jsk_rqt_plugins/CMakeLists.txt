cmake_minimum_required(VERSION 3.28.3)
project(jsk_rqt_plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The following rqt plugins are still waiting for an upstream dependency to be
# released for ROS 2. They are left as ROS 1 code under src/jsk_rqt_plugins/ and
# are not registered in plugin.xml. Port them once the listed package is available:
#   src/jsk_rqt_plugins/hist.py (HistogramPlot)
#     -> jsk_recognition_msgs (HistogramWithRange)
#   src/jsk_rqt_plugins/plot_2d.py (Plot2D)
#     -> jsk_recognition_msgs (PlotData, PlotDataArray)
#   src/jsk_rqt_plugins/image_view2_wrapper.py (ImageView2Plugin)
#     -> image_view2 (MouseEvent); it is in jsk_common/jsk_ros_patch/image_view2
#        but still a catkin package
#   src/jsk_rqt_plugins/yes_no_button.py (YesNoButton)
#     -> jsk_gui_msgs (YesNo); same blocker as
#        jsk_rviz_plugins/src/yes_no_button_interface.cpp
#
# ServiceRadioButtons has no standalone bin/ entry point, exactly as on ROS 1: it
# is launched through resource/rqt_service_radio_buttons.perspective, or as the
# radio tab of ServiceTabbedButtons.
#
# The corresponding bin/ entry points (rqt_histogram_plot, rqt_2d_plot,
# rqt_image_view2, rqt_yn_btn), sample_scripts/{sample_hist_pub,sample_2d_plot}.py
# and sample/{sample_histogram,sample_2dplot,sample_image_view2}.launch are kept
# as ROS 1 files for the same reason and are not installed.

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(
  PROGRAMS
    bin/rqt_3d_plot
    bin/rqt_drc_mini_maxwell
    bin/rqt_service_buttons
    bin/rqt_status_light
    bin/rqt_string_label
    bin/rqt_tabbed_buttons
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    sample_scripts/sample_3d_plot.py
    sample_scripts/sample_drc_mini_maxwell.py
    sample_scripts/sample_service_buttons.py
    sample_scripts/sample_service_radio_buttons.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch resource sample
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # ROS 2 counterpart of catkin_add_nosetests(test)
  ament_add_pytest_test(test_util test/test_util.py)

  # ROS 2 counterpart of add_rostest(test/test_rqt_plugins.test): brings up the
  # migrated rqt plugins and checks that none of them dies.
  # rqt needs a display, so the test skips itself unless DISPLAY is set;
  # use `xvfb-run -a colcon test ...` to run it headless.
  add_launch_test(test/test_rqt_plugins.py TIMEOUT 180)
endif()

ament_package()
