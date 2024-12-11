^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_rqt_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.10 (2024-12-11)
-------------------

2.1.9 (2024-12-11)
------------------
* add more visualization options for BoundingBox and BoundingBoxArray (`#844 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/844>`_)
* [ROS-O] patches (`#891 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/891>`_)

  * drop ROS_DISTRO STRGREATER tests
    As the Debian packages use "Debian"
    https://salsa.debian.org/science-team/ros-ros-environment/-/blame/master/debian/rules?ref_type=heads#L6
    which is incompatible, does not trigger them
    c++14 is the default from melodic onward, so the additional statements
    there are not necessary

* face_detector is released, so remove .travis.rosinstall.noetic . add test to check with latest pyyaml from pip (`#884 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/884>`_)

  * [jsk_rqt_plugins] Fix yaml load in tabbed_button_general.py

* update perspective for noetic (`#873 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/873>`_)
* drop obsolete (and broken urlgrabber dependency) (`#867 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/867>`_)
* add test to check (ServiceButtonGeneralWidget_in_tab (`#880 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/880>`_)

  * Fix super function argument in tabbed_button_general
  * add test to check https://github.com/jsk-ros-pkg/jsk_visualization/pull/879

* use QFileDialog for file select (`#876 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/876>`_)
* fix plugin category (`#877 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/877>`_)
* return int in mouse event (`#874 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/874>`_)

* Contributors: Naoaki Kanazawa, Kei Okada, Shingo Kitagawa, Yoshiki Obinata, v4hn

2.1.8 (2022-01-11)
------------------
* [jsk_interactive_markers][jsk_rqt_plugins] fix yaml load (`#818 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/818>`_)
* fix with '2to3 -w -f zip .' (`#817 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/817>`_)

  * In python3, sys.maxint changed to sys.maxsize
  * fix "TypeError: 'dict_keys' object is not subscriptable" error
    we can fix this by '2to3 -w -f dict', but it will change too many lines, so we do not use this for now....
  * fix with '2to3 -w -f import .'
  * fix cStringIO.StringIO -> io.BytesIO, see https://stackoverflow.com/questions/11914472/stringio-in-python3 / https://stackoverflow.com/questions/50797043/string-argument-expected-got-bytes-in-buffer-write
  * [jsk_rqt_plugins][jsk_rviz_plugins] Fix has_key

* add tabbed_buttuns to rqt_plugin (`#813 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/813>`_)
* update .travis to 0.5.21 (`#814 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/814>`_)
* Updating the URLs of the jsk_rviz_plugins and jsk_rqt_plugins so the generated README points to working links (closes #805 ). (`#806 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/806>`_)
* Support single button setup in ServiceButtons plugin (`#798 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/798>`_)

  * `*column_indices` expects to be an array of integers and the length
  should be equal or longer than 2.
  * If column_indices is just an array of a single integer (length is
  1), do not apply sum with expanding a list argument.

* fix for using button_general.py without perspective file (`#796 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/796>`_)
* [jsk_rqt_plugins/button_general.py] reset button if SetBool fail (`#794 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/794>`_)
* sample_service_button.py support SetBool (`#792 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/792>`_)
* add sample_service_radio_buttons script (`#789 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/789>`_)
* add sample_service_buttons scriptf (`#790 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/790>`_)
* add sample_service_buttons.launch (`#791 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/791>`_)

* Contributors: Adi Vardi, Kei Okada, Naoki Hiraoka, Ryohei Ueda, Sam Pfeiffer, Shingo Kitagawa, Yohei Kakiuchi

2.1.7 (2020-10-17)
------------------
* add noetic test (`#774 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/774>`_)

  * run 2to3 -f except
  * support noetic, conver to package format 3

* Contributors: Kei Okada

2.1.6 (2020-04-13)
------------------
* fix test failure in jsk_rqt_plugins (`#766 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/766>`_)

  * disable jsk_rqt_plugins test in indigo
    this is because indigo does not support matplotlib
    liner_model.RANSACRegressor.

* [jsk_rqt_plugins] load rosparam correctly in button_general.py (`#746 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/746>`_)
* Replace image_pipeline with image_publisher (`#729 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/729>`_)

  * Replace image_pipeline with image_publisher
    * image_pipeline is a meta package and normal ros packages are not
    intended to depend on meta packages.

* Contributors: Ryohei Ueda, Shingo Kitagawa

2.1.5 (2019-02-18)
------------------

2.1.4 (2018-11-01)
------------------
* Fix install destination (`#717 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/717>`_)
  * Update comment about installation
  * Add comment for install destination
  * Install missing test/ as well
  * Use source permission when installing executables
  * Fix installation destination
* [jsk_rqt_plugins] Fix for working correctly on kinetic + qt5 (`#708 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/708>`_)
  * Add sample for rqt_image_view2
  * Update test for rqt_drc_mini_maxwell
  * Add sample for rqt_drc_mini_maxwell
  * Update test for rqt_3d_plot
  * Add sample for rqt_3d_plot
  * Fix for substituting to 'intercept'
  * Specify version of image_pipeline
  * Support matplotlib backend for qt5
  * Support qt>=5 in rqt plugins
  * Add dependency for test
  * Set retry=3 for test
  * Install all programs in jsk_rqt_plugins/bin/
  * Add test for program in jsk_rqt_plugins
  * use matplotlib.backends.backend_qt5agg in Qt5
  * add queue_size in publishers
  * check opencv version for kinetic
  * follow new sklearn ranac linear regression
  * use correct module for qt5
  * refactor plot_2d.py

* [jsk_rqt_plugins] fix hist.py to work on kinetic (`#707 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/707>`_)
* Revert "[jsk_rqt_plugins/HistogramPlot] modify to correctly run on kinetic (`#688 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/688>`_)" (`#704 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/704>`_)
  This reverts commit 79dcedf7fd761fc638f1cffc1cfaa46a7bb4f1a2.
* [jsk_rqt_plugins/HistogramPlot] modify to correctly run on kinetic (`#688 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/688>`_)
  * refactor codes in hist.py
  * use correct module of qt5 for kinetic
  * check opencv version for kinetic
  * add queue_size in publisher
* [jsk_rqt_plugins] Add dependency of sklearn (`#701 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/701>`_)
* Add <url> to package.xml to add link to README (`#681 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/681>`_)
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka, Shingo Kitagawa, Yuto Uchimi, Iory Yanokura

2.1.3 (2017-10-26)
------------------
* PR `#672 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/672>`_ needs jsk_gui_msgs 4.3.0 (https://github.com/jsk-ros-pkg/jsk_common_msgs/pull/18) (`#673 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/673>`_)
* Display message on rqt_yn_btn (`#672 <https://github.com/jsk-ros-pkg/jsk_visualization/issues/672>`_)
* Contributors: Kei Okada, Kentaro Wada

2.1.2 (2017-07-07)
------------------

2.1.1 (2017-02-15)
------------------

2.1.0 (2017-02-13)
------------------

2.0.1 (2016-12-15)
------------------

2.0.0 (2016-12-14)
------------------

1.0.34 (2016-09-29)
-------------------

1.0.33 (2016-09-13)
-------------------

1.0.32 (2016-07-20)
-------------------

1.0.31 (2016-05-19)
-------------------
* Warn about unsupported topic type
* Check class type of data instead of subscribed topic type in rqt_histgram_plot to support HistgramWithRangeArray
* Contributors: Kentaro Wada, Iori Kumagai

1.0.30 (2016-03-25)
-------------------

1.0.29 (2016-03-20)
-------------------

1.0.28 (2016-02-03)
-------------------
* Except NavigationToolbar ImportError from matplotlib
  this is caused with upgraded matplotlib.
  reported at: https://github.com/semiautomaticgit/SemiAutomaticClassificationPlugin/issues/2
  Modified:
  - jsk_rqt_plugins/src/jsk_rqt_plugins/hist.py
  - jsk_rqt_plugins/src/jsk_rqt_plugins/plot.py
  - jsk_rqt_plugins/src/jsk_rqt_plugins/plot_2d.py
* [jsk_rqt_plugins] Support min-max fields of PlotDataArray in
  rqt_2d_plot
* [jsk_rqt_plugins/plot_2d] Update to support new
  jsk_recognition_msgs/PlotData fields
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.27 (2015-12-08)
-------------------
* [jsk_rqt_plugins] Avoid already advertised error for rqt_yn_btn
* Contributors: Kentaro Wada

1.0.26 (2015-12-03)
-------------------
* [jsk_rqt_plugins] Advertise service after initialized
* Contributors: Kentaro Wada

1.0.25 (2015-10-10)
-------------------
* [jsk_rqt_plugins] Fit line to date by ransac
* [jsk_rqt_plugins] Move README to sphinx + readthedocs
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.24 (2015-09-08)
-------------------
* Fix get_slot_type_field_names for None msg
* [jsk_rqt_plugins] Add more plot options for rqt_2d_plot
* [jsk_rqt_plugins/rqt_2d_plot] Add sample
* [jsk_rqt_plugins/plot_2d] Show grid
* [jsk_rqt_plugins] Add script to plot data as scatter
* [jsk_rqt_plugins] Add util to get slot field
* [jsk_rqt_plugins] Depends on rqt_image_view
* [jsk_rqt_plugins/rqt_histogram_plot] Publish rendered image from
  rqt_histogram_plot.
  In order to overlay the image on rviz
* [jsk_rqt_plugins] Update README about rqt_histogram_plot
* [jsk_rqt_plugins/rqt_histogram_plot] Support legend
* [jsk_rqt_plugins/histogram_plot] Support
  jsk_recognition_msgs/HistogramWithRange message to specify
  x-axis values
* [jsk_rqt_plugins] Remove unused comments
* [jsk_rqt_plugins][rqt_string_label] Display more candidates which has string field
* [jsk_rqt_plugins] Catch error for unexpected message type
* [jsk_rqt_plugins] rqt_string_label supports nested string as well as std_msgs/String
* [jsk_rqt_plugins][button_general.py] Make combo_box shared
* Contributors: Kentaro Wada, Ryohei Ueda

1.0.23 (2015-07-15)
-------------------

1.0.22 (2015-06-24)
-------------------

1.0.21 (2015-06-11)
-------------------
* [jsk_rqt_plugins] Add sign for configuration button in the README image of rqt_service_button
* [jsk_rqt_plugins] catch import error and use roslib in rqt_yn_btn
* [jsk_rqt_plugins] Add README for rqt_service_button
* [jsk_rqt_plugins] Add rqt_yn_btn
* [jsk_rqt_plugins] generate button groups
* Contributors: Kentaro Wada, Masaki Murooka

1.0.20 (2015-05-04)
-------------------
* [jsk_rqt_plugins/iamge_view2_wrapper] Use thread to update image topic
  list isntead of QTimer not to hung up rqt_gui
* use button general class for push button and radio button. enable to set parameter name to set layout
* add radio button plugin
* display label and icon in button
* [jsk_rqt_plugins] Add python-urlgrabber dependency
* Contributors: Masaki Murooka, Ryohei Ueda

1.0.19 (2015-04-09)
-------------------
* [jsk_rqt_plugins] Use parens to import a lot of symbols from modules
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics and redraw
  in StatusLightWidget
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics in DRCEnvironmentViewerWidget
* [jsk_rqt_plugins] Fix periodic duration to call updateTopics in StringLabelWidget
* [jsk_rqt_plugins] Optimize image_view2_wrapper:
  1. Use signal to tell redraw event from subscription callback
  2. Fix periodic duration to call updateTopics
* [jsk_rqt_plugins] Add image_view2 to build depend
* [jsk_rqt_plugins] Support move event without clicking mouse in
  image_view2 rqt wrapper
* [jsk_rqt_plugins] Fix for handling right click in rqt_image_view2 wrapper
* [jsk_rqt_plugins] Lower frequency to update StatusLight
* [jsk_rqt_plugins] Do not redraw image if no needed in image_view2 wrapper
* [jsk_rqt_plugins] Change message type to uint8 from int32 in Status plugin
* [jsk_rqt_plugins] Fix typo
* [jsk_rqt_plugins] Add simple widget to visualize status
* [jsk_rqt_plugins] Add settings button and remove combo box from top view
* [jsk_rqt_plugins] Do not convert invalid ROS images to OpenCV images in rqt_image_view2
* [jsk_rqt_plugins] Update label setting of StringLabel plugin.
  * Use larger font. font size is 14
  * Align text to left
  * Enable word wraping
* [jsk_rqt_plugins] Use topic stored in perspective file in StringLabel plugin
* [jsk_rqt_plugins] Simple widget to display std_msgs/String
* [jsk_rqt_plugins] Add ComboBox and store setting in rqt_image_view2
* [jsk_rqt_plugins] Fix drawing of rqt_image_view2 when size is not usual
* [jsk_rqt_plugins] Read configuration of buttons from private parameter space
* [jsk_rqt_plugins] Add perspective to combine rqt_image_view2 and rqt_service_button
* [jsk_rqt_plugins] image_view2 rqt wrapper
* [jsk_rqt_plugins] Support image for button icons in rqt_service_button
* [jsk_rqt_plugins] Add simple rqt plugin to list buttons to call empty
  services and configurable by yaml file
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.18 (2015-01-30)
-------------------

1.0.17 (2015-01-29)
-------------------
* [jsk_rqt_plugins] Add simple viewer to visualize mini maxwell status.
* Contributors: Ryohei Ueda

1.0.16 (2015-01-04)
-------------------

1.0.15 (2014-12-13)
-------------------

1.0.14 (2014-12-09)
-------------------

1.0.13 (2014-10-10)
-------------------

1.0.12 (2014-09-23)
-------------------

1.0.11 (2014-09-22)
-------------------

1.0.10 (2014-09-13)
-------------------

1.0.9 (2014-09-07)
------------------

1.0.8 (2014-09-04)
------------------

1.0.7 (2014-08-06)
------------------

1.0.6 (2014-07-14)
------------------

1.0.5 (2014-06-29)
------------------
* fix jsk_rqt_plugins for groovy users
* only one topic should be taken into account. the argument of the topics
  cannot be an array
* add rqt plugin to visualize histogram
* Contributors: Ryohei Ueda

1.0.4 (2014-05-31)
------------------
* jsk_rqt_plugins: add mk/rosbuild to build_depend
* jsk_rqt_plugins) install missing .ui file
* Contributors: Kei Okada, Isaac IY Saito

1.0.3 (2014-05-22)
------------------

1.0.2 (2014-05-21)
------------------

1.0.1 (2014-05-20)
------------------

1.0.0 (2014-05-17)
------------------

0.0.3 (2014-05-15)
------------------

0.0.2 (2014-05-15)
------------------
* add --no-legend option to disable legend
* support polygon mode. if you want to plot in line mode, please add -L option
* implement 3d plotter
* add jsk_rqt_plugins directory
* Contributors: Ryohei Ueda
