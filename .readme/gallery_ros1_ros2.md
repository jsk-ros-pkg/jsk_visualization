ROS 1 / ROS 2 gallery
---------------------

The same source tree builds for ROS 1 (rviz, rqt) and ROS 2 (rviz2, rqt).
These are the sample launch files (`roslaunch <pkg> <sample>.launch` on ROS 1,
`ros2 launch <pkg> <sample>.launch.py` on ROS 2) and the rqt scripts (`rosrun` / `ros2 run`)
captured on ROS One (rviz) and on ROS 2 Jazzy (rviz2); in the GIFs of the interactive markers
and of image_view2 the markers and the image are dragged with the mouse.

### jsk_rviz_plugins

| plugin / sample | ROS 1 | ROS 2 |
|:--|:--:|:--:|
| OverlayText, String, Plotter2D, PieChart,<br>OverlayImage, OverlayMenu, OverlayDiagnostic<br>`overlay_sample`<br>(on ROS 2, OverlayText and PieChart<br>are left to rviz_2d_overlay_plugins) | <img src=".readme/gallery/rviz_overlay_ros1.gif" width="400"> | <img src=".readme/gallery/rviz_overlay_ros2.gif" width="400"> |
| Pictogram, PictogramArray<br>`pictogram_sample` | <img src=".readme/gallery/rviz_pictogram_ros1.png" width="400"> | <img src=".readme/gallery/rviz_pictogram_ros2.png" width="400"> |
| LinearGauge<br>`linear_gauge_sample` | <img src=".readme/gallery/rviz_linear_gauge_ros1.png" width="400"> | <img src=".readme/gallery/rviz_linear_gauge_ros2.png" width="400"> |
| PieChart<br>`piechart_sample` | <img src=".readme/gallery/rviz_piechart_ros1.png" width="400"> | ROS 1 only<br>(rviz_2d_overlay_plugins on ROS 2) |
| RobotCommandInterfaceAction (panel)<br>`robot_command_interface_sample` | <img src=".readme/gallery/rviz_robot_command_interface_ros1.png" width="400"> | <img src=".readme/gallery/rviz_robot_command_interface_ros2.png" width="400"> |
| EmptyServiceCallInterfaceAction (panel)<br>`service_call_panel` | <img src=".readme/gallery/rviz_service_call_panel_ros1.png" width="400"> | <img src=".readme/gallery/rviz_service_call_panel_ros2.png" width="400"> |
| BoundingBoxArray<br>`boundingbox_sample` | <img src=".readme/gallery/rviz_bounding_box_array_ros1.png" width="400"> | ROS 1 only<br>(needs jsk_recognition_msgs,<br>not released for ROS 2) |
| PolygonArray<br>`polygon_array_sample` | <img src=".readme/gallery/rviz_polygon_array_ros1.png" width="400"> | ROS 1 only<br>(needs jsk_recognition_msgs,<br>not released for ROS 2) |
| SegmentArray<br>`segment_array_sample` | <img src=".readme/gallery/rviz_segment_array_ros1.png" width="400"> | ROS 1 only<br>(needs jsk_recognition_msgs,<br>not released for ROS 2) |

### jsk_rqt_plugins

| plugin / sample | ROS 1 | ROS 2 |
|:--|:--:|:--:|
| StringLabel<br>`rqt_string_label` | <img src=".readme/gallery/rqt_string_label_ros1.png" width="400"> | <img src=".readme/gallery/rqt_string_label_ros2.png" width="400"> |
| StatusLight<br>`rqt_status_light` | <img src=".readme/gallery/rqt_status_light_ros1.png" width="400"> | <img src=".readme/gallery/rqt_status_light_ros2.png" width="400"> |
| YesNoButton<br>`rqt_yn_btn` | <img src=".readme/gallery/rqt_yn_btn_ros1.png" width="400"> | <img src=".readme/gallery/rqt_yn_btn_ros2.png" width="400"> |
| ImageView2Plugin (with image_view2)<br>`sample_image_view2` | <img src=".readme/gallery/rqt_image_view2_ros1.gif" width="400"> | <img src=".readme/gallery/rqt_image_view2_ros2.gif" width="400"> |
| Plot3D<br>`sample_3dplot` | <img src=".readme/gallery/rqt_3d_plot_ros1.png" width="400"> | <img src=".readme/gallery/rqt_3d_plot_ros2.png" width="400"> |
| ServiceButtons<br>`sample_service_buttons` | <img src=".readme/gallery/rqt_service_buttons_ros1.png" width="400"> | <img src=".readme/gallery/rqt_service_buttons_ros2.png" width="400"> |
| ServiceRadioButtons<br>`sample_service_radio_buttons` | <img src=".readme/gallery/rqt_service_radio_buttons_ros1.png" width="400"> | <img src=".readme/gallery/rqt_service_radio_buttons_ros2.png" width="400"> |
| ServiceTabbedButtons<br>`sample_tabbed_buttons` | <img src=".readme/gallery/rqt_tabbed_buttons_ros1.png" width="400"> | <img src=".readme/gallery/rqt_tabbed_buttons_ros2.png" width="400"> |
| DRCEnvironmentViewer<br>`sample_drc_mini_maxwell` | <img src=".readme/gallery/rqt_drc_mini_maxwell_ros1.png" width="400"> | <img src=".readme/gallery/rqt_drc_mini_maxwell_ros2.png" width="400"> |
| HistogramPlot<br>`sample_histogram` | <img src=".readme/gallery/rqt_histogram_plot_ros1.png" width="400"> | ROS 1 only<br>(needs jsk_recognition_msgs,<br>not released for ROS 2) |
| Plot2D<br>`sample_2dplot` | <img src=".readme/gallery/rqt_2d_plot_ros1.png" width="400"> | ROS 1 only<br>(needs jsk_recognition_msgs,<br>not released for ROS 2) |

### jsk_interactive_marker

| plugin / sample | ROS 1 | ROS 2 |
|:--|:--:|:--:|
| marker_6dof<br>`marker_6dof_sample` | <img src=".readme/gallery/im_marker_6dof_ros1.gif" width="400"> | <img src=".readme/gallery/im_marker_6dof_ros2.gif" width="400"> |
| transformable_server_sample<br>`transformable_server_sample` | <img src=".readme/gallery/im_transformable_server_ros1.gif" width="400"> | <img src=".readme/gallery/im_transformable_server_ros2.gif" width="400"> |
