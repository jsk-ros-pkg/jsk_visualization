# jsk_visualization for ROS2

# 環境
- Ubuntu 20.04(ROS2 Foxy)

# 実装
## jsk_rviz_plugins
### Display
| プラグイン | 実装着手 | ビルド | 動作確認 | 備考 |
| :----- | :----- | :----- | :----- | :----- |
| Plotter2D | ✅ | ✅ | ✅ |
| PieChart | ✅ | ✅ | ✅ |
| String | ✅ | ✅ | ✅ |
| OverlayText | ✅ | ✅ | ✅ |
| OverlayDiagnostic | ✅ | ✅ | ✅ |
| OverlayMenu | ✅ | ✅ | ✅ |
| BoundingBox | ✅ | ✅ | ✅ |
| OverlayImage | ✅ | ✅ | |
| TwistStamped | ✅ | ✅ | ✅ |
| BoundingBoxArray | ✅ | ✅ | |
| Footstep | ✅ | ✅ | |
| HumanSkeltonArray | ✅ | ✅ | |
| NormalDisplay | ✅ | ✅ | |
| Pictogram | ✅ | ✅ | |
| PictogramArray | ✅ | ✅ | |
| TorusArrayDisplay | ✅ | ✅ | |
| SegmentArray | ✅ | ✅ | |
| TFTrajectory | ✅ | ✅ | |
| TargetVisualizer | ✅ | ✅ | |
| Diagnostics | ✅ | ✅ | |
| LinearGauge | ✅ | ✅ | ✅ | |
| CameraInfo |  |  |  |
| PoseArray | | | | (default pluginに実装済み) |
| OverlayCamera | | | | (Panelを含む? )|
| PeoplePositionMeasuermentArray | | | | (msg未実装) |
| RvizScenePublisher  | | | | (#include <opencv2/opencv.hpp>) |
| QuietInteractiveMarker  | | | | (ROS2にinteractive_markerがない?) |
| AmbientSound  | | | | (?) |
| VideoCapture  | | | | (#include <opencv2/opencv.hpp>) |
| PolygonArray  | | | | (#include "jsk_recognition_utils/geo/polygon.hpp") |
| SimpleOccupancyGridArray  | | | | (#include "jsk_recognition_utils/geo/plane.h") |

### Panel
| プラグイン | 実装着手 | ビルド | 動作確認 | 備考 |
| :----- | :----- | :----- | :----- |  :----- | 
| ObjectFitOperatorAction | ✅ | ✅ | | |
| RecordAction | ✅ | ✅ | | |
| YesNoButton | ✅ | ✅ | | |
| PublishTopic | ✅ | ✅ | | |
| CancelAction | ✅ | ✅ | | |
| TabletControllerPanel | | | | |
| SelectPointCloudPublishAction | | | | |
| RobotCommandInterfaceAction | | | | |
| EmptyServiceCallInterfaceAction | | | | |
| TransformableMarkerOperatorAction | | | | |

### Tool
| プラグイン | 実装着手 | ビルド | 動作確認 | 備考 |
| :----- | :----- | :----- | :----- | :----- | 
| CloseAll | ✅ | ✅ | ✅ | |
| OpenAll | ✅ | ✅ | ✅ | |
| ScreenshotListener | ✅ | ✅ | | |
| OverlayPickerTool | | | | |

### ViewController
| プラグイン | 実装着手 | ビルド | 動作確認 | 備考 |
| :----- | :----- | :----- | :----- | :----- |
| TabletViewController | | | | |

## jsk_rqt_plugins
| プラグイン | 実装着手 | ビルド | 動作確認 | 備考 |
| :----- | :----- | :----- | :----- | :----- |
| rqt_2d_plot | | | | |
| rqt_3d_plot | | | | |
| rqt_drc_mini_maxwell | | | | |
| rqt_histogram_plot | | | | |
| rqt_image_view2 | | | | |
| rqt_service_buttons | | | | |
| rqt_service_radio_buttons | | | | |
| rqt_status_light | | | | |
| rqt_string_label | | | | |
| rqt_yn_btn | | | | |