cmake_minimum_required(VERSION 3.5)
project(jsk_recognition_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoundingBoxArray.msg
  msg/BoundingBoxMovement.msg
  msg/BoundingBox.msg
  msg/Circle2DArray.msg
  msg/Circle2D.msg
  msg/ClassificationResult.msg
  # msg/ClusterPointIndices.msg
  msg/ColorHistogramArray.msg
  msg/ColorHistogram.msg
  msg/DepthCalibrationParameter.msg
  msg/DepthErrorResult.msg
  msg/HeightmapConfig.msg
  msg/Histogram.msg
  #msg/HistogramWithRangeBin.msg
  #msg/HistogramWithRange.msg
  #msg/ICPResult.msg
  msg/HumanSkeleton.msg
  msg/HumanSkeletonArray.msg
  msg/ImageDifferenceValue.msg
  msg/Int32Stamped.msg
  msg/LineArray.msg
  msg/Line.msg
  # msg/ModelCoefficientsArray.msg
  # msg/ParallelEdgeArray.msg
  # msg/ParallelEdge.msg
  msg/PlotData.msg
  msg/PointsArray.msg
  msg/PolygonArray.msg
  msg/PosedCameraInfo.msg
  msg/RectArray.msg
  msg/Rect.msg
  msg/RotatedRect.msg
  msg/RotatedRectStamped.msg
  msg/Segment.msg
  msg/SegmentArray.msg
  msg/SimpleHandle.msg
  msg/SimpleOccupancyGridArray.msg
  msg/SimpleOccupancyGrid.msg
  msg/SlicedPointCloud.msg
  msg/SnapItRequest.msg
  msg/SparseImage.msg
  msg/SparseOccupancyGridArray.msg
  msg/SparseOccupancyGridCell.msg
  msg/SparseOccupancyGridColumn.msg
  msg/SparseOccupancyGrid.msg
  msg/TimeRange.msg
  msg/TorusArray.msg
  msg/Torus.msg
  msg/WeightedPoseArray.msg

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
