#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

class TriangleFoot{
 public:
  visualization_msgs::Marker makeTriangleMarker();
  visualization_msgs::Marker makeRFootMarker();
  visualization_msgs::Marker makeLFootMarker();
  visualization_msgs::Marker makeFootMarker(geometry_msgs::Pose pose);
  visualization_msgs::InteractiveMarker makeInteractiveMarker();
  void moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void reverseTriangleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateBoxInteractiveMarker();
  interactive_markers::MenuHandler makeMenuHandler();

  TriangleFoot ();
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::string server_name;
  std::string marker_name;

  interactive_markers::MenuHandler menu_handler;
  double size_;
  bool reverse;
};
