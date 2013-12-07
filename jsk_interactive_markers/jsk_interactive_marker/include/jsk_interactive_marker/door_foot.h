#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

class DoorFoot{
 public:
  visualization_msgs::Marker makeRWallMarker();
  visualization_msgs::Marker makeLWallMarker();
  visualization_msgs::Marker makeDoorMarker();
  visualization_msgs::Marker makeKnobMarker();
  visualization_msgs::Marker makeKnobMarker(int position);

  visualization_msgs::Marker makeRFootMarker();
  visualization_msgs::Marker makeLFootMarker();

  visualization_msgs::Marker makeFootMarker(geometry_msgs::Pose pose, bool right);
  visualization_msgs::InteractiveMarker makeInteractiveMarker();
  void moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void pushDoorCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void pullDoorCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateBoxInteractiveMarker();
  interactive_markers::MenuHandler makeMenuHandler();

  DoorFoot ();
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::string server_name;
  std::string marker_name;

  interactive_markers::MenuHandler menu_handler;
  double size_;
  bool push;
  bool use_color_knob;
  std::vector<geometry_msgs::PoseStamped> foot_list;
};

geometry_msgs::Pose getPose( XmlRpc::XmlRpcValue val);
double getXmlValue( XmlRpc::XmlRpcValue val );
