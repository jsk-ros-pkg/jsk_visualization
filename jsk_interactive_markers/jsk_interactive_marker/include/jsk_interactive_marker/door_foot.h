#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

class DoorFoot{
 public:
  void procAnimation();
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
  void showStandLocationCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void showNextStepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void showPreviousStepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateBoxInteractiveMarker();
  interactive_markers::MenuHandler makeMenuHandler();

  DoorFoot ();
 private:
  bool footstep_show_initial_p_;
  int footstep_index_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::string server_name;
  std::string marker_name;

  interactive_markers::MenuHandler menu_handler;
  double size_;
  bool push;
  bool use_color_knob;
  std::vector<geometry_msgs::PoseStamped> foot_list;
  geometry_msgs::Pose door_pose;
};

geometry_msgs::Pose getPose( XmlRpc::XmlRpcValue val);
double getXmlValue( XmlRpc::XmlRpcValue val );
