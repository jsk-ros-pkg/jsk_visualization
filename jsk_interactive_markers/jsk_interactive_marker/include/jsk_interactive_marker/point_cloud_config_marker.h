#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

class PointCloudConfigMarker{
 public:
  struct MarkerControlConfig{
    MarkerControlConfig(){
    }
    MarkerControlConfig(double s){
      size.x = s;
      size.y = s;
      size.z = s;
    }
    geometry_msgs::Pose pose;
    geometry_msgs::Vector3 size;
    int marker_id;
    double resolution_;
  };

  visualization_msgs::Marker makeBoxMarker(geometry_msgs::Vector3 size);
  visualization_msgs::InteractiveMarker makeBoxInteractiveMarker(MarkerControlConfig mconfig, std::string name);
  void moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  visualization_msgs::Marker makeMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void publishMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void cancelCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void clearCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void changeResolutionCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void changeBoxSize(geometry_msgs::Vector3 size);
  void changeBoxSizeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void updateBoxInteractiveMarker();

  interactive_markers::MenuHandler makeMenuHandler();
  PointCloudConfigMarker ();
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  ros::Publisher pub_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_10cm_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_5cm_menu_;

  interactive_markers::MenuHandler::EntryHandle box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_100_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_50_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_25_menu_;


  std::string server_name;
  std::string marker_name;

  double size_;
  MarkerControlConfig marker_control_config;
};
