#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>


class PointCloudConfigMarker{
 public:
  struct MarkerControlConfig{
    MarkerControlConfig(): marker_id(0), resolution_(0.05){
      
    }
    MarkerControlConfig(double s): marker_id(0), resolution_(0.05){
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
  visualization_msgs::Marker makeTextMarker(geometry_msgs::Vector3 size);
  visualization_msgs::InteractiveMarker makeBoxInteractiveMarker(MarkerControlConfig mconfig, std::string name);
  void moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  visualization_msgs::Marker makeMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void publishMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void cancelCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void clearCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void clearBoxCB( const std_msgs::Empty::ConstPtr &msg);
  void clearBox();

  void changeResolutionCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void changeBoxSize(geometry_msgs::Vector3 size);
  void changeBoxSizeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  

  void updateBoxInteractiveMarker();
  void changeBoxResolution(const std_msgs::Float32::ConstPtr &msg);
  void publishCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &pose);
  void updatePoseCB(const geometry_msgs::PoseStamped::ConstPtr &pose);
  void addBoxCB(const std_msgs::Empty::ConstPtr &msg);
  interactive_markers::MenuHandler makeMenuHandler();
  PointCloudConfigMarker ();
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  visualization_msgs::InteractiveMarkerFeedbackConstPtr latest_feedback_;
  ros::Publisher pub_;
  ros::Publisher current_pose_pub_;
  
  
  ros::Subscriber pose_update_sub_;
  ros::Subscriber add_box_sub_;
  ros::Subscriber clear_box_sub_;
  ros::Subscriber change_box_size_sub_;
  ros::Subscriber change_box_resolution_sub_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_20cm_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_10cm_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_5cm_menu_;

  interactive_markers::MenuHandler::EntryHandle box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_100_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_50_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_25_menu_;


  std::string server_name;
  std::string marker_name;
  std::string base_frame;
  double size_;
  MarkerControlConfig marker_control_config;
};
