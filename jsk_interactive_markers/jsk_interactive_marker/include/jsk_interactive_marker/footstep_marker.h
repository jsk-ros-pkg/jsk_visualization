// -*- mode: c++ -*-

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>

class FootstepMarker {
public:
  FootstepMarker();
  virtual ~FootstepMarker();
  
protected:
  void initializeInteractiveMarker();
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  double foot_size_x_;
  double foot_size_y_;
  double foot_size_z_;
  double footstep_margin_;
  std::string marker_frame_id_;
  geometry_msgs::PoseStamped marker_pose_;
  ros::Subscriber move_marker_sub_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
};
