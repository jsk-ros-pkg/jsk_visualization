// -*- mode: c++ -*-

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <jsk_footstep_msgs/PlanFootstepsAction.h>

class FootstepMarker {
public:
  FootstepMarker();
  virtual ~FootstepMarker();
  
protected:
  void initializeInteractiveMarker();
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  geometry_msgs::Pose getFootstepPose(bool leftp);
  void planIfPossible();
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  double foot_size_x_;
  double foot_size_y_;
  double foot_size_z_;
  double footstep_margin_;
  std::string marker_frame_id_;
  geometry_msgs::PoseStamped marker_pose_;
  ros::Subscriber move_marker_sub_;
  ros::Publisher footstep_pub_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  actionlib::SimpleActionClient<jsk_footstep_msgs::PlanFootstepsAction> ac_;
  bool use_footstep_planner_;
  bool plan_run_;
};
