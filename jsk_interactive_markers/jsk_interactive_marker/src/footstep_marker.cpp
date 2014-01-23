#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/footstep_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_footstep_msgs/PlanFootstepsGoal.h>
#include <jsk_footstep_msgs/PlanFootstepsResult.h>

FootstepMarker::FootstepMarker(): ac_("footstep_planner", true), plan_run_(false) {
  // read parameters
  marker_frame_id_ = "/map";
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  pnh.param("foot_size_x", foot_size_x_, 0.247);
  pnh.param("foot_size_y", foot_size_y_, 0.135);
  pnh.param("foot_size_z", foot_size_z_, 0.001);
  pnh.param("footstep_margin", footstep_margin_, 0.2);
  pnh.param("use_footstep_planner", use_footstep_planner_, true);
  footstep_pub_ = nh.advertise<jsk_footstep_msgs::FootstepArray>("footstep", 1);
  server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));

  marker_pose_.header.frame_id = marker_frame_id_;
  marker_pose_.header.stamp = ros::Time::now();
  marker_pose_.pose.orientation.w = 1.0;
  initializeInteractiveMarker();

  move_marker_sub_ = nh.subscribe("move_marker", 1, &FootstepMarker::moveMarkerCB, this);

  tf_listener_.reset(new tf::TransformListener);
  ROS_INFO("waiting server...");
  if (use_footstep_planner_) {
    ac_.waitForServer();
  }
}

void FootstepMarker::processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  marker_pose_.header = feedback->header;
  marker_pose_.pose = feedback->pose;
  marker_frame_id_ = feedback->header.frame_id;
  planIfPossible();
}

void FootstepMarker::planIfPossible() {
  // check the status of the ac_
  if (!use_footstep_planner_) {
    return;                     // do nothing
  }
  bool call_planner = !plan_run_;
  if (plan_run_) {
    actionlib::SimpleClientGoalState state = ac_.getState();
    if (state.isDone()) {
      jsk_footstep_msgs::PlanFootstepsResult::ConstPtr result = ac_.getResult();
      footstep_pub_.publish(result->result);
      ROS_INFO("planning is finished");
      call_planner = true;
    }
  }

  if (call_planner) {
    plan_run_ = true;
    jsk_footstep_msgs::PlanFootstepsGoal goal;
    jsk_footstep_msgs::FootstepArray goal_footstep;
    goal_footstep.header.frame_id = marker_frame_id_;
    goal_footstep.header.stamp = ros::Time(0.0);
    jsk_footstep_msgs::Footstep goal_left;
    goal_left.leg = jsk_footstep_msgs::Footstep::LEFT;
    goal_left.pose = getFootstepPose(true);
    jsk_footstep_msgs::Footstep goal_right;
    goal_right.pose = getFootstepPose(false);
    goal_right.leg = jsk_footstep_msgs::Footstep::RIGHT;
    goal_footstep.footsteps.push_back(goal_left);
    goal_footstep.footsteps.push_back(goal_right);
    goal.goal_footstep = goal_footstep;
    jsk_footstep_msgs::FootstepArray initial_footstep;
    initial_footstep.header.frame_id = marker_frame_id_;
    initial_footstep.header.stamp = ros::Time(0.0);
    // TODO: decide initial footstep by tf
    jsk_footstep_msgs::Footstep initial_left;
    initial_left.leg = jsk_footstep_msgs::Footstep::LEFT;
    initial_left.pose.position.y = footstep_margin_ / 2.0;
    initial_left.pose.orientation.w = 1.0;
    initial_footstep.footsteps.push_back(initial_left);
    jsk_footstep_msgs::Footstep initial_right;
    initial_right.leg = jsk_footstep_msgs::Footstep::RIGHT;
    initial_right.pose.position.y = footstep_margin_ / 2.0;
    initial_right.pose.orientation.w = 1.0;
    initial_footstep.footsteps.push_back(initial_right);
    goal.initial_footstep = initial_footstep;
    ac_.sendGoal(goal);
  }
}

geometry_msgs::Pose FootstepMarker::getFootstepPose(bool leftp) {
  tf::Vector3 offset(0, 0, 0);
  if (leftp) {
    offset[1] = footstep_margin_ / 2.0;
  }
  else {
    offset[1] = - footstep_margin_ / 2.0;
  }
  tf::Transform marker_origin;
  marker_origin.setOrigin(tf::Vector3(marker_pose_.pose.position.x,
                                      marker_pose_.pose.position.y,
                                      marker_pose_.pose.position.z));
  marker_origin.setRotation(tf::Quaternion(marker_pose_.pose.orientation.x,
                                           marker_pose_.pose.orientation.y,
                                           marker_pose_.pose.orientation.z,
                                           marker_pose_.pose.orientation.w));
  tf::Transform offset_trans;
  offset_trans.setRotation(tf::Quaternion(0, 0, 0, 1.0));
  offset_trans.setOrigin(offset);
  
  tf::Transform footstep_transform = marker_origin * offset_trans;
  geometry_msgs::Pose ret;
  ret.position.x = footstep_transform.getOrigin()[0];
  ret.position.y = footstep_transform.getOrigin()[1];
  ret.position.z = footstep_transform.getOrigin()[2];
  ret.orientation.x = footstep_transform.getRotation()[0];
  ret.orientation.y = footstep_transform.getRotation()[1];
  ret.orientation.z = footstep_transform.getRotation()[2];
  ret.orientation.w = footstep_transform.getRotation()[3];
  return ret;
}

void FootstepMarker::moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // move the marker
  geometry_msgs::PoseStamped transformed_pose;
  tf_listener_->transformPose(marker_frame_id_, *msg, transformed_pose);
  marker_pose_ = transformed_pose;
  // need to solve TF
  server_->setPose("footstep_marker", transformed_pose.pose);
  server_->applyChanges();
  planIfPossible();
}

void FootstepMarker::initializeInteractiveMarker() {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker_frame_id_;
  //int_marker.header.stamp = ros::Time(0);
  int_marker.name = "footstep_marker";
  
  visualization_msgs::Marker left_box_marker;
  left_box_marker.type = visualization_msgs::Marker::CUBE;
  left_box_marker.scale.x = foot_size_x_;
  left_box_marker.scale.y = foot_size_y_;
  left_box_marker.scale.z = foot_size_z_;
  left_box_marker.color.r = 0.0;
  left_box_marker.color.g = 1.0;
  left_box_marker.color.b = 0.0;
  left_box_marker.color.a = 1.0;
  left_box_marker.pose.position.y = footstep_margin_ / 2.0;

  visualization_msgs::InteractiveMarkerControl left_box_control;
  left_box_control.always_visible = true;
  left_box_control.markers.push_back( left_box_marker );

  int_marker.controls.push_back( left_box_control );

  visualization_msgs::Marker right_box_marker;
  right_box_marker.type = visualization_msgs::Marker::CUBE;
  right_box_marker.scale.x = foot_size_x_;
  right_box_marker.scale.y = foot_size_y_;
  right_box_marker.scale.z = foot_size_z_;
  right_box_marker.color.r = 1.0;
  right_box_marker.color.g = 0.0;
  right_box_marker.color.b = 0.0;
  right_box_marker.color.a = 1.0;
  right_box_marker.pose.position.y = - footstep_margin_ / 2.0;

  visualization_msgs::InteractiveMarkerControl right_box_control;
  right_box_control.always_visible = true;
  right_box_control.markers.push_back( right_box_marker );

  int_marker.controls.push_back( right_box_control );
  
  visualization_msgs::InteractiveMarkerControl control;
  
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  
  server_->insert(int_marker,
                  boost::bind(&FootstepMarker::processFeedbackCB, this, _1));
  server_->applyChanges();
}

FootstepMarker::~FootstepMarker() {
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "footstep_marker");
  FootstepMarker marker;
  ros::spin();
  return 0;
}
