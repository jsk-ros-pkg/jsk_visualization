// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define BOOST_PARAMETER_MAX_ARITY 7
#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/footstep_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_footstep_msgs/PlanFootstepsGoal.h>
#include <jsk_footstep_msgs/PlanFootstepsResult.h>
#include <std_srvs/Empty.h>
#include <jsk_recognition_msgs/CallSnapIt.h>
#include <Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <jsk_recognition_utils/geo_util.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <jsk_interactive_marker/SnapFootPrint.h>
#include <jsk_interactive_marker/SnapFootPrintInput.h>
#include <jsk_interactive_marker/SetHeuristic.h>
#include <jsk_topic_tools/log_utils.h>

FootstepMarker::FootstepMarker():
ac_("footstep_planner", true), ac_exec_("footstep_controller", true),
plan_run_(false), lleg_first_(true) {
  // read parameters
  tf_listener_.reset(new tf::TransformListener);
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  srv_ = std::make_shared <dynamic_reconfigure::Server<Config> > (pnh);
  typename dynamic_reconfigure::Server<Config>::CallbackType f =
    boost::bind (&FootstepMarker::configCallback, this, _1, _2);
  srv_->setCallback (f);
  pnh.param("foot_size_x", foot_size_x_, 0.247);
  pnh.param("foot_size_y", foot_size_y_, 0.135);
  pnh.param("foot_size_z", foot_size_z_, 0.01);
  pnh.param("lfoot_frame_id", lfoot_frame_id_, std::string("lfsensor"));
  pnh.param("rfoot_frame_id", rfoot_frame_id_, std::string("rfsensor"));
  pnh.param("show_6dof_control", show_6dof_control_, true);
  // pnh.param("use_projection_service", use_projection_service_, false);
  // pnh.param("use_projection_topic", use_projection_topic_, false);
  pnh.param("always_planning", always_planning_, true);
  // if (use_projection_topic_) {
    project_footprint_pub_ = pnh.advertise<jsk_interactive_marker::SnapFootPrintInput>("project_footprint", 1);
  // }
  // read lfoot_offset
  readPoseParam(pnh, "lfoot_offset", lleg_offset_);
  readPoseParam(pnh, "rfoot_offset", rleg_offset_);
  
  pnh.param("footstep_margin", footstep_margin_, 0.2);
  pnh.param("use_footstep_planner", use_footstep_planner_, true);

  pnh.param("use_footstep_controller", use_footstep_controller_, true);
  pnh.param("use_initial_footstep_tf", use_initial_footstep_tf_, true);
  pnh.param("wait_snapit_server", wait_snapit_server_, false);
  bool nowait = true;
  pnh.param("no_wait", nowait, true);
  pnh.param("frame_id", marker_frame_id_, std::string("/map"));
  footstep_pub_ = nh.advertise<jsk_footstep_msgs::FootstepArray>("footstep_from_marker", 1);
  snapit_client_ = nh.serviceClient<jsk_recognition_msgs::CallSnapIt>("snapit");
  snapped_pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("snapped_pose", 1);
  current_pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  estimate_occlusion_client_ = nh.serviceClient<std_srvs::Empty>("require_estimation");
  if (!nowait && wait_snapit_server_) {
    snapit_client_.waitForExistence();
  }
  
  if (pnh.getParam("initial_reference_frame", initial_reference_frame_)) {
    use_initial_reference_ = true;
    ROS_INFO_STREAM("initial_reference_frame: " << initial_reference_frame_);
  }
  else {
    use_initial_reference_ = false;
    ROS_INFO("initial_reference_frame is not specified ");
  }

  server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
  // menu_handler_.insert( "Snap Legs",
  //                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  // menu_handler_.insert( "Reset Legs",
  //                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Look Ground",
                        boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Execute the Plan",
                        boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Force to replan",
                        boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  // menu_handler_.insert( "Estimate occlusion",
  //                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Cancel Walk",
                        boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Toggle 6dof marker",
                        boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  // menu_handler_.insert( "Resume Footstep",
  //                     boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert("Straight Heuristic",
                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert("Stepcost Heuristic**",
                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert("LLeg First",
                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert("RLeg First",
                       boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  marker_pose_.header.frame_id = marker_frame_id_;
  marker_pose_.header.stamp = ros::Time::now();
  marker_pose_.pose.orientation.w = 1.0;

  resetLegPoses();

  // initialize lleg_initial_pose, rleg_initial_pose
  lleg_initial_pose_.position.y = footstep_margin_ / 2.0;
  lleg_initial_pose_.orientation.w = 1.0;
  rleg_initial_pose_.position.y = - footstep_margin_ / 2.0;
  rleg_initial_pose_.orientation.w = 1.0;
  
  if (use_initial_reference_) {
    while (ros::ok()) {
      try {
        if (!tf_listener_->waitForTransform(marker_frame_id_, initial_reference_frame_,
                                            ros::Time(0.0), ros::Duration(10.0))) {
          ROS_INFO_THROTTLE(1.0,
                            "waiting for transform %s => %s", marker_frame_id_.c_str(),
                            initial_reference_frame_.c_str());
          continue;
        }
        ROS_INFO("resolved transform %s => %s", marker_frame_id_.c_str(),
                 initial_reference_frame_.c_str());
        tf::StampedTransform transform;
        tf_listener_->lookupTransform(marker_frame_id_, initial_reference_frame_,
                                      ros::Time(0), transform);
        marker_pose_.pose.position.x = transform.getOrigin().x();
        marker_pose_.pose.position.y = transform.getOrigin().y();
        marker_pose_.pose.position.z = transform.getOrigin().z();
        marker_pose_.pose.orientation.x = transform.getRotation().x();
        marker_pose_.pose.orientation.y = transform.getRotation().y();
        marker_pose_.pose.orientation.z = transform.getRotation().z();
        marker_pose_.pose.orientation.w = transform.getRotation().w();
        break;
      }
      catch (tf2::TransformException& e) {
        ROS_ERROR("Failed to lookup transformation: %s", e.what());
      }
    }
  }

  initializeInteractiveMarker();

  if (use_footstep_planner_) {
    ROS_INFO("waiting planner server...");
    ac_.waitForServer();
    ROS_INFO("found planner server...");
  }
  if (use_footstep_controller_) {
    ROS_INFO("waiting controller server...");
    ac_exec_.waitForServer();
    ROS_INFO("found controller server...");
  }
  
  move_marker_sub_ = nh.subscribe("move_marker", 1, &FootstepMarker::moveMarkerCB, this);
  menu_command_sub_ = nh.subscribe("menu_command", 1, &FootstepMarker::menuCommandCB, this);
  exec_sub_ = pnh.subscribe("execute", 1, &FootstepMarker::executeCB, this);
  resume_sub_ = pnh.subscribe("resume", 1, &FootstepMarker::resumeCB, this);
  plan_if_possible_srv_ = pnh.advertiseService("force_to_replan", &FootstepMarker::forceToReplan, this);
  if (use_initial_footstep_tf_) {
    // waiting TF
    while (ros::ok()) {
      try {
      if (tf_listener_->waitForTransform(lfoot_frame_id_, marker_frame_id_,
                                         ros::Time(0.0), ros::Duration(10.0))
          && tf_listener_->waitForTransform(rfoot_frame_id_, marker_frame_id_,
                                            ros::Time(0.0), ros::Duration(10.0))) {
        break;
      }
      ROS_INFO("waiting for transform {%s, %s} => %s", lfoot_frame_id_.c_str(),
               rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
      }
      catch (tf2::TransformException& e) {
        ROS_ERROR("Failed to lookup transformation: %s", e.what());
      }
    }
    ROS_INFO("resolved transform {%s, %s} => %s", lfoot_frame_id_.c_str(),
             rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
  }
  // if (use_projection_topic_) {
    projection_sub_ = pnh.subscribe("projected_pose", 1,
                                    &FootstepMarker::projectionCallback, this);
  // }
}

void FootstepMarker::configCallback(Config& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(plane_mutex_);
  use_projection_topic_ = config.use_projection_topic;
  use_projection_service_ = config.use_projection_service;
  use_plane_snap_ = config.use_plane_snap;
  use_2d_ = config.use_2d;
}

// a function to read double value from XmlRpcValue.
// if the value is integer like 0 and 1, we need to
// cast it to int first, and after that, casting to double.
double getXMLDoubleValue(XmlRpc::XmlRpcValue val) {
  switch(val.getType()) {
  case XmlRpc::XmlRpcValue::TypeInt:
    return (double)((int)val);
  case XmlRpc::XmlRpcValue::TypeDouble:
    return (double)val;
  default:
    return 0;
  }
}

void FootstepMarker::readPoseParam(ros::NodeHandle& pnh, const std::string param,
                                   tf::Transform& offset) {
  XmlRpc::XmlRpcValue v;
  geometry_msgs::Pose pose;
  if (pnh.hasParam(param)) {
    pnh.param(param, v, v);
    // check if v is 7 length Array
    if (v.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        v.size() == 7) {
      // safe parameter access by getXMLDoubleValue
      pose.position.x = getXMLDoubleValue(v[0]);
      pose.position.y = getXMLDoubleValue(v[1]);
      pose.position.z = getXMLDoubleValue(v[2]);
      pose.orientation.x = getXMLDoubleValue(v[3]);
      pose.orientation.y = getXMLDoubleValue(v[4]);
      pose.orientation.z = getXMLDoubleValue(v[5]);
      pose.orientation.w = getXMLDoubleValue(v[6]);
      // converst the message as following: msg -> eigen -> tf
      //void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e);
      Eigen::Affine3d e;
      tf::poseMsgToEigen(pose, e); // msg -> eigen
      tf::transformEigenToTF(e, offset); // eigen -> tf
    }
    else {
      ROS_ERROR_STREAM(param << " is malformed, which should be 7 length array");
    }
  }
  else {
    ROS_WARN_STREAM("there is no parameter on " << param);
  }
}

void FootstepMarker::resetLegPoses() {
  lleg_pose_.orientation.x = 0.0;
  lleg_pose_.orientation.y = 0.0;
  lleg_pose_.orientation.z = 0.0;
  lleg_pose_.orientation.w = 1.0;
  lleg_pose_.position.x = 0.0;
  lleg_pose_.position.y = footstep_margin_ / 2.0;
  lleg_pose_.position.z = 0.0;
  
  rleg_pose_.orientation.x = 0.0;
  rleg_pose_.orientation.y = 0.0;
  rleg_pose_.orientation.z = 0.0;
  rleg_pose_.orientation.w = 1.0;
  rleg_pose_.position.x = 0.0;
  rleg_pose_.position.y = - footstep_margin_ / 2.0;
  rleg_pose_.position.z = 0.0;
}

geometry_msgs::Pose FootstepMarker::computeLegTransformation(uint8_t leg) {
  geometry_msgs::Pose new_pose;
  jsk_recognition_msgs::CallSnapIt srv;
  srv.request.request.header.stamp = ros::Time::now();
  srv.request.request.header.frame_id = marker_frame_id_;
  srv.request.request.target_plane.header.stamp = ros::Time::now();
  srv.request.request.target_plane.header.frame_id = marker_frame_id_;
  srv.request.request.target_plane.polygon = computePolygon(leg);
  if (snapit_client_.call(srv)) {
    Eigen::Affine3d A, T, B, B_prime;
    tf::poseMsgToEigen(srv.response.transformation, T);
    tf::poseMsgToEigen(marker_pose_.pose, A);
    if (leg == jsk_footstep_msgs::Footstep::LEFT) {
      tf::poseMsgToEigen(lleg_pose_, B);
    }
    else if (leg == jsk_footstep_msgs::Footstep::RIGHT) {
      tf::poseMsgToEigen(rleg_pose_, B);
    }
    B_prime = A.inverse() * T * A * B;
    tf::poseEigenToMsg(B_prime, new_pose);
  }
  else {
    // throw exception
    ROS_ERROR("failed to call snapit");
  }
  return new_pose;
}

void FootstepMarker::snapLegs() {
  geometry_msgs::Pose l_pose = computeLegTransformation(jsk_footstep_msgs::Footstep::LEFT);
  geometry_msgs::Pose r_pose = computeLegTransformation(jsk_footstep_msgs::Footstep::RIGHT);

  lleg_pose_ = l_pose;
  rleg_pose_ = r_pose;
  
}

geometry_msgs::Polygon FootstepMarker::computePolygon(uint8_t leg) {
  geometry_msgs::Polygon polygon;
  // tree
  // marker_frame_id_ ---[marker_pose_]---> [leg_pose_] --> points
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
  points.push_back(Eigen::Vector3d(foot_size_x_ / 2.0, foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(-foot_size_x_ / 2.0, foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(-foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0));

  Eigen::Affine3d marker_pose_eigen;
  Eigen::Affine3d leg_pose_eigen;
  tf::poseMsgToEigen(marker_pose_.pose, marker_pose_eigen);
  if (leg == jsk_footstep_msgs::Footstep::LEFT) {
    tf::poseMsgToEigen(lleg_pose_, leg_pose_eigen);
  }
  else if (leg == jsk_footstep_msgs::Footstep::RIGHT) {
    tf::poseMsgToEigen(rleg_pose_, leg_pose_eigen);
  }
  
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector3d point = points[i];
    Eigen::Vector3d new_point = marker_pose_eigen * leg_pose_eigen * point;
    geometry_msgs::Point32 point_msg;
    point_msg.x = new_point[0];
    point_msg.y = new_point[1];
    point_msg.z = new_point[2];
    polygon.points.push_back(point_msg);
  }
  
  return polygon;
}

void FootstepMarker::executeCB(const std_msgs::Empty::ConstPtr& msg) {
  executeFootstep();
}

void FootstepMarker::resumeCB(const std_msgs::Empty::ConstPtr& msg) {
  resumeFootstep();
}

void FootstepMarker::menuCommandCB(const std_msgs::UInt8::ConstPtr& msg) {
  processMenuFeedback(msg->data);
}

void FootstepMarker::updateInitialFootstep() {
  //ROS_INFO("updateInitialFootstep");
  try {
    if (!use_initial_footstep_tf_) {
      return;
    } 
    tf::StampedTransform lfoot_transform, rfoot_transform;
    tf_listener_->lookupTransform(marker_frame_id_, lfoot_frame_id_, ros::Time(0.0), lfoot_transform);
    tf_listener_->lookupTransform(marker_frame_id_, rfoot_frame_id_, ros::Time(0.0), rfoot_transform);

    // apply offset
    // convert like tf -> eigen -> msg
    Eigen::Affine3d le, re;
    tf::transformTFToEigen(lfoot_transform * lleg_offset_, le); // tf -> eigen
    tf::poseEigenToMsg(le, lleg_initial_pose_);  // eigen -> msg
    tf::transformTFToEigen(rfoot_transform * rleg_offset_, re); // tf -> eigen
    tf::poseEigenToMsg(re, rleg_initial_pose_);  // eigen -> msg
  
    // we need to move the marker
    initializeInteractiveMarker();
  }
  catch (tf2::TransformException& e) {
    ROS_ERROR("Failed to lookup transformation: %s", e.what());
  }
}

void FootstepMarker::lookGround()
{
  std_srvs::Empty empty;
  if (ros::service::call("/lookaround_ground", empty)) {
    ROS_INFO("Finished to look ground");
  }
  else {
    ROS_ERROR("Failed to look ground");
  }
}

bool FootstepMarker::forceToReplan(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
  planIfPossible();
  return true;
}

void FootstepMarker::processMenuFeedback(uint8_t menu_entry_id) {
  switch (menu_entry_id) {
  case 1: {                     // look ground
    lookGround();
    break;
  }
  case 2: {                     // execute
    executeCB(std_msgs::Empty::ConstPtr());
    break;
  }
  case 3: {                     // replan
    planIfPossible();
    break;
  }
  case 4: {                     // cancel walk
    cancelWalk();
    break;
  }
  case 5: {                     // toggle 6dof marker
    show_6dof_control_ = !show_6dof_control_;
    break;
  }
  case 6: {                     // toggle 6dof marker
    changePlannerHeuristic(":straight-heuristic");
    break;
  }
  case 7: {                     // toggle 6dof marker
    changePlannerHeuristic(":stepcost-heuristic**");
    break;
  }
  case 8: {                     // toggle 6dof marker
    lleg_first_ = true;
    break;
  }
  case 9: {                     // toggle 6dof marker
    lleg_first_ = false;
    break;
  }
    
  default: {
    break;
  }
  }
}

void FootstepMarker::changePlannerHeuristic(const std::string& heuristic)
{
  jsk_interactive_marker::SetHeuristic heuristic_req;
  heuristic_req.request.heuristic = heuristic;
  if (!ros::service::call("/footstep_planner/set_heuristic", heuristic_req)) {
    ROS_ERROR("failed to set heuristic");
  }
  else {
    ROS_INFO("Success to set heuristic: %s", heuristic.c_str());
  }
}

void FootstepMarker::cancelWalk()
{
  ROS_WARN("canceling walking");
  ac_exec_.cancelAllGoals();
  ROS_WARN("canceled walking");
}

void FootstepMarker::callEstimateOcclusion()
{
  std_srvs::Empty srv;
  estimate_occlusion_client_.call(srv);
}

bool FootstepMarker::projectMarkerToPlane()
{
  if (use_projection_service_) {
    jsk_interactive_marker::SnapFootPrint snap;
    snap.request.input_pose = marker_pose_;
    snap.request.lleg_pose.orientation.w = 1.0;
    snap.request.rleg_pose.orientation.w = 1.0;
    snap.request.lleg_pose.position.y = footstep_margin_ / 2.0;
    snap.request.rleg_pose.position.y = - footstep_margin_ / 2.0;
    if (ros::service::call("project_footprint", snap) && snap.response.success) {
      // Resolve tf
      geometry_msgs::PoseStamped resolved_pose;
      tf_listener_->transformPose(marker_pose_.header.frame_id,
                                  snap.response.snapped_pose,
                                  resolved_pose);
      // Check distance to project
      Eigen::Vector3d projected_point, marker_point;
      tf::pointMsgToEigen(marker_pose_.pose.position, marker_point);
      tf::pointMsgToEigen(resolved_pose.pose.position, projected_point);
      if ((projected_point - marker_point).norm() < 0.3) {
        server_->setPose("footstep_marker", resolved_pose.pose);
        snapped_pose_pub_.publish(resolved_pose);
        current_pose_pub_.publish(resolved_pose);
        server_->applyChanges();
        marker_pose_.pose = resolved_pose.pose;
        return true;
      }
      else {
        return false;
      }
    }
    else {
      ROS_WARN("Failed to snap footprint");
      return false;
    }
  }
  else if (use_projection_topic_) {
    jsk_interactive_marker::SnapFootPrintInput msg;
    msg.input_pose = marker_pose_;
    msg.lleg_pose.orientation.w = 1.0;
    msg.rleg_pose.orientation.w = 1.0;
    msg.lleg_pose.position.y = footstep_margin_ / 2.0;
    msg.rleg_pose.position.y = - footstep_margin_ / 2.0;
    project_footprint_pub_.publish(msg);
    return true;                // true...?
  }
}

void FootstepMarker::menuFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  processMenuFeedback(feedback->menu_entry_id);
}

void FootstepMarker::processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  boost::mutex::scoped_lock lock(plane_mutex_);

  marker_pose_.header = feedback->header;
  marker_pose_.pose = feedback->pose;
  marker_frame_id_ = feedback->header.frame_id;
  bool skip_plan = false;
  geometry_msgs::PoseStamped input_pose;
  current_pose_pub_.publish(marker_pose_);
  try {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      if (use_plane_snap_) {
        skip_plan = !projectMarkerToPlane();
      }
    }
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP && !skip_plan) {
      if (always_planning_) planIfPossible();
    }
  }
  catch (tf2::TransformException& e) {
    ROS_ERROR("Failed to lookup transformation: %s", e.what());
  }
}

void FootstepMarker::resumeFootstep() {
  if (!use_footstep_controller_) {
    return;
  }
  actionlib::SimpleClientGoalState state = ac_exec_.getState();
  if (!state.isDone()) {
    ROS_ERROR("still executing footstep");
    return;
  }
  jsk_footstep_msgs::ExecFootstepsGoal goal;
  goal.strategy = jsk_footstep_msgs::ExecFootstepsGoal::RESUME;
  ac_exec_.sendGoal(goal);
}

void FootstepMarker::projectionCallback(const geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::PoseStamped resolved_pose;
  tf_listener_->transformPose(marker_pose_.header.frame_id,
                              pose,
                              resolved_pose);
  // Check distance to project
  Eigen::Vector3d projected_point, marker_point;
  tf::pointMsgToEigen(marker_pose_.pose.position, marker_point);
  tf::pointMsgToEigen(resolved_pose.pose.position, projected_point);
  if ((projected_point - marker_point).norm() < 0.3) {
    marker_pose_.pose = resolved_pose.pose;
    snapped_pose_pub_.publish(resolved_pose);
    current_pose_pub_.publish(resolved_pose);
  }
}

void FootstepMarker::executeFootstep() {
  if (!use_footstep_controller_) {
    return;
  }
  actionlib::SimpleClientGoalState state = ac_exec_.getState();
  if (!state.isDone()) {
    ROS_ERROR("still executing footstep");
    return;
  }
  if (!plan_result_) {
    ROS_ERROR("no planner result is available");
    return;
  }
  
  
  jsk_footstep_msgs::ExecFootstepsGoal goal;
  goal.footstep = plan_result_->result;
  //goal.strategy = jsk_footstep_msgs::ExecFootstepsGoal::DEFAULT_STRATEGY;
  ROS_INFO("sending goal...");
  ac_exec_.sendGoal(goal);
  // ac_exec_.waitForResult();
  // ROS_INFO("done executing...");
}

void FootstepMarker::planIfPossible() {
  boost::mutex::scoped_lock lock(plan_run_mutex_);
  // check the status of the ac_
  if (!use_footstep_planner_) {
    return;                     // do nothing
  }
  bool call_planner = !plan_run_;
  if (call_planner) {
    plan_run_ = true;
    jsk_footstep_msgs::PlanFootstepsGoal goal;
    jsk_footstep_msgs::FootstepArray goal_footstep;
    goal_footstep.header.frame_id = marker_frame_id_;
    goal_footstep.header.stamp = ros::Time(0.0);
    jsk_footstep_msgs::Footstep goal_left;
    goal_left.leg = jsk_footstep_msgs::Footstep::LEFT;
    goal_left.pose = getFootstepPose(true);
    goal_left.dimensions.x = foot_size_x_;
    goal_left.dimensions.y = foot_size_y_;
    goal_left.dimensions.z = foot_size_z_;
    jsk_footstep_msgs::Footstep goal_right;
    goal_right.pose = getFootstepPose(false);
    goal_right.leg = jsk_footstep_msgs::Footstep::RIGHT;
    goal_right.dimensions.x = foot_size_x_;
    goal_right.dimensions.y = foot_size_y_;
    goal_right.dimensions.z = foot_size_z_;
    goal_footstep.footsteps.push_back(goal_left);
    goal_footstep.footsteps.push_back(goal_right);
    goal.goal_footstep = goal_footstep;
    jsk_footstep_msgs::FootstepArray initial_footstep;
    initial_footstep.header.frame_id = marker_frame_id_;
    initial_footstep.header.stamp = ros::Time(0.0);
    // TODO: decide initial footstep by tf
    jsk_footstep_msgs::Footstep initial_left;
    initial_left.leg = jsk_footstep_msgs::Footstep::LEFT;
    initial_left.pose = lleg_initial_pose_;
    initial_left.dimensions.x = foot_size_x_;
    initial_left.dimensions.y = foot_size_y_;
    initial_left.dimensions.z = foot_size_z_;
    
    jsk_footstep_msgs::Footstep initial_right;
    initial_right.leg = jsk_footstep_msgs::Footstep::RIGHT;
    initial_right.pose = rleg_initial_pose_;
    initial_right.dimensions.x = foot_size_x_;
    initial_right.dimensions.y = foot_size_y_;
    initial_right.dimensions.z = foot_size_z_;
    if (lleg_first_) {
      initial_footstep.footsteps.push_back(initial_left);
      initial_footstep.footsteps.push_back(initial_right);
    }
    else {
      initial_footstep.footsteps.push_back(initial_right);
      initial_footstep.footsteps.push_back(initial_left);
    }
    goal.initial_footstep = initial_footstep;
    ac_.sendGoal(goal, boost::bind(&FootstepMarker::planDoneCB, this, _1, _2));
  }
}

void FootstepMarker::planDoneCB(const actionlib::SimpleClientGoalState &state, 
                                 const PlanResult::ConstPtr &result)
{
  boost::mutex::scoped_lock lock(plan_run_mutex_);
  ROS_INFO("planDoneCB");
  plan_result_ = ac_.getResult();
  footstep_pub_.publish(plan_result_->result);
  ROS_INFO("planning is finished");
  plan_run_ = false;
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
  geometry_msgs::PoseStamped prev_pose = marker_pose_;
  marker_pose_ = transformed_pose;
  bool skip_plan = false;
  if (use_plane_snap_) {
    // do something magicalc
    skip_plan = !projectMarkerToPlane();
    if (skip_plan) {
      marker_pose_ = prev_pose;
    }
  }
  
  // need to solve TF
  server_->setPose("footstep_marker", transformed_pose.pose);
  server_->applyChanges();
  current_pose_pub_.publish(marker_pose_);
  if (!skip_plan) {
    planIfPossible();
  }
}

visualization_msgs::Marker FootstepMarker::makeFootstepMarker(geometry_msgs::Pose pose) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = foot_size_x_;
  marker.scale.y = foot_size_y_;
  marker.scale.z = foot_size_z_;
  marker.color.a = 1.0;
  marker.pose = pose;
  return marker;
}

void FootstepMarker::initializeInteractiveMarker() {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker_frame_id_;
  //int_marker.header.stamp = ros::Time(0);
  int_marker.name = "footstep_marker";
  int_marker.pose = marker_pose_.pose;
  visualization_msgs::Marker left_box_marker = makeFootstepMarker(lleg_pose_);
  left_box_marker.color.g = 1.0;

  visualization_msgs::InteractiveMarkerControl left_box_control;
  left_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  left_box_control.always_visible = true;
  left_box_control.markers.push_back( left_box_marker );

  int_marker.controls.push_back( left_box_control );

  visualization_msgs::Marker right_box_marker = makeFootstepMarker(rleg_pose_);
  right_box_marker.color.r = 1.0;

  visualization_msgs::InteractiveMarkerControl right_box_control;
  right_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  right_box_control.always_visible = true;
  right_box_control.markers.push_back( right_box_marker );

  int_marker.controls.push_back( right_box_control );
  if (show_6dof_control_) {
    if (use_2d_) {
      im_helpers::add3Dof2DControl(int_marker, false);
    }
    else {
      im_helpers::add6DofControl(int_marker, false);
    }
  }
  
  server_->insert(int_marker,
                  boost::bind(&FootstepMarker::processFeedbackCB, this, _1));

  // initial footsteps
  visualization_msgs::InteractiveMarker initial_lleg_int_marker;
  initial_lleg_int_marker.header.frame_id = marker_frame_id_;
  initial_lleg_int_marker.name = "left_initial_footstep_marker";
  initial_lleg_int_marker.pose.orientation.w = 1.0;
  visualization_msgs::Marker initial_left_marker = makeFootstepMarker(lleg_initial_pose_);
  initial_left_marker.color.g = 1.0;

  visualization_msgs::InteractiveMarkerControl initial_left_box_control;
  initial_left_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  initial_left_box_control.always_visible = true;
  initial_left_box_control.markers.push_back(initial_left_marker);

  initial_lleg_int_marker.controls.push_back( initial_left_box_control );
  server_->insert(initial_lleg_int_marker,
                  boost::bind(&FootstepMarker::processFeedbackCB, this, _1));

  visualization_msgs::InteractiveMarker initial_rleg_int_marker;
  initial_rleg_int_marker.header.frame_id = marker_frame_id_;
  initial_rleg_int_marker.name = "right_initial_footstep_marker";
  initial_rleg_int_marker.pose.orientation.w = 1.0;
  visualization_msgs::Marker initial_right_marker = makeFootstepMarker(rleg_initial_pose_);
  initial_right_marker.color.r = 1.0;

  visualization_msgs::InteractiveMarkerControl initial_right_box_control;
  initial_right_box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  initial_right_box_control.always_visible = true;
  initial_right_box_control.markers.push_back(initial_right_marker);

  initial_rleg_int_marker.controls.push_back( initial_right_box_control );
  server_->insert(initial_rleg_int_marker,
                  boost::bind(&FootstepMarker::processFeedbackCB, this, _1));
  
  menu_handler_.apply( *server_, "footstep_marker");
  
  server_->applyChanges();
}

FootstepMarker::~FootstepMarker() {
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "footstep_marker");
  FootstepMarker marker;
  ros::Rate r(10.0);
  while (ros::ok()) {
    ros::spinOnce();
    marker.updateInitialFootstep();
    r.sleep();
  }
  return 0;
}
