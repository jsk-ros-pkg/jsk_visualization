#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/footstep_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_footstep_msgs/PlanFootstepsGoal.h>
#include <jsk_footstep_msgs/PlanFootstepsResult.h>
#include <jsk_pcl_ros/CallSnapIt.h>
#include <Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

FootstepMarker::FootstepMarker(): ac_("footstep_planner", true), plan_run_(false) {
  // read parameters
  
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  pnh.param("foot_size_x", foot_size_x_, 0.247);
  pnh.param("foot_size_y", foot_size_y_, 0.135);
  pnh.param("foot_size_z", foot_size_z_, 0.01);
  pnh.param("lfoot_frame_id", lfoot_frame_id_, std::string("lfsensor"));
  pnh.param("rfoot_frame_id", rfoot_frame_id_, std::string("rfsensor"));
  // read lfoot_offset
  readPoseParam(pnh, "lfoot_offset", lleg_offset_);
  readPoseParam(pnh, "rfoot_offset", rleg_offset_);
  
  pnh.param("footstep_margin", footstep_margin_, 0.2);
  pnh.param("use_footstep_planner", use_footstep_planner_, true);
  pnh.param("use_initial_footstep_tf", use_initial_footstep_tf_, true);
  pnh.param("wait_snapit_server", wait_snapit_server_, false);
  pnh.param("frame_id", marker_frame_id_, std::string("/map"));
  footstep_pub_ = nh.advertise<jsk_footstep_msgs::FootstepArray>("footstep", 1);
  snapit_client_ = nh.serviceClient<jsk_pcl_ros::CallSnapIt>("snapit");
  if (wait_snapit_server_) {
    snapit_client_.waitForExistence();
  }
  
    
  server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
  menu_handler_.insert( "Snap Legs", boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  menu_handler_.insert( "Reset Legs", boost::bind(&FootstepMarker::menuFeedbackCB, this, _1));
  
  marker_pose_.header.frame_id = marker_frame_id_;
  marker_pose_.header.stamp = ros::Time::now();
  marker_pose_.pose.orientation.w = 1.0;

  resetLegPoses();

  // initialize lleg_initial_pose, rleg_initial_pose
  lleg_initial_pose_.position.y = footstep_margin_ / 2.0;
  lleg_initial_pose_.orientation.w = 1.0;
  rleg_initial_pose_.position.y = - footstep_margin_ / 2.0;
  rleg_initial_pose_.orientation.w = 1.0;
  
  initializeInteractiveMarker();
  
  move_marker_sub_ = nh.subscribe("move_marker", 1, &FootstepMarker::moveMarkerCB, this);
  menu_command_sub_ = nh.subscribe("menu_command", 1, &FootstepMarker::menuCommandCB, this);
  tf_listener_.reset(new tf::TransformListener);

  if (use_initial_footstep_tf_) {
    // waiting TF
    while (ros::ok()) {
      if (tf_listener_->waitForTransform(lfoot_frame_id_, marker_frame_id_,
                                         ros::Time(0.0), ros::Duration(10.0))
          && tf_listener_->waitForTransform(rfoot_frame_id_, marker_frame_id_,
                                            ros::Time(0.0), ros::Duration(10.0))) {
        break;
      }
      ROS_INFO("waiting for transform {%s, %s} => %s", lfoot_frame_id_.c_str(),
               rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
    }
  }

  if (use_footstep_planner_) {
    ROS_INFO("waiting server...");
    ac_.waitForServer();
  }
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
  jsk_pcl_ros::CallSnapIt srv;
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

void FootstepMarker::menuCommandCB(const std_msgs::UInt8::ConstPtr& msg) {
  processMenuFeedback(msg->data);
}

void FootstepMarker::updateInitialFootstep() {
  ROS_INFO("updateInitialFootstep");
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

void FootstepMarker::processMenuFeedback(uint8_t menu_entry_id) {
  switch (menu_entry_id) {
  case 1: {                     // snapit
    snapLegs();
    initializeInteractiveMarker();
    break;
  }
  case 2: {
    resetLegPoses();
    initializeInteractiveMarker();
    break;
  }
  default: {
    break;
  }
  }
}

void FootstepMarker::menuFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  processMenuFeedback(feedback->menu_entry_id);
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
    initial_left.pose = lleg_initial_pose_;
    initial_footstep.footsteps.push_back(initial_left);
    jsk_footstep_msgs::Footstep initial_right;
    initial_right.leg = jsk_footstep_msgs::Footstep::RIGHT;
    initial_right.pose = rleg_initial_pose_;
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

  im_helpers::add6DofControl(int_marker, false);
    
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
