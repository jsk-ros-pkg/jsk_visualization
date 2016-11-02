// -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Kei Okada nor the names of its
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

#include <jsk_interactive_marker/dynamic_transform_publisher.h>

namespace jsk_interactive_marker {

void DynamicTfPublisher::onInit() {
  nh_ = ros::NodeHandle("~");
  nh_.param<std::string>("parent_frame_id", parent_frame_id_, "");
  nh_.param<std::string>("frame_id", frame_id_, "");
  nh_.param<double>("publish_period", publish_period_, 500);
  publish_period_ = publish_period_ / 1000.0;
  node_name_ = ros::this_node::getName();

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(nh_));
  ReconfigureServer::CallbackType f =
    boost::bind(&DynamicTfPublisher::reconfiguration_callback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Set up interactive marker server
  interactive_server_.reset(
    new interactive_markers::InteractiveMarkerServer(node_name_, "", false));

  visualization_msgs::InteractiveMarker int_marker = createMarker();
  interactive_server_->insert(
    int_marker, boost::bind(&DynamicTfPublisher::processFeedback, this, _1));
  interactive_server_->applyChanges();

  publish_timer_ =
    nh_.createTimer(ros::Duration(publish_period_),
                    boost::bind(&DynamicTfPublisher::timerCallback, this, _1));
}

void DynamicTfPublisher::timerCallback(const ros::TimerEvent) {
  if (need_config_update_) {
    config_.x = x_;
    config_.y = y_;
    config_.z = z_;
    config_.roll = roll_;
    config_.pitch = pitch_;
    config_.yaw = yaw_;
    config_.frame_id = frame_id_;
    config_.parent_frame_id = parent_frame_id_;
    reconfigure_server_->updateConfig(config_);
    need_config_update_ = false;
  }

  transformStamped.transform.translation.x = x_;
  transformStamped.transform.translation.y = y_;
  transformStamped.transform.translation.z = z_;

  transformStamped.transform.rotation.x = qx_;
  transformStamped.transform.rotation.y = qy_;
  transformStamped.transform.rotation.z = qz_;
  transformStamped.transform.rotation.w = qw_;

  if (frame_id_ == "" || parent_frame_id_ == "") {
      return;
  }

  transformStamped.child_frame_id = frame_id_;
  transformStamped.header.frame_id = parent_frame_id_;
  transformStamped.header.stamp = ros::Time::now();
  broadcaster_.sendTransform(transformStamped);
}

void DynamicTfPublisher::reconfiguration_callback(Config &config,
                                                  uint32_t level) {
  boost::mutex::scoped_lock lock(mutex_);
  config_ = config;
  x_ = config_.x;
  y_ = config_.y;
  z_ = config_.z;
  roll_ = config_.roll;
  yaw_ = config_.yaw;
  pitch_ = config_.pitch;
  parent_frame_id_ = config_.parent_frame_id;
  frame_id_ = config_.frame_id;

  tf::Quaternion q(yaw_, pitch_, roll_);
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
  qw_ = q.w();

  if (interactive_server_) {
    visualization_msgs::InteractiveMarker int_marker = createMarker();
    interactive_server_->insert(
      int_marker, boost::bind(&DynamicTfPublisher::processFeedback, this, _1));
    interactive_server_->applyChanges();
  }
}

visualization_msgs::InteractiveMarker DynamicTfPublisher::createMarker() {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = config_.parent_frame_id;
  int_marker.name = node_name_;
  int_marker.description = node_name_;

  int_marker.pose.position.x = x_;
  int_marker.pose.position.y = y_;
  int_marker.pose.position.z = z_;
  int_marker.pose.orientation.x = qx_;
  int_marker.pose.orientation.y = qy_;
  int_marker.pose.orientation.z = qz_;
  int_marker.pose.orientation.w = qw_;

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  return int_marker;
}

void DynamicTfPublisher::processFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  boost::mutex::scoped_lock lock(mutex_);
  x_ = feedback->pose.position.x;
  y_ = feedback->pose.position.y;
  z_ = feedback->pose.position.z;

  qx_ = feedback->pose.orientation.x;
  qy_ = feedback->pose.orientation.y;
  qz_ = feedback->pose.orientation.z;
  qw_ = feedback->pose.orientation.w;

  // get rpy from quaternion.
  tf::Quaternion q(qx_, qy_, qz_, qw_);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  need_config_update_ = true;
  interactive_server_->applyChanges();
}

bool DynamicTfPublisher::need_config_update_ = false;

}  // end of namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "jsk_interactive_marker");
  jsk_interactive_marker::DynamicTfPublisher dtp;
  dtp.onInit();
  ros::spin();
  return 0;
}
