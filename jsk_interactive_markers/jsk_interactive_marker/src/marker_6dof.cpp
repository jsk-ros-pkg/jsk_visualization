// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <jsk_topic_tools/rosparam_utils.h>

class Marker6DOF {
public:
  Marker6DOF(): show_6dof_circle_(true) {
    ros::NodeHandle nh, pnh("~");
    pnh.param("publish_tf", publish_tf_, false);
    pnh.param("publish_pose_periodically", publish_pose_periodically_, false);
    pnh.param("tf_frame", tf_frame_, std::string("object"));
    double tf_duration;
    pnh.param("tf_duration", tf_duration, 0.1);
    pnh.param("object_type", object_type_, std::string("sphere"));
    pnh.param("object_x", object_x_, 1.0);
    pnh.param("object_y", object_y_, 1.0);
    pnh.param("object_z", object_z_, 1.0);
    pnh.param("object_r", object_r_, 1.0);
    pnh.param("object_g", object_g_, 1.0);
    pnh.param("object_b", object_b_, 1.0);
    pnh.param("object_a", object_a_, 1.0);
    std::string frame_id;
    pnh.param("frame_id", frame_id, std::string("/map"));
    latest_pose_.header.frame_id = frame_id;
    double initial_x, initial_y, initial_z;
    pnh.param("initial_x", initial_x, 0.0);
    pnh.param("initial_y", initial_y, 0.0);
    pnh.param("initial_z", initial_z, 0.0);
    latest_pose_.pose.position.x = initial_x;
    latest_pose_.pose.position.y = initial_y;
    latest_pose_.pose.position.z = initial_z;
    std::vector<double> initial_orientation;
    if (jsk_topic_tools::readVectorParameter(pnh, "initial_orientation", initial_orientation)) {
      latest_pose_.pose.orientation.x = initial_orientation[0];
      latest_pose_.pose.orientation.y = initial_orientation[1];
      latest_pose_.pose.orientation.z = initial_orientation[2];
      latest_pose_.pose.orientation.w = initial_orientation[3];
    }
    else {
      latest_pose_.pose.orientation.w = 1.0;
    }
    pnh.param("line_width", line_width_, 0.007);
    pnh.param("mesh_file", mesh_file_, std::string(""));
    if (pnh.hasParam("interactive_marker_scale")) {
      pnh.param("interactive_marker_scale", int_marker_scale_, 1.0);
    } else {
      int_marker_scale_ = std::max(object_x_, std::max(object_y_, object_z_)) + 0.5;
    }
    if (publish_tf_) {
      tf_broadcaster_.reset(new tf::TransformBroadcaster);
    }
    
    pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pose_stamped_sub_ = pnh.subscribe("move_marker", 1, &Marker6DOF::moveMarkerCB, this);
  
    circle_menu_entry_
      = menu_handler_.insert("Toggle 6DOF Circle",
                             boost::bind(&Marker6DOF::menuFeedbackCB, this, _1));
    menu_handler_.setCheckState(circle_menu_entry_,
                                interactive_markers::MenuHandler::CHECKED);
    server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
    initializeInteractiveMarker();
    // Timer to update current pose on Rviz in the case which user re-enabled the plugin
    timer_pose_ = nh.createTimer(ros::Duration(0.1), boost::bind(&Marker6DOF::timerPoseCallback, this, _1));
    if (publish_tf_) {
      timer_tf_ = nh.createTimer(ros::Duration(tf_duration), boost::bind(&Marker6DOF::timerTFCallback, this, _1));
    }
  }
  
protected:
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    if(!publish_pose_periodically_) {
      pose_pub_.publish(msg);
    }
    server_->setPose("marker", msg->pose, msg->header);
    latest_pose_ = geometry_msgs::PoseStamped(*msg);
    server_->applyChanges();
  }

  
  void calculateBoundingBox( visualization_msgs::Marker& object_marker){
    geometry_msgs::Point top[5];
    top[0].x = object_x_/2;
    top[0].y = object_y_/2;
    top[1].x = -object_x_/2;
    top[1].y = object_y_/2;
    top[2].x = -object_x_/2;
    top[2].y = -object_y_/2;
    top[3].x = object_x_/2;
    top[3].y = -object_y_/2;    
    top[4].x = object_x_/2;
    top[4].y = object_y_/2;

    geometry_msgs::Point bottom[5];
    bottom[0].x = object_x_/2;
    bottom[0].y = object_y_/2;
    bottom[1].x = -object_x_/2;
    bottom[1].y = object_y_/2;
    bottom[2].x = -object_x_/2;
    bottom[2].y = -object_y_/2;
    bottom[3].x = object_x_/2;
    bottom[3].y = -object_y_/2;
    bottom[4].x = object_x_/2;
    bottom[4].y = object_y_/2;

    for(int i = 0; i< 5; i++){
      top[i].z = object_z_/2;
      bottom[i].z = -object_z_/2;
    }

    for(int i = 0; i< 4; i++){
      object_marker.points.push_back(top[i]);
      object_marker.points.push_back(top[i+1]);
      object_marker.points.push_back(bottom[i]);
      object_marker.points.push_back(bottom[i+1]);
      object_marker.points.push_back(top[i]);
      object_marker.points.push_back(bottom[i]);
    }
  }
  
  void initializeInteractiveMarker() {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = latest_pose_.header.frame_id;
    int_marker.name = "marker";
    int_marker.pose = geometry_msgs::Pose(latest_pose_.pose);
    
    visualization_msgs::Marker object_marker;
    if(object_type_ == std::string("cube")){
      object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
    }
    else if( object_type_ == std::string("sphere") ){
      object_marker.type = visualization_msgs::Marker::SPHERE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
    }
    else if(object_type_ == std::string("line")){
      object_marker.type = visualization_msgs::Marker::LINE_LIST;
      object_marker.scale.x = line_width_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
      calculateBoundingBox(object_marker);
    }
    else if(object_type_ == std::string("mesh")){
      object_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
      object_marker.mesh_resource = mesh_file_;
    }

    
    visualization_msgs::InteractiveMarkerControl object_marker_control;
    object_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    object_marker_control.always_visible = true;
    object_marker_control.markers.push_back(object_marker);
    int_marker.controls.push_back(object_marker_control);
  
    visualization_msgs::InteractiveMarkerControl control;
    if (show_6dof_circle_) {
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
    }
  
    int_marker.scale = int_marker_scale_;

    server_->insert(int_marker,
                    boost::bind(&Marker6DOF::processFeedbackCB, this, _1));
    
    menu_handler_.apply(*server_, "marker");
    server_->applyChanges();
  }

  void publishTF(const geometry_msgs::PoseStamped& pose) {
    tf::Transform transform;
    tf::poseMsgToTF(pose.pose, transform);
    tf_broadcaster_->sendTransform(tf::StampedTransform(
                                     transform, pose.header.stamp,
                                     pose.header.frame_id,
                                     tf_frame_));
  }
  
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose = feedback->pose;
    latest_pose_ = pose;
    if (!publish_pose_periodically_) {
      pose_pub_.publish(pose);
    }
  }

  void menuFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    show_6dof_circle_ = !show_6dof_circle_;
    if (show_6dof_circle_) {
      menu_handler_.setCheckState(circle_menu_entry_,
                                  interactive_markers::MenuHandler::CHECKED);
    }
    else {
      menu_handler_.setCheckState(circle_menu_entry_,
                                  interactive_markers::MenuHandler::UNCHECKED);
    }
    initializeInteractiveMarker(); // ok...?
  }

  void timerPoseCallback(const ros::TimerEvent& e) {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped pose = latest_pose_;
    pose.header.stamp = e.current_real;
    server_->setPose("marker", pose.pose, pose.header);
    server_->applyChanges();
    if (publish_pose_periodically_) {
      pose_pub_.publish(pose);
    }
  }

  void timerTFCallback(const ros::TimerEvent& e) {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped pose = latest_pose_;
    pose.header.stamp = e.current_real;
    publishTF(pose);
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Subscriber pose_stamped_sub_;
  ros::Publisher pose_pub_;
  std::string object_type_;
  double object_x_;
  double object_y_;
  double object_z_;
  double object_r_;
  double object_g_;
  double object_b_;
  double object_a_;
  double line_width_;
  double int_marker_scale_;
  std::string mesh_file_;
  bool show_6dof_circle_;
  bool publish_tf_;
  bool publish_pose_periodically_;
  std::string tf_frame_;
  ros::Timer timer_pose_;
  ros::Timer timer_tf_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  boost::mutex mutex_;
  interactive_markers::MenuHandler::EntryHandle circle_menu_entry_;
  geometry_msgs::PoseStamped latest_pose_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_6dof");
  Marker6DOF marker;
  ros::spin();
  return 0;
}
