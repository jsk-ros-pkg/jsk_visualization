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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_interactive_marker/ros1_ros2_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/tools.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <boost/thread/mutex.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, Point)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, Pose)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, PoseStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(visualization_msgs, InteractiveMarker)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(visualization_msgs, InteractiveMarkerControl)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(visualization_msgs, Marker)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(visualization_msgs, InteractiveMarkerFeedback)
#include <jsk_ros1_ros2_compat/const_ptr.h>
namespace jsk_topic_tools
{
  inline bool readVectorParameter(jsk_interactive_marker::ParamHandle& pnh, const std::string& name,
                                  std::vector<double>& result)
  {
    return pnh.getParam(name, result);
  }
}
#else
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <jsk_topic_tools/rosparam_utils.h>
#endif

class Marker6DOF {
public:
  Marker6DOF(): show_6dof_circle_(true) {
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node = ROS1_ROS2_COMPAT::g_node;
    jsk_interactive_marker::ParamHandle pnh(node);
#else
    ros::NodeHandle nh, pnh("~");
#endif
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
    pnh.param("frame_id", frame_id_, std::string("/map"));
#if ROS_VERSION_MAJOR != 1
    // tf2 of ROS 2 does not take the leading slash of ROS 1's frame ids
    if (!frame_id_.empty() && frame_id_[0] == '/') {
      frame_id_.erase(0, 1);
    }
#endif
    latest_pose_.header.frame_id = frame_id_;
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
#if ROS_VERSION_MAJOR != 1
    if (publish_tf_) {
      tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(node));
      tf_buffer_.reset(new tf2_ros::Buffer(node->get_clock()));
      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    }

    pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
    pose_stamped_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/move_marker", 1, std::bind(&Marker6DOF::moveMarkerCB, this, std::placeholders::_1));
#else
    if (publish_tf_) {
      tf_broadcaster_.reset(new tf::TransformBroadcaster);
      tf_listener_.reset(new tf::TransformListener);
    }
    
    pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pose_stamped_sub_ = pnh.subscribe("move_marker", 1, &Marker6DOF::moveMarkerCB, this);
#endif
  
    circle_menu_entry_
      = menu_handler_.insert("Toggle 6DOF Circle",
                             boost::bind(&Marker6DOF::menuFeedbackCB, this, _1));
    menu_handler_.setCheckState(circle_menu_entry_,
                                interactive_markers::MenuHandler::CHECKED);
#if ROS_VERSION_MAJOR != 1
    server_.reset( new interactive_markers::InteractiveMarkerServer(node->get_fully_qualified_name(), node));
    initializeInteractiveMarker();
    // Timer to update current pose on Rviz in the case which user re-enabled the plugin
    timer_pose_ = jsk_interactive_marker::createTimer(node, 0.1, boost::bind(&Marker6DOF::timerPoseCallback, this, _1));
    if (publish_tf_) {
      timer_tf_ = jsk_interactive_marker::createTimer(node, tf_duration, boost::bind(&Marker6DOF::timerTFCallback, this, _1));
    }
#else
    server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
    initializeInteractiveMarker();
    // Timer to update current pose on Rviz in the case which user re-enabled the plugin
    timer_pose_ = nh.createTimer(ros::Duration(0.1), boost::bind(&Marker6DOF::timerPoseCallback, this, _1));
    if (publish_tf_) {
      timer_tf_ = nh.createTimer(ros::Duration(tf_duration), boost::bind(&Marker6DOF::timerTFCallback, this, _1));
    }
#endif
  }
  
protected:
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    if(!publish_pose_periodically_) {
#if ROS_VERSION_MAJOR != 1
      pose_pub_->publish(*msg);
#else
      pose_pub_.publish(msg);
#endif
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
#if ROS_VERSION_MAJOR != 1
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = tf_frame_;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
#else
    tf::Transform transform;
    tf::poseMsgToTF(pose.pose, transform);
    tf_broadcaster_->sendTransform(tf::StampedTransform(
                                     transform, pose.header.stamp,
                                     pose.header.frame_id,
                                     tf_frame_));
#endif
  }
  
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose = feedback->pose;

    if (publish_tf_) {
      // feedback->header.frame_id equals to fixed frame of rviz.
      // Pose should be transformed respect to frame_id_ to publish correct tf frames.
      try {
#if ROS_VERSION_MAJOR != 1
        latest_pose_ = tf_buffer_->transform(pose, frame_id_);
#else
        tf_listener_->transformPose(frame_id_, pose, latest_pose_);
#endif
      }
      catch (tf2::TransformException& e) {
        ROS_ERROR_STREAM("Failed to transform " << pose.header.frame_id << " to " << frame_id_ << ": " << e.what());
        return;
      }
    }
    else {
      latest_pose_ = pose;
    }
    if (!publish_pose_periodically_) {
#if ROS_VERSION_MAJOR != 1
      pose_pub_->publish(pose);
#else
      pose_pub_.publish(pose);
#endif
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
#if ROS_VERSION_MAJOR != 1
      pose_pub_->publish(pose);
#else
      pose_pub_.publish(pose);
#endif
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
#if ROS_VERSION_MAJOR != 1
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
#else
  ros::Subscriber pose_stamped_sub_;
  ros::Publisher pose_pub_;
#endif
  std::string object_type_;
  double object_x_;
  double object_y_;
  double object_z_;
  double object_r_;
  double object_g_;
  double object_b_;
  double object_a_;
  std::string frame_id_;
  double line_width_;
  double int_marker_scale_;
  std::string mesh_file_;
  bool show_6dof_circle_;
  bool publish_tf_;
  bool publish_pose_periodically_;
  std::string tf_frame_;
#if ROS_VERSION_MAJOR != 1
  rclcpp::TimerBase::SharedPtr timer_pose_;
  rclcpp::TimerBase::SharedPtr timer_tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
#else
  ros::Timer timer_pose_;
  ros::Timer timer_tf_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
#endif
  boost::mutex mutex_;
  interactive_markers::MenuHandler::EntryHandle circle_menu_entry_;
  geometry_msgs::PoseStamped latest_pose_;
};


int main(int argc, char** argv) {
#if ROS_VERSION_MAJOR != 1
  rclcpp::init(argc, argv);
  ROS1_ROS2_COMPAT::g_node = rclcpp::Node::make_shared("marker_6dof");
  {
    Marker6DOF marker;
    rclcpp::spin(ROS1_ROS2_COMPAT::g_node);
  }
  ROS1_ROS2_COMPAT::g_node.reset();
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "marker_6dof");
  Marker6DOF marker;
  ros::spin();
#endif
  return 0;
}
