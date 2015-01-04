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

#include <geometry_msgs/PoseStamped.h>

class Marker6DOF {
public:
Marker6DOF(): show_6dof_circle_(true) {
  ros::NodeHandle nh, pnh("~");
  pnh.param("frame_id", frame_id_, std::string("/map"));
  latest_pose_.header.frame_id = frame_id_;
  latest_pose_.pose.orientation.w = 1.0;
  pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pose_stamped_sub_ = pnh.subscribe("move_marker", 1, &Marker6DOF::moveMarkerCB, this);
  
  circle_menu_entry_
    = menu_handler_.insert("Toggle 6DOF Circle",
                           boost::bind(&Marker6DOF::menuFeedbackCB, this, _1));
  menu_handler_.setCheckState(circle_menu_entry_,
                              interactive_markers::MenuHandler::CHECKED);
  server_.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getName()));
  initializeInteractiveMarker();
}
  virtual ~Marker6DOF() {};
  
protected:
  void moveMarkerCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_pub_.publish(msg);
    server_->setPose("marker", msg->pose, msg->header);
    latest_pose_ = geometry_msgs::PoseStamped(*msg);
    server_->applyChanges();
  }
  
  void initializeInteractiveMarker() {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = latest_pose_.header.frame_id;
    int_marker.name = "marker";
    int_marker.pose = geometry_msgs::Pose(latest_pose_.pose);
    
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    sphere_marker.color.r = 0.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 1.0;
    sphere_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::InteractiveMarkerControl sphere_marker_control;
    sphere_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    sphere_marker_control.always_visible = true;
    sphere_marker_control.markers.push_back(sphere_marker);
    int_marker.controls.push_back(sphere_marker_control);
  
    visualization_msgs::InteractiveMarkerControl control;
  
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    if (show_6dof_circle_) {
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    if (show_6dof_circle_) {
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    if (show_6dof_circle_) {
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  
    server_->insert(int_marker,
                    boost::bind(&Marker6DOF::processFeedbackCB, this, _1));
    
    menu_handler_.apply(*server_, "marker");
    server_->applyChanges();
  }
  
  void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    geometry_msgs::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose = feedback->pose;
    latest_pose_ = pose;
    pose_pub_.publish(pose);
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
  
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Subscriber pose_stamped_sub_;
  ros::Publisher pose_pub_;
  std::string frame_id_;
  bool show_6dof_circle_;
  interactive_markers::MenuHandler::EntryHandle circle_menu_entry_;
  geometry_msgs::PoseStamped latest_pose_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_6dof");
  Marker6DOF marker;
  ros::spin();
  return 0;
}
