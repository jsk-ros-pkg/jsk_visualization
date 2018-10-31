// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include "jsk_interactive_marker/pointcloud_cropper.h"
#include "jsk_interactive_marker/interactive_marker_helpers.h"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>

#include <pcl_ros/pcl_nodelet.h>

//using namespace jsk_interactive_marker;
namespace jsk_interactive_marker {
  Cropper::Cropper(const unsigned int nr_parameter):
    nr_parameter_(nr_parameter),
    pose_(Eigen::Affine3f::Identity())
  {
    parameters_.resize(nr_parameter_);
  }

  Cropper::~Cropper()
  {

  }

  void Cropper::setPose(Eigen::Affine3f pose)
  {
    pose_ = pose;
  }

  Eigen::Affine3f Cropper::getPose()
  {
    return pose_;
  }

  void Cropper::crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output)
  {
    Eigen::Vector3f transf(pose_.translation());
    ROS_DEBUG("%s transf: %f %f %f", __FUNCTION__, transf[0], transf[1], transf[2]);
    output->points.clear();
    for (size_t i = 0; i < input->points.size(); i++) {
      pcl::PointXYZ p = input->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
        if (isInside(p)) {
          output->points.push_back(p);
        }
      }
    }
  }


  void Cropper::updateParameter(const double param, const unsigned int index)
  {
    if (parameters_.size() >= index + 1) {
      parameters_[index] = param;
    }
  }
  
  SphereCropper::SphereCropper(): Cropper(1)
  {
    fillInitialParameters();    // not so good?
  }

  SphereCropper::~SphereCropper()
  {

  }

  bool SphereCropper::isInside(const pcl::PointXYZ& p)
  {
    Eigen::Vector3f pos = p.getVector3fMap();
    Eigen::Vector3f origin(pose_.translation());
    double distance = (pos - origin).norm();
    // ROS_DEBUG("pos: [%f, %f, %f], origin: [%f, %f, %f], distance: %f, R: %f",
    //          pos[0], pos[1], pos[2],
    //          origin[0], origin[1], origin[2],
    //          distance, getRadius());
    if (distance < getRadius()) {
      return true;
    }
    else {
      return false;
    }
  }

  double SphereCropper::getRadius()
  {
    return parameters_[0];
  }

  void SphereCropper::fillInitialParameters()
  {
    parameters_[0] = 0.5;         // 50cm
  }

  std::string SphereCropper::getName()
  {
    return "SphereCropper";
  }

  visualization_msgs::Marker SphereCropper::getMarker()
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = getRadius() * 2;
    marker.scale.y = getRadius() * 2;
    marker.scale.z = getRadius() * 2;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }

  CubeCropper::CubeCropper(): Cropper(3)
  {
    fillInitialParameters();    // not so good?
  }

  CubeCropper::~CubeCropper()
  {

  }

  std::string CubeCropper::getName()
  {
    return "CubeCropper";
  }

  visualization_msgs::Marker CubeCropper::getMarker()
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = getWidthX() * 2;
    marker.scale.y = getWidthY() * 2;
    marker.scale.z = getWidthZ() * 2;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }

  void CubeCropper::fillInitialParameters()
  {
    parameters_[0] = 0.5;         // 50cm
    parameters_[1] = 0.5;         // 50cm
    parameters_[2] = 0.5;         // 50cm
  }

  double CubeCropper::getWidthX()
  {
    return parameters_[0];
  }

  double CubeCropper::getWidthY()
  {
    return parameters_[1];
  }

  double CubeCropper::getWidthZ()
  {
    return parameters_[2];
  }

  bool CubeCropper::isInside(const pcl::PointXYZ& p)
  {
    Eigen::Vector3f pos = p.getVector3fMap();
    Eigen::Vector3f diff = pos - pose_.translation();
    if ((fabs(diff[0]) < getWidthX()) &&
        (fabs(diff[1]) < getWidthY()) &&
        (fabs(diff[2]) < getWidthZ())) {
      return true;
    }
    else {
      return false;
    }
  }
  
  PointCloudCropper::PointCloudCropper(ros::NodeHandle& nh, ros::NodeHandle &pnh)
  {
    tf_listener_.reset(new tf::TransformListener);
    // initialize cropper_candidates_
    cropper_candidates_.push_back(std::make_shared<SphereCropper>());
    cropper_candidates_.push_back(std::make_shared<CubeCropper>());
    cropper_ = cropper_candidates_[0];
    point_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("output", 1);
    point_visualization_pub_ = pnh.advertise<sensor_msgs::PointCloud2>(
      "visualization_pointcloud", 1);
    server_.reset(new interactive_markers::InteractiveMarkerServer(
                    ros::this_node::getName()));
    initializeInteractiveMarker();
    srv_ = std::make_shared <dynamic_reconfigure::Server<Config> > (pnh);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PointCloudCropper::configCallback, this, _1, _2);
    srv_->setCallback (f);
    point_sub_ = pnh.subscribe("input", 1, &PointCloudCropper::inputCallback, this);
  }

  PointCloudCropper::~PointCloudCropper()
  {
    
  }

  void PointCloudCropper::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    for (size_t i = 0; i < cropper_candidates_.size(); i++) {
      cropper_candidates_[i]->updateParameter(config.param0, 0);
      cropper_candidates_[i]->updateParameter(config.param1, 1);
      cropper_candidates_[i]->updateParameter(config.param2, 2);
    }
    reInitializeInteractiveMarker();
  }
  
  void PointCloudCropper::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped input_pose_stamped, transformed_pose_stamped;
    input_pose_stamped.pose = feedback->pose;
    input_pose_stamped.header.stamp = feedback->header.stamp;
    input_pose_stamped.header.frame_id = feedback->header.frame_id;
    if (!latest_pointcloud_) {
      ROS_WARN("no pointcloud is available yet");
      return;
    }
    // 1. update cropper's pose according to the posigoin of the process feedback
    try {
      tf_listener_->transformPose(
        latest_pointcloud_->header.frame_id,
        input_pose_stamped,
        transformed_pose_stamped);
    }
    catch (...) {
      ROS_FATAL("tf exception");
      return;
    }
    Eigen::Affine3d pose_d;
    tf::poseMsgToEigen(transformed_pose_stamped.pose, pose_d);
    // convert Eigen::Affine3d to Eigen::Affine3f
    Eigen::Vector3d transd(pose_d.translation());
    Eigen::Quaterniond rotated(pose_d.rotation());
    Eigen::Vector3f transf(transd[0], transd[1], transd[2]);
    Eigen::Quaternionf rotatef(rotated.w(),
                               rotated.x(), rotated.y(), rotated.z());
    Eigen::Affine3f pose_f = Eigen::Translation3f(transf) * rotatef;
    ROS_DEBUG("transf: %f %f %f", transf[0], transf[1], transf[2]);
    cropper_->setPose(pose_f);
    // 2. crop pointcloud
    cropAndPublish(point_visualization_pub_);
  }

  void PointCloudCropper::cropAndPublish(ros::Publisher& pub)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*latest_pointcloud_, *input);
    cropper_->crop(input, output);
    ROS_DEBUG_STREAM(output->points.size() << " points to be cropped");
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*output, ros_output);
    ros_output.header = latest_pointcloud_->header;
    pub.publish(ros_output);
  }
      
  void PointCloudCropper::initializeInteractiveMarker(
    Eigen::Affine3f pose_offset)
  {
    updateInteractiveMarker();
    // menu
    menu_handler_.insert(
      "Crop",
      boost::bind(&PointCloudCropper::menuFeedback, this, _1));
    // submenu to change the cropper
    interactive_markers::MenuHandler::EntryHandle sub_cropper_menu_handle
      = menu_handler_.insert("Switch");
    cropper_entries_.clear();
    for (size_t i = 0; i < cropper_candidates_.size(); i++) {
      Cropper::Ptr the_cropper = cropper_candidates_[i];
      interactive_markers::MenuHandler::EntryHandle cropper_entry
        = menu_handler_.insert(
          sub_cropper_menu_handle, the_cropper->getName(),
          boost::bind(&PointCloudCropper::changeCropperCallback, this, _1));
      if (the_cropper != cropper_) {
        menu_handler_.setCheckState(
          cropper_entry,
          interactive_markers::MenuHandler::UNCHECKED);
      }
      else {
        menu_handler_.setCheckState(
          cropper_entry,
          interactive_markers::MenuHandler::CHECKED);
      }
      cropper_entries_.push_back(cropper_entry);
    }
    menu_handler_.apply(*server_, "pointcloud cropper");
    server_->applyChanges();
  }
  
  void PointCloudCropper::reInitializeInteractiveMarker()
  {
    if (server_) {
      updateInteractiveMarker(cropper_->getPose());
      // update checkbox status of the menu
      updateMenuCheckboxStatus();
      menu_handler_.reApply(*server_);
      server_->applyChanges();
    }
  }
  
  void PointCloudCropper::updateInteractiveMarker(
    Eigen::Affine3f pose_offset)
  {
    visualization_msgs::InteractiveMarker int_marker;
    if (latest_pointcloud_) {
      int_marker.header.frame_id = latest_pointcloud_->header.frame_id;
    }
    else {
      int_marker.header.frame_id = "/camera_link";
    }
    int_marker.name = "pointcloud cropper";
    int_marker.description = cropper_->getName();
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    visualization_msgs::Marker cropper_marker = cropper_->getMarker();
    control.interaction_mode
      = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.markers.push_back(cropper_marker);
    int_marker.controls.push_back(control);
    // set the position of the cropper_marker
    Eigen::Vector3f offset_pos(pose_offset.translation());
    Eigen::Quaternionf offset_rot(pose_offset.rotation());
    int_marker.pose.position.x = offset_pos[0];
    int_marker.pose.position.y = offset_pos[1];
    int_marker.pose.position.z = offset_pos[2];
    int_marker.pose.orientation.x = offset_rot.x();
    int_marker.pose.orientation.y = offset_rot.y();
    int_marker.pose.orientation.z = offset_rot.z();
    int_marker.pose.orientation.w = offset_rot.w();
    control.markers.push_back(cropper_marker);
    ROS_DEBUG("pos: %f, %f, %f", int_marker.pose.position.x,
              int_marker.pose.position.y,
              int_marker.pose.position.z);
    ROS_DEBUG("rot: %f.; %f, %f, %f", int_marker.pose.orientation.w,
              int_marker.pose.orientation.x,
              int_marker.pose.orientation.y,
              int_marker.pose.orientation.z);

    // add 6dof marker
    im_helpers::add6DofControl(int_marker, false);
    
    server_->insert(int_marker,
                    boost::bind(&PointCloudCropper::processFeedback, this, _1));    
  }

  void PointCloudCropper::changeCropperCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    unsigned int menu_entry_id = feedback->menu_entry_id;
    EntryHandleVector::iterator it = std::find(cropper_entries_.begin(),
                                               cropper_entries_.end(),
                                               menu_entry_id);
    size_t index = std::distance(cropper_entries_.begin(), it);
    if (index >= cropper_candidates_.size()) {
      ROS_ERROR("the index of the chosen cropper is out of the"
                "range of candidate");
      return;
    }
    Cropper::Ptr next_cropper = cropper_candidates_[index];
    if (next_cropper == cropper_) {
      ROS_DEBUG("same cropper");
      return;
    }
    else {
      changeCropper(next_cropper);
    }
  }

  void PointCloudCropper::updateMenuCheckboxStatus()
  {
    for (size_t i = 0; i < cropper_candidates_.size(); i++) {
      Cropper::Ptr the_cropper = cropper_candidates_[i];
      if (the_cropper == cropper_) {
        menu_handler_.setCheckState(
          cropper_entries_[i],
          interactive_markers::MenuHandler::CHECKED);
      }
      else {
        menu_handler_.setCheckState(
          cropper_entries_[i],
          interactive_markers::MenuHandler::UNCHECKED);
      }
    }
  }
  
  void PointCloudCropper::changeCropper(Cropper::Ptr next_cropper)
  {
    next_cropper->setPose(cropper_->getPose());
    cropper_ = next_cropper;
    
    reInitializeInteractiveMarker();
  }

  void PointCloudCropper::menuFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    unsigned int menu_entry_id = feedback->menu_entry_id;
    if (menu_entry_id == 1) {   // "Crop"
      cropAndPublish(point_pub_);
    }
  }

  void PointCloudCropper::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_pointcloud_ = msg;
    cropAndPublish(point_visualization_pub_);
  }
  
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_cropper");
  ros::NodeHandle nh, pnh("~");
  jsk_interactive_marker::PointCloudCropper cropper(nh, pnh);
  ros::spin();
}
