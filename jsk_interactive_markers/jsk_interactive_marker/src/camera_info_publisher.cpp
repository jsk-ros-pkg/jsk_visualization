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

#include "jsk_interactive_marker/camera_info_publisher.h"
#include <sensor_msgs/distortion_models.h>
#include <tf/transform_broadcaster.h>

namespace jsk_interactive_marker
{
  CameraInfoPublisher::CameraInfoPublisher()
  {
    ros::NodeHandle nh, pnh("~");
    
    latest_pose_.orientation.w = 1.0;
    tf_listener_.reset(new tf::TransformListener());
    pub_camera_info_ = pnh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    // setup dynamic reconfigure
    srv_ = std::make_shared <dynamic_reconfigure::Server<Config> > (pnh);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &CameraInfoPublisher::configCallback, this, _1, _2);
    srv_->setCallback (f);

    // read parameters
    if (!pnh.getParam("frame_id", frame_id_)) {
      ROS_WARN("~frame_id is not specified, use camera as frame_id");
      frame_id_ = "camera";
    }
    if (!pnh.getParam("parent_frame_id", parent_frame_id_)) {
      ROS_WARN("~parent_frame_id is not specified, use base_link as parent_frame_id");
      parent_frame_id_ = "base_link";
    }

    // interactive marker
    server_.reset(new interactive_markers::InteractiveMarkerServer(
                    ros::this_node::getName()));
    initializeInteractiveMarker();
    bool sync_pointcloud;
    bool sync_image;
    
    if (!pnh.getParam("sync_pointcloud", sync_pointcloud)) {
      sync_pointcloud = false;
    }
    if (sync_pointcloud) {
      ROS_INFO("~sync_pointcloud is specified, synchronize ~camera_info to pointcloud");
      sub_sync_ = pnh.subscribe(
        "input", 1, &CameraInfoPublisher::pointcloudCallback, this);
    }
    else {
      if (!pnh.getParam("sync_image", sync_image)) {
        sync_image = false;
      }
      if (sync_image) {
        ROS_INFO("~sync_image is specified, synchronize ~camera_info to image");
        sub_sync_ = pnh.subscribe(
          "input", 1, &CameraInfoPublisher::imageCallback, this);
      }
      else {
        ROS_INFO("~sync_image or ~sync_pointcloud are not specified, use static_rate");
        double static_rate;
        pnh.param("static_rate", static_rate, 30.0); // defaults to 30 Hz
        timer_ = nh.createTimer(
          ros::Duration( 1 / static_rate ),
          boost::bind(&CameraInfoPublisher::staticRateCallback,
                      this, _1));
      }
    }
  }

  CameraInfoPublisher::~CameraInfoPublisher()
  {

  }
  
  void CameraInfoPublisher::initializeInteractiveMarker()
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = parent_frame_id_;
    int_marker.name = "camera info";
    im_helpers::add6DofControl(int_marker, false);
    server_->insert(int_marker,
                    boost::bind(&CameraInfoPublisher::processFeedback, this, _1));
    server_->applyChanges();
  }

  void CameraInfoPublisher::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseStamped new_pose, transformed_pose;
    new_pose.pose = feedback->pose;
    new_pose.header = feedback->header;
    try {
      tf_listener_->transformPose(
        parent_frame_id_,
        new_pose, transformed_pose);
      latest_pose_ = transformed_pose.pose;
    }
    catch (...) {
      ROS_FATAL("tf exception");
      return;
    }
  }


  void CameraInfoPublisher::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    width_ = config.width;
    height_ = config.height;
    f_ = config.f;
  }

  void CameraInfoPublisher::publishCameraInfo(const ros::Time& stamp)
  {
    boost::mutex::scoped_lock lock(mutex_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
    camera_info.height = height_;
    camera_info.width = width_;
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camera_info.D.resize(5, 0);
    camera_info.K.assign(0.0);
    camera_info.R.assign(0.0);
    camera_info.P.assign(0.0);
    camera_info.K[0] = camera_info.K[4] = f_;
    
    camera_info.K[0] = camera_info.P[0] = camera_info.K[4] = camera_info.P[5] = f_;
    camera_info.K[2] = camera_info.P[2] = width_ / 2.0;
    camera_info.K[5] = camera_info.P[6] = height_ / 2.0;
    camera_info.K[8] = camera_info.P[10] = 1.0;
    camera_info.R[0] = camera_info.R[4] = camera_info.R[8] = 1.0;
    pub_camera_info_.publish(camera_info);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(latest_pose_.position.x,
                                    latest_pose_.position.y,
                                    latest_pose_.position.z));
    tf::Quaternion q(latest_pose_.orientation.x,
                     latest_pose_.orientation.y,
                     latest_pose_.orientation.z,
                     latest_pose_.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, stamp,
                                          parent_frame_id_, frame_id_));
  }

  void CameraInfoPublisher::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publishCameraInfo(msg->header.stamp);
  }
  
  void CameraInfoPublisher::imageCallback(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    publishCameraInfo(msg->header.stamp);
  }
  
  void CameraInfoPublisher::staticRateCallback(
    const ros::TimerEvent& event)
  {
    publishCameraInfo(event.current_real);
  }
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_publisher");
  jsk_interactive_marker::CameraInfoPublisher publisher;
  ros::spin();
  return 0;
}
