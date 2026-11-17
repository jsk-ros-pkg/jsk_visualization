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

#include "jsk_interactive_marker/camera_info_publisher.h"
#if ROS_VERSION_MAJOR != 1
#include <sensor_msgs/distortion_models.hpp>
#else
#include <sensor_msgs/distortion_models.h>
#include <tf/transform_broadcaster.h>
#endif

namespace jsk_interactive_marker
{
  CameraInfoPublisher::CameraInfoPublisher()
  {
#if ROS_VERSION_MAJOR != 1
    node_ = ROS1_ROS2_COMPAT::g_node;
    jsk_interactive_marker::ParamHandle pnh(node_);

    latest_pose_.orientation.w = 1.0;
    tf_buffer_.reset(new tf2_ros::Buffer(node_->get_clock()));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(node_));
    pub_camera_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 1);
#else
    ros::NodeHandle nh, pnh("~");
    
    latest_pose_.orientation.w = 1.0;
    tf_listener_.reset(new tf::TransformListener());
    pub_camera_info_ = pnh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
#endif
    if (!pnh.getParam("yaml_filename", yaml_filename_)) {
      yaml_filename_ = "";
      ROS_WARN("~yaml_fliename is not specified, use default camera info parameters");
    }
    else {
      camera_info_yaml_ = YAML::LoadFile(yaml_filename_);
    }

#if ROS_VERSION_MAJOR != 1
    // the parameters of cfg/CameraInfoPublisher.cfg, configCallback() when set;
    // integers are taken as on ROS 1 (dynamically typed)
    {
      auto to_double = [](const rclcpp::ParameterValue& value) {
        return value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
          static_cast<double>(value.get<int64_t>()) : value.get<double>();
      };
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      descriptor.floating_point_range.resize(1);
      descriptor.floating_point_range[0].from_value = 1;
      descriptor.floating_point_range[0].to_value = 5000;
      descriptor.description = "width of camera info";
      config_.width = to_double(node_->declare_parameter("width", rclcpp::ParameterValue(640.0), descriptor));
      descriptor.description = "height of camera info";
      config_.height = to_double(node_->declare_parameter("height", rclcpp::ParameterValue(480.0), descriptor));
      descriptor.description = "f of camera_info, used as fx and fy";
      config_.f = to_double(node_->declare_parameter("f", rclcpp::ParameterValue(525.0), descriptor));
      configCallback(config_, 0);
      srv_ = node_->add_on_set_parameters_callback(
        [this, to_double](const std::vector<rclcpp::Parameter>& parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          Config config = config_;
          for (const rclcpp::Parameter& parameter : parameters) {
            if (parameter.get_name() != "width" && parameter.get_name() != "height" &&
                parameter.get_name() != "f") {
              continue;
            }
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
                parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
              result.successful = false;
              result.reason = parameter.get_name() + " must be a number";
              return result;
            }
            double value = to_double(parameter.get_parameter_value());
            if (parameter.get_name() == "width") config.width = value;
            else if (parameter.get_name() == "height") config.height = value;
            else config.f = value;
          }
          config_ = config;
          configCallback(config_, 0);
          return result;
        });
    }
#else
    // setup dynamic reconfigure
    srv_ = std::make_shared <dynamic_reconfigure::Server<Config> > (pnh);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &CameraInfoPublisher::configCallback, this, _1, _2);
    srv_->setCallback (f);
#endif

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
#if ROS_VERSION_MAJOR != 1
    server_.reset(new interactive_markers::InteractiveMarkerServer(
                    node_->get_fully_qualified_name(), node_));
#else
    server_.reset(new interactive_markers::InteractiveMarkerServer(
                    ros::this_node::getName()));
#endif
    initializeInteractiveMarker();
    bool sync_pointcloud;
    bool sync_image;
    
    if (!pnh.getParam("sync_pointcloud", sync_pointcloud)) {
      sync_pointcloud = false;
    }
    if (sync_pointcloud) {
      ROS_INFO("~sync_pointcloud is specified, synchronize ~camera_info to pointcloud");
#if ROS_VERSION_MAJOR != 1
      sub_sync_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input", 1, std::bind(&CameraInfoPublisher::pointcloudCallback, this, std::placeholders::_1));
#else
      sub_sync_ = pnh.subscribe(
        "input", 1, &CameraInfoPublisher::pointcloudCallback, this);
#endif
    }
    else {
      if (!pnh.getParam("sync_image", sync_image)) {
        sync_image = false;
      }
      if (sync_image) {
        ROS_INFO("~sync_image is specified, synchronize ~camera_info to image");
#if ROS_VERSION_MAJOR != 1
        sub_sync_ = node_->create_subscription<sensor_msgs::msg::Image>(
          "~/input", 1, std::bind(&CameraInfoPublisher::imageCallback, this, std::placeholders::_1));
#else
        sub_sync_ = pnh.subscribe(
          "input", 1, &CameraInfoPublisher::imageCallback, this);
#endif
      }
      else {
        ROS_INFO("~sync_image or ~sync_pointcloud are not specified, use static_rate");
        double static_rate;
        pnh.param("static_rate", static_rate, 30.0); // defaults to 30 Hz
#if ROS_VERSION_MAJOR != 1
        timer_ = jsk_interactive_marker::createTimer(
          node_, 1 / static_rate,
          boost::bind(&CameraInfoPublisher::staticRateCallback,
                      this, _1));
#else
        timer_ = nh.createTimer(
          ros::Duration( 1 / static_rate ),
          boost::bind(&CameraInfoPublisher::staticRateCallback,
                      this, _1));
#endif
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
#if ROS_VERSION_MAJOR != 1
      transformed_pose = tf_buffer_->transform(new_pose, parent_frame_id_);
#else
      tf_listener_->transformPose(
        parent_frame_id_,
        new_pose, transformed_pose);
#endif
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

#if ROS_VERSION_MAJOR != 1
  void CameraInfoPublisher::publishCameraInfo(const rclcpp::Time& stamp)
#else
  void CameraInfoPublisher::publishCameraInfo(const ros::Time& stamp)
#endif
  {
    boost::mutex::scoped_lock lock(mutex_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
#if ROS_VERSION_MAJOR != 1
    // the fields are d, k, r, p on ROS 2 (D, K, R, P on ROS 1)
    if (yaml_filename_ != "") {
      camera_info.height = camera_info_yaml_["image_height"].as<uint32_t>();
      camera_info.width = camera_info_yaml_["image_width"].as<uint32_t>();
      camera_info.distortion_model =
        camera_info_yaml_["camera_model"].as<std::string>();
      std::vector<double> K, R, P;
      camera_info.d = camera_info_yaml_["distortion_coefficients"]["data"].as<std::vector<double>>();
      K = camera_info_yaml_["camera_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&camera_info.k[0], &K[0], sizeof(double)*9);
      R = camera_info_yaml_["rectification_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&camera_info.r[0], &R[0], sizeof(double)*9);
      P = camera_info_yaml_["projection_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&camera_info.p[0], &P[0], sizeof(double)*12);
    }
    else {
      camera_info.height = height_;
      camera_info.width = width_;
      camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      camera_info.d.resize(5, 0);
      camera_info.k.fill(0.0);
      camera_info.r.fill(0.0);
      camera_info.p.fill(0.0);
      camera_info.k[0] = camera_info.p[0] = camera_info.k[4] = camera_info.p[5] = f_;
      camera_info.k[2] = camera_info.p[2] = width_ / 2.0;
      camera_info.k[5] = camera_info.p[6] = height_ / 2.0;
      camera_info.k[8] = camera_info.p[10] = 1.0;
      camera_info.r[0] = camera_info.r[4] = camera_info.r[8] = 1.0;
    }
    pub_camera_info_->publish(camera_info);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_id_;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = latest_pose_.position.x;
    transform.transform.translation.y = latest_pose_.position.y;
    transform.transform.translation.z = latest_pose_.position.z;
    transform.transform.rotation = latest_pose_.orientation;
    tf_broadcaster_->sendTransform(transform);
#else
    if (yaml_filename_ != "") {
      camera_info.height = camera_info_yaml_["image_height"].as<uint32_t>();
      camera_info.width = camera_info_yaml_["image_width"].as<uint32_t>();
      camera_info.distortion_model =
        camera_info_yaml_["camera_model"].as<std::string>();
      std::vector<double> D, K, R, P;
      boost::array<double, 9ul> Kl, Rl;
      boost::array<double, 12ul> Pl;
      D = camera_info_yaml_["distortion_coefficients"]["data"].as<std::vector<double>>();
      K = camera_info_yaml_["camera_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Kl[0], &K[0], sizeof(double)*9);
      R = camera_info_yaml_["rectification_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Rl[0], &R[0], sizeof(double)*9);
      P = camera_info_yaml_["projection_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Pl[0], &P[0], sizeof(double)*12);
      camera_info.D = D;
      camera_info.K = Kl;
      camera_info.R = Rl;
      camera_info.P = Pl;
    }
    else {
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
    }
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
#endif
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
#if ROS_VERSION_MAJOR != 1
  rclcpp::init(argc, argv);
  ROS1_ROS2_COMPAT::g_node = rclcpp::Node::make_shared("camera_info_publisher");
  {
    jsk_interactive_marker::CameraInfoPublisher publisher;
    rclcpp::spin(ROS1_ROS2_COMPAT::g_node);
  }
  ROS1_ROS2_COMPAT::g_node.reset();
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "camera_info_publisher");
  jsk_interactive_marker::CameraInfoPublisher publisher;
  ros::spin();
#endif
  return 0;
}
