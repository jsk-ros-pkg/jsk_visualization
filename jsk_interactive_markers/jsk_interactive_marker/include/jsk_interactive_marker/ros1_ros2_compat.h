// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
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

#ifndef JSK_INTERACTIVE_MARKER_ROS1_ROS2_COMPAT_H_
#define JSK_INTERACTIVE_MARKER_ROS1_ROS2_COMPAT_H_

// ROS 1 / ROS 2 detection for the sources shared by both, the same as
// jsk_ros1_ros2_compat/ros_version.h: <ros/ros.h> defines
// ROS_VERSION_MAJOR to 1 on ROS 1, so `#if ROS_VERSION_MAJOR != 1` means
// ROS 2. It is repeated here, not included from jsk_ros1_ros2_compat,
// because jsk_ros1_ros2_compat (jsk_common >= 3.0) is not released for
// ROS 1, so this package depends on it on ROS 2 only. Compilers without
// __has_include (gcc < 5, e.g. indigo) are ROS 1.
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif

#if ROS_VERSION_MAJOR != 1
// ROS_INFO()/ROS_ERROR()... on ROS1_ROS2_COMPAT::g_node, which main()
// sets, JSK_ROS1_ROS2_COMPAT_MSG_ALIAS() for the ROS 1 spelling of the
// message types (visualization_msgs::Marker for
// visualization_msgs::msg::Marker), ...
#include <jsk_ros1_ros2_compat/compat.h>
// boost::bind(&Foo::callback, this, _1) of the ROS 1 sources, which get
// _1 through roscpp's headers there
#include <boost/bind/bind.hpp>
using namespace boost::placeholders;

// not in jsk_ros1_ros2_compat/logging.h
#ifndef ROS_FATAL
#define ROS_FATAL(...) RCLCPP_FATAL(ROS1_ROS2_COMPAT::g_node->get_logger(), __VA_ARGS__)
#endif

#include <chrono>
#include <string>
#include <vector>

namespace ros
{
  // the ros::TimerEvent of ROS 1's timer callbacks, given by
  // jsk_interactive_marker::createTimer() below
  struct TimerEvent
  {
    rclcpp::Time current_real;
  };
}

namespace jsk_interactive_marker
{
  // ROS 1's `ros::NodeHandle pnh("~")` for the parameters:
  // pnh.param(name, value, default), getParam() and hasParam() on the
  // parameters of the node. They are declared when first read, dynamically
  // typed as rospy does, so that e.g. `object_x: 1` is read as a double as
  // on ROS 1.
  class ParamHandle
  {
  public:
    explicit ParamHandle(rclcpp::Node::SharedPtr node) : node_(node) {}

    bool hasParam(const std::string& name)
    {
      return value(name).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
    }

    template<typename T>
    bool getParam(const std::string& name, T& out)
    {
      return convert(value(name), out);
    }

    template<typename T>
    void param(const std::string& name, T& out, const T& default_value)
    {
      if (!getParam(name, out)) {
        out = default_value;
        // shown by `ros2 param get` as with rospy.get_param()'s default
        node_->set_parameter(rclcpp::Parameter(name, rclcpp::ParameterValue(default_value)));
      }
    }

  protected:
    rclcpp::ParameterValue value(const std::string& name)
    {
      if (!node_->has_parameter(name)) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        node_->declare_parameter(name, rclcpp::ParameterValue(), descriptor);
      }
      return node_->get_parameter(name).get_parameter_value();
    }

    static bool convert(const rclcpp::ParameterValue& v, bool& out)
    {
      if (v.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) return false;
      out = v.get<bool>();
      return true;
    }
    static bool convert(const rclcpp::ParameterValue& v, int& out)
    {
      if (v.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) return false;
      out = v.get<int64_t>();
      return true;
    }
    static bool convert(const rclcpp::ParameterValue& v, double& out)
    {
      if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) out = v.get<double>();
      else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) out = v.get<int64_t>();
      else return false;
      return true;
    }
    static bool convert(const rclcpp::ParameterValue& v, std::string& out)
    {
      if (v.get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;
      out = v.get<std::string>();
      return true;
    }
    static bool convert(const rclcpp::ParameterValue& v, std::vector<double>& out)
    {
      if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        out = v.get<std::vector<double> >();
      }
      else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        std::vector<int64_t> integers = v.get<std::vector<int64_t> >();
        out.assign(integers.begin(), integers.end());
      }
      else return false;
      return true;
    }

    rclcpp::Node::SharedPtr node_;
  };

  // ROS 1's ros::Publisher for `pub.publish(msg)`, assigned from
  // node->create_publisher<MsgT>()
  template<typename MsgT>
  class Publisher
  {
  public:
    Publisher() {}
    Publisher(typename rclcpp::Publisher<MsgT>::SharedPtr pub) : pub_(pub) {}
    void publish(const MsgT& msg) const { pub_->publish(msg); }
  protected:
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
  };

  // ROS 1's nh.advertiseService(name, callback) with a callback of
  // (Request&, Response&), e.g. boost::bind(&Foo::service, this, _1, _2)
  template<typename SrvT, typename CallbackT>
  typename rclcpp::Service<SrvT>::SharedPtr advertiseService(
    rclcpp::Node::SharedPtr node, const std::string& name, CallbackT callback)
  {
    return node->create_service<SrvT>(
      name,
      [callback](const std::shared_ptr<typename SrvT::Request> req,
                 std::shared_ptr<typename SrvT::Response> res) {
        callback(*req, *res);
      });
  }

  // ROS 1's nh.createTimer(ros::Duration(period), callback), with the
  // ros::TimerEvent above for the callback
  template<typename CallbackT>
  rclcpp::TimerBase::SharedPtr createTimer(rclcpp::Node::SharedPtr node, double period, CallbackT callback)
  {
    rclcpp::Clock::SharedPtr clock = node->get_clock();
    return node->create_timer(
      std::chrono::duration<double>(period),
      [clock, callback]() {
        ros::TimerEvent event;
        event.current_real = clock->now();
        callback(event);
      });
  }
}
#endif

#endif  // JSK_INTERACTIVE_MARKER_ROS1_ROS2_COMPAT_H_
