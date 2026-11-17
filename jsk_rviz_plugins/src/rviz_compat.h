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

#ifndef JSK_RVIZ_PLUGINS_RVIZ_COMPAT_H_
#define JSK_RVIZ_PLUGINS_RVIZ_COMPAT_H_

// ROS 1 (rviz) / ROS 2 (rviz_common) compatibility of jsk_rviz_plugins.
//
// The sources of jsk_rviz_plugins are shared by ROS 1 and ROS 2 and tell
// them apart with `#if ROS_VERSION_MAJOR != 1`, as jsk_common does
// (jsk_ros1_ros2_compat/ros_version.h): roscpp is only on the include path
// of a ROS 1 (catkin) build, and its ros/common.h defines
// ROS_VERSION_MAJOR to 1; on ROS 2 the macro stays undefined and reads as
// 0. The ROS 1 build of this package does not depend on
// jsk_ros1_ros2_compat, hence the same detection is spelled out here.
// __has_include is unknown to old compilers (e.g. GCC 4.8 on indigo),
// which only exist on ROS 1.
#if defined(__has_include)
#if __has_include(<ros/ros.h>)
#include <ros/ros.h>
#endif
#else
#include <ros/ros.h>
#endif

#if ROS_VERSION_MAJOR != 1
#include <chrono>
#include <rclcpp/rclcpp.hpp>
// JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(pkg, Type): pkg::Type -> pkg::msg::Type
#include <jsk_ros1_ros2_compat/msg_alias.h>
#include <rviz_common/properties/parse_color.hpp>

// rviz_common, rviz_rendering and rviz_default_plugins keep the class
// names of rviz, in other namespaces. As JSK_ROS1_ROS2_COMPAT_MSG_ALIAS()
// does for the message types, the rviz:: names used by jsk_rviz_plugins
// are reopened as aliases of their ROS 2 counterparts, so that the code
// using them is the same for ROS 1 and ROS 2; the headers declaring them
// are included by each source, in its ROS 2 branch.
namespace rviz_common
{
class Config;
class Display;
class DisplayContext;
class DisplayGroup;
template<class MessageType> class MessageFilterDisplay;
class Panel;
class RenderPanel;
class Tool;
class ToolManager;
class UniformStringStream;
class ViewManager;
class ViewportMouseEvent;
namespace properties
{
class BoolProperty;
class ColorProperty;
class EditableEnumProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class PropertyTreeModel;
class RosTopicProperty;
class StatusProperty;
class StringProperty;
class TfFrameProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_rendering
{
class Arrow;
class BillboardLine;
class MovableText;
}  // namespace rviz_rendering

namespace rviz_default_plugins
{
namespace displays
{
class InteractiveMarkerDisplay;
}  // namespace displays
}  // namespace rviz_default_plugins

namespace rviz
{
using rviz_common::Config;
using rviz_common::Display;
using rviz_common::DisplayContext;
using rviz_common::DisplayGroup;
template<class MessageType>
using MessageFilterDisplay = rviz_common::MessageFilterDisplay<MessageType>;
using rviz_common::Panel;
using rviz_common::RenderPanel;
using rviz_common::Tool;
using rviz_common::ToolManager;
using rviz_common::UniformStringStream;
using rviz_common::ViewManager;
using rviz_common::ViewportMouseEvent;

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::EditableEnumProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;
using rviz_common::properties::PropertyTreeModel;
using rviz_common::properties::RosTopicProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::TfFrameProperty;
using rviz_common::properties::VectorProperty;
using rviz_common::properties::ogreToQt;
using rviz_common::properties::qtToOgre;

using rviz_rendering::Arrow;
using rviz_rendering::BillboardLine;
using rviz_rendering::MovableText;

using rviz_default_plugins::displays::InteractiveMarkerDisplay;
}  // namespace rviz

// The ROS 1 logging macros, on the logger of jsk_rviz_plugins; rviz
// plugins have no node of their own to log through, unlike the
// executables jsk_ros1_ros2_compat/logging.h is made for.
#ifndef ROS_DEBUG
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("jsk_rviz_plugins"), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("jsk_rviz_plugins"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("jsk_rviz_plugins"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("jsk_rviz_plugins"), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("jsk_rviz_plugins"), __VA_ARGS__)
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("jsk_rviz_plugins"), msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(rclcpp::get_logger("jsk_rviz_plugins"), msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(rclcpp::get_logger("jsk_rviz_plugins"), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("jsk_rviz_plugins"), msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(rclcpp::get_logger("jsk_rviz_plugins"), msg)
#endif

// `#if ROS_VERSION_MINIMUM(1,12,0)` (std:: instead of boost::shared_ptr,
// since kinetic) is true for any ROS 2.
#ifndef ROS_VERSION_MINIMUM
#define ROS_VERSION_MINIMUM(major, minor, patch) 1
#endif

// ros::WallTime / ros::WallDuration, the system wall clock (not the ROS
// clock) the displays animate with, as far as jsk_rviz_plugins uses them;
// rclcpp has no wall clock type of its own. As ros::param::param() of
// jsk_ros1_ros2_compat/logging.h, the ROS 1 names are reopened here so
// that the code using them stays the same.
namespace ros
{
class WallDuration
{
public:
  WallDuration(double sec = 0.0) : sec_(sec) {}
  double toSec() const { return sec_; }
private:
  double sec_;
};

class WallTime
{
public:
  WallTime() : sec_(0.0) {}
  static WallTime now()
  {
    WallTime t;
    t.sec_ = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    return t;
  }
  double toSec() const { return sec_; }
  WallTime operator+(const WallDuration& d) const
  {
    WallTime t;
    t.sec_ = sec_ + d.toSec();
    return t;
  }
  WallDuration operator-(const WallTime& t) const { return WallDuration(sec_ - t.sec_); }
private:
  double sec_;
};
}  // namespace ros
#endif  // ROS_VERSION_MAJOR != 1

#endif  // JSK_RVIZ_PLUGINS_RVIZ_COMPAT_H_
