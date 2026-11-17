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
#endif

#endif  // JSK_INTERACTIVE_MARKER_ROS1_ROS2_COMPAT_H_
