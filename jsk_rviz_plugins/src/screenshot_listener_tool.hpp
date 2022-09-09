// -*- mode: c++ -*-
// Copyright (c) 2014, JSK Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the JSK Lab nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef JSK_RVIZ_PLUGIN_SCREENSHOT_LISTENER_H_
#define JSK_RVIZ_PLUGIN_SCREENSHOT_LISTENER_H_

#include <jsk_rviz_plugin_msgs/srv/screenshot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/tool.hpp>

namespace jsk_rviz_plugins
{
class ScreenshotListenerTool : public rviz_common::Tool  //, rclcpp::Node
{
public:
  ScreenshotListenerTool();
  virtual ~ScreenshotListenerTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

protected:
  virtual bool takeScreenShot(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<jsk_rviz_plugin_msgs::srv::Screenshot::Request> req,
    const std::shared_ptr<jsk_rviz_plugin_msgs::srv::Screenshot::Response> res);
  // jsk_rviz_plugins::Screenshot::Request& req,
  // jsk_rviz_plugins::Screenshot::Response& res);

  //ros::ServiceServer screenshot_service_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Service<jsk_rviz_plugin_msgs::srv::Screenshot>::SharedPtr screenshot_service_;

private:
};
}  // namespace jsk_rviz_plugins

#endif
