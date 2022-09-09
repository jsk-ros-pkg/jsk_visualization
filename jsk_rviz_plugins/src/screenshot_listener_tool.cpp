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

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <rviz/tool_manager.h>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QImageWriter>
#include "screenshot_listener_tool.hpp"

namespace jsk_rviz_plugins
{
  ScreenshotListenerTool::ScreenshotListenerTool()
    : rviz_common::Tool()//, Node("ScreenshotListenerTool")
  {

  }
  ScreenshotListenerTool::~ScreenshotListenerTool()
  {

  }

  void ScreenshotListenerTool::onInitialize()
  {
    //ros::NodeHandle nh;
    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    // screenshot_service_ = nh.advertiseService(
    //   "/rviz/screenshot",
    //   &ScreenshotListenerTool::takeScreenShot, this);
    RCLCPP_INFO(nh_->get_logger(), "create srv");
    using namespace std::placeholders;
    screenshot_service_ = nh_->create_service<jsk_rviz_plugin_msgs::srv::Screenshot>(
        "/rviz/screenshot",
        std::bind(&ScreenshotListenerTool::takeScreenShot, this, _1, _2, _3));
  }
  
  void ScreenshotListenerTool::activate()
  {
    RCLCPP_INFO(nh_->get_logger(), "activate");
  }

  void ScreenshotListenerTool::deactivate()
  {
    RCLCPP_INFO(nh_->get_logger(), "deactivate");
  }

  bool ScreenshotListenerTool::takeScreenShot(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<jsk_rviz_plugin_msgs::srv::Screenshot::Request> req,
    const std::shared_ptr<jsk_rviz_plugin_msgs::srv::Screenshot::Response> res)
  {
    RCLCPP_INFO(nh_->get_logger(), "take picture: " + req->file_name);
    //QPixmap screenshot = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
    // QPixmap screenshot = ->windowHandle()->screen()->grabWindow(context_->getViewManager()->getRenderPanel()->winId());
    // QString output_file = QString::fromStdString(req->file_name);
    // QImageWriter writer(output_file);
    // writer.write(screenshot.toImage());
    context_->getViewManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(req->file_name);
    return true;
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::ScreenshotListenerTool, rviz_common::Tool )
