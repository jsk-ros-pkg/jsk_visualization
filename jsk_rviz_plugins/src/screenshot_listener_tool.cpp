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

#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <QGuiApplication>
#include <QScreen>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#else
#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#endif
#include <QImageWriter>
#include "screenshot_listener_tool.h"

namespace jsk_rviz_plugins
{
  ScreenshotListenerTool::ScreenshotListenerTool()
    : rviz::Tool()
  {

  }
  ScreenshotListenerTool::~ScreenshotListenerTool()
  {

  }

  void ScreenshotListenerTool::onInitialize()
  {
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    screenshot_service_ = node->create_service<jsk_rviz_plugins::Screenshot>(
      "/rviz/screenshot",
      std::bind(&ScreenshotListenerTool::takeScreenShot, this,
                std::placeholders::_1, std::placeholders::_2));
#else
    ros::NodeHandle nh;
    screenshot_service_ = nh.advertiseService(
      "/rviz/screenshot",
      &ScreenshotListenerTool::takeScreenShot, this);
#endif
  }
  
  void ScreenshotListenerTool::activate()
  {
    
  }

  void ScreenshotListenerTool::deactivate()
  {
   
  }

#if ROS_VERSION_MAJOR != 1
  void ScreenshotListenerTool::takeScreenShot(
    const std::shared_ptr<jsk_rviz_plugins::Screenshot::Request> req,
    std::shared_ptr<jsk_rviz_plugins::Screenshot::Response> /*res*/)
  {
    // the 3D view is a window of its own (rviz_rendering::RenderWindow)
    QPixmap screenshot = QGuiApplication::primaryScreen()->grabWindow(
      context_->getViewManager()->getRenderPanel()->getRenderWindow()->winId());
    QString output_file = QString::fromStdString(req->file_name);
#else
  bool ScreenshotListenerTool::takeScreenShot(
    jsk_rviz_plugins::Screenshot::Request& req,
    jsk_rviz_plugins::Screenshot::Response& res)
  {
    QPixmap screenshot = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
    QString output_file = QString::fromStdString(req.file_name);
#endif
    QImageWriter writer(output_file);
    writer.write(screenshot.toImage());
#if ROS_VERSION_MAJOR == 1
    return true;
#endif
  }

}

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::ScreenshotListenerTool, rviz::Tool )
