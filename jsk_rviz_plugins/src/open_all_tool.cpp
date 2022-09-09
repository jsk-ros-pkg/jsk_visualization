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

// #include <ros/ros.h>
#include "open_all_tool.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/view_manager.hpp>

namespace jsk_rviz_plugins
{
OpenAllTool::OpenAllTool() : rviz_common::Tool() {}

OpenAllTool::~OpenAllTool() {}

void OpenAllTool::onInitialize() {}

void OpenAllTool::openProperty(rviz_common::properties::Property * property)
{
  property->expand();
  if (property->numChildren() > 0) {
    for (size_t i = 0; i < property->numChildren(); i++) {
      openProperty(property->childAt(i));
    }
    context_->queueRender();
  }
}

void OpenAllTool::activate()
{
  rviz_common::DisplayGroup * display_group = context_->getRootDisplayGroup();
  openProperty(display_group);
  // rviz_common::ToolManager* tool_manager = context_->getToolManager();
  // tool_manager->setCurrentTool(tool_manager->getTool(0));
}

void OpenAllTool::deactivate() {}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::OpenAllTool, rviz_common::Tool)
