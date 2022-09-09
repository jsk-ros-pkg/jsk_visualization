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

#include <QApplication>
#include <QMenu>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

// https://github.com/ros2/rviz/pull/767
//#include <rviz_common/tool_manager.hpp>
// #include <tool_manager.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/view_manager.hpp>
// #include <rviz_default_plugins/visibility_control.hpp>

#include <iomanip>

#include "linear_gauge_display.hpp"
#include "overlay_diagnostic_display.hpp"
#include "overlay_image_display.hpp"
#include "overlay_menu_display.hpp"
#include "overlay_picker_tool.hpp"
#include "overlay_text_display.hpp"
#include "pie_chart_display.hpp"
#include "plotter_2d_display.hpp"
#include "rviz_utils.hpp"

namespace jsk_rviz_plugins
{
OverlayPickerTool::OverlayPickerTool()
: is_moving_(false), shift_pressing_(false), rviz_common::Tool()
{
}

// int OverlayPickerTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel)
// {
//   if (event->type() == QEvent::KeyPress && event->key() == 16777248) { // sift
//     shift_pressing_ = true;
//   }
//   else if (event->type() == QEvent::KeyRelease && event->key() == 16777248) {
//     shift_pressing_ = false;
//   }
//   return 0;
// }

int OverlayPickerTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.left() && event.leftDown()) {
    if (!is_moving_) {
      onClicked(event);
    }
  } else if (event.left() && is_moving_) {
    onMove(event);
  } else if (is_moving_ && !(event.left() && event.leftDown())) {
    onRelease(event);
  }
  return 0;
}

bool OverlayPickerTool::handleDisplayClick(
  rviz_common::properties::Property * property, rviz_common::ViewportMouseEvent & event)
{
  if (isPropertyType<rviz_common::DisplayGroup>(property)) {
    rviz_common::DisplayGroup * group_property =
      isPropertyType<rviz_common::DisplayGroup>(property);
    for (int i = 0; i < group_property->numChildren(); i++) {
      if (handleDisplayClick(group_property->childAt(i), event)) {
        return true;
      }
    }
  } else {
    if (startMovement<OverlayTextDisplay>(property, event, "overlay_text_display")) {
      return true;
    } else if (startMovement<Plotter2DDisplay>(property, event, "plotter_2d_display")) {
      return true;
    } else if (startMovement<PieChartDisplay>(property, event, "pie_chart_display")) {
      return true;
    } else if (startMovement<OverlayImageDisplay>(property, event, "overlay_image_display")) {
      return true;
    } else if (startMovement<OverlayDiagnosticDisplay>(
                 property, event, "overlay_diagnostic_display")) {
      return true;
    } else if (startMovement<OverlayMenuDisplay>(property, event, "overlay_menu_display")) {
      return true;
    } else if (startMovement<LinearGaugeDisplay>(property, event, "linear_gauge_display")) {
      return true;
    } else {
      return false;
    }
  }

  return false;
}

void OverlayPickerTool::onClicked(rviz_common::ViewportMouseEvent & event)
{
  JSK_LOG_DEBUG("onClicked");
  is_moving_ = true;
  JSK_LOG_DEBUG("clicked: (%d, %d)", event.x, event.y);
  // check the active overlay plugin
  rviz_common::DisplayGroup * display_group = context_->getRootDisplayGroup();
  handleDisplayClick(display_group, event);
}

void OverlayPickerTool::onMove(rviz_common::ViewportMouseEvent & event)
{
  JSK_LOG_DEBUG("onMove");
  JSK_LOG_DEBUG("moving: (%d, %d)", event.x, event.y);
  if (target_property_) {
    if (target_property_type_ == "overlay_text_display") {
      movePosition<OverlayTextDisplay>(event);
    } else if (target_property_type_ == "plotter_2d_display") {
      movePosition<Plotter2DDisplay>(event);
    } else if (target_property_type_ == "pie_chart_display") {
      movePosition<PieChartDisplay>(event);
    } else if (target_property_type_ == "overlay_image_display") {
      movePosition<OverlayImageDisplay>(event);
    } else if (target_property_type_ == "overlay_diagnostic_display") {
      movePosition<OverlayDiagnosticDisplay>(event);
    } else if (target_property_type_ == "overlay_menu_display") {
      movePosition<OverlayMenuDisplay>(event);
    } else if (target_property_type_ == "linear_gauge_display") {
      movePosition<LinearGaugeDisplay>(event);
    }
  }
}

void OverlayPickerTool::onRelease(rviz_common::ViewportMouseEvent & event)
{
  JSK_LOG_DEBUG("onRelease");
  is_moving_ = false;
  JSK_LOG_DEBUG("released: (%d, %d)", event.x, event.y);
  if (target_property_) {
    if (target_property_type_ == "overlay_text_display") {
      setPosition<OverlayTextDisplay>(event);
    } else if (target_property_type_ == "plotter_2d_display") {
      setPosition<Plotter2DDisplay>(event);
    } else if (target_property_type_ == "pie_chart_display") {
      setPosition<PieChartDisplay>(event);
    } else if (target_property_type_ == "overlay_image_display") {
      setPosition<OverlayImageDisplay>(event);
    } else if (target_property_type_ == "overlay_diagnostic_display") {
      setPosition<OverlayDiagnosticDisplay>(event);
    } else if (target_property_type_ == "overlay_menu_display") {
      setPosition<OverlayMenuDisplay>(event);
    } else if (target_property_type_ == "linear_gauge_display") {
      setPosition<LinearGaugeDisplay>(event);
    }
  }
  // clear cache
  target_property_ = NULL;
  target_property_type_ = "";
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::OverlayPickerTool, rviz_common::Tool)
