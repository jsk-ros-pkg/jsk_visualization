// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "overlay_picker_tool.h"

#include <string>

#include <QApplication>
#include <QKeyEvent>
#include <QMenu>
#include <QTimer>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>

// OverlayText and PieChart are provided by rviz_2d_overlay_plugins in ROS 2,
// so only the displays living in this package are pickable here.
#include "linear_gauge_display.h"
#include "overlay_diagnostic_display.h"
#include "overlay_image_display.h"
#include "overlay_menu_display.h"
#include "plotter_2d_display.h"
#include "string_display.h"

namespace jsk_rviz_plugins
{
  static rclcpp::Logger logger()
  {
    return rclcpp::get_logger("OverlayPickerTool");
  }

  OverlayPickerTool::OverlayPickerTool()
    : rviz_common::Tool(), is_moving_(false), target_property_(NULL),
      move_offset_x_(0), move_offset_y_(0), shift_pressing_(false)
  {

  }

  int OverlayPickerTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* /*panel*/)
  {
    if (event->type() == QEvent::KeyPress && event->key() == Qt::Key_Shift) { // shift
      shift_pressing_ = true;
    }
    else if (event->type() == QEvent::KeyRelease && event->key() == Qt::Key_Shift) {
      shift_pressing_ = false;
    }
    return 0;
  }


  int OverlayPickerTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
  {
    if (event.left() && event.leftDown()) {
      if (!is_moving_) {
        onClicked(event);
      }
    }
    else if (event.left() && is_moving_) {
      onMove(event);
    }
    else if (is_moving_ && !(event.left() && event.leftDown())) {
      onRelease(event);
    }
    return 0;
  }

  bool OverlayPickerTool::handleDisplayClick(rviz_common::properties::Property* property,
                                             rviz_common::ViewportMouseEvent& event)
  {
    if (isPropertyType<rviz_common::DisplayGroup>(property)) {
      rviz_common::DisplayGroup* group_property = isPropertyType<rviz_common::DisplayGroup>(property);
      for (int i = 0; i < group_property->numChildren(); i++) {
        if (handleDisplayClick(group_property->childAt(i), event)) {
          return true;
        }
      }
    }
    else {
      if (startMovement<StringDisplay>(property, event, "string_display")) {
        return true;
      }
      else if (startMovement<Plotter2DDisplay>(property, event, "plotter_2d_display")) {
        return true;
      }
      else if (startMovement<LinearGaugeDisplay>(property, event, "linear_gauge_display")) {
        return true;
      }
      else if (startMovement<OverlayImageDisplay>(property, event, "overlay_image_display")) {
        return true;
      }
      else if (startMovement<OverlayDiagnosticDisplay>(property, event, "overlay_diagnostic_display")) {
        return true;
      }
      else if (startMovement<OverlayMenuDisplay>(property, event, "overlay_menu_display")) {
        return true;
      }
      else {
        return false;
      }
    }
    return false;
  }

  void OverlayPickerTool::onClicked(rviz_common::ViewportMouseEvent& event)
  {
    RCLCPP_DEBUG(logger(), "onClicked");
    is_moving_ = true;
    RCLCPP_DEBUG(logger(), "clicked: (%d, %d)", event.x, event.y);
    // check the active overlay plugin
    rviz_common::DisplayGroup* display_group = context_->getRootDisplayGroup();
    handleDisplayClick(display_group, event);
  }

  void OverlayPickerTool::onMove(rviz_common::ViewportMouseEvent& event)
  {
    RCLCPP_DEBUG(logger(), "onMove");
    RCLCPP_DEBUG(logger(), "moving: (%d, %d)", event.x, event.y);
    if (target_property_) {
      if (target_property_type_ == "string_display") {
        movePosition<StringDisplay>(event);
      }
      else if (target_property_type_ == "plotter_2d_display") {
        movePosition<Plotter2DDisplay>(event);
      }
      else if (target_property_type_ == "linear_gauge_display") {
        movePosition<LinearGaugeDisplay>(event);
      }
      else if (target_property_type_ == "overlay_image_display") {
        movePosition<OverlayImageDisplay>(event);
      }
      else if (target_property_type_ == "overlay_diagnostic_display") {
        movePosition<OverlayDiagnosticDisplay>(event);
      }
      else if (target_property_type_ == "overlay_menu_display") {
        movePosition<OverlayMenuDisplay>(event);
      }
    }
  }

  void OverlayPickerTool::onRelease(rviz_common::ViewportMouseEvent& event)
  {
    RCLCPP_DEBUG(logger(), "onRelease");
    is_moving_ = false;
    RCLCPP_DEBUG(logger(), "released: (%d, %d)", event.x, event.y);
    if (target_property_) {
      if (target_property_type_ == "string_display") {
        setPosition<StringDisplay>(event);
      }
      else if (target_property_type_ == "plotter_2d_display") {
        setPosition<Plotter2DDisplay>(event);
      }
      else if (target_property_type_ == "linear_gauge_display") {
        setPosition<LinearGaugeDisplay>(event);
      }
      else if (target_property_type_ == "overlay_image_display") {
        setPosition<OverlayImageDisplay>(event);
      }
      else if (target_property_type_ == "overlay_diagnostic_display") {
        setPosition<OverlayDiagnosticDisplay>(event);
      }
      else if (target_property_type_ == "overlay_menu_display") {
        setPosition<OverlayMenuDisplay>(event);
      }
    }
    // clear cache
    target_property_ = NULL;
    target_property_type_ = "";
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayPickerTool, rviz_common::Tool )
