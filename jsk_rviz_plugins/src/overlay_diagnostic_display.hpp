// -*- mode: c++; -*-
// Copyright (c) 2014, JSK Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// *
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

#ifndef JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <QPainter>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "overlay_utils.hpp"
#include "visibility_control.hpp"
#endif

namespace jsk_rviz_plugins
{
class OverlayDiagnosticDisplay
: public rviz_common::RosTopicDisplay<diagnostic_msgs::msg::DiagnosticArray>
{
  Q_OBJECT
public:
  OverlayDiagnosticDisplay();
  typedef enum { OK_STATE, ERROR_STATE, WARN_STATE, STALL_STATE } State;
  ~OverlayDiagnosticDisplay();

  // methods for OverlayPickerTool
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  void setPosition(int x, int y);
  int getX() { return left_; };
  int getY() { return top_; };

protected:
  bool isStalled();
  void processMessage(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;
  void onInitialize() override;
  void redraw();
  State getLatestState();
  void drawSAC(QImage & Hud);
  void drawEVA(QImage & Hud);
  void drawEVAConnectedRectangle(QPainter & painter, QColor color, QColor small_color, int width);
  void drawEVANonConnectedRectangle(
    QPainter & painter, QColor color, QColor small_color, int width, double gap);
  void fillNamespaceList();
  QColor foregroundColor();
  QColor textColor();
  double textWidth(QPainter & painter, double font_size, const std::string & text);
  double textHeight(QPainter & painter, double font_size);
  QColor blendColor(QColor a, QColor b, double a_rate);
  void drawText(QPainter & painter, QColor fg_color, const std::string & text);
  double drawAnimatingText(
    QPainter & painter, QColor fg_color, const double height, const double font_size,
    const std::string text);
  // return true if plugin needs to animate
  bool isAnimating();
  double animationRate();
  std::string statusText();
  std::mutex mutex_;
  OverlayObject::Ptr overlay_;

  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> latest_status_;

  State previous_state_;
  //ros::WallTime latest_message_time_;
  rclcpp::Time latest_message_time_;
  //ros::WallTime animation_start_time_;
  rclcpp::Time animation_start_time_;
  rclcpp::Clock clock_;
  int size_;
  std::string diagnostics_namespace_;
  int type_;
  std::set<std::string> namespaces_;
  double alpha_;
  int top_, left_;
  double t_;
  double stall_duration_;
  bool is_animating_;
  rviz_common::properties::EditableEnumProperty * diagnostics_namespace_property_;
  rviz_common::properties::EnumProperty * type_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * size_property_;
  rviz_common::properties::FloatProperty * stall_duration_property_;

protected Q_SLOTS:
  void updateType();
  void updateRosTopic();
  void updateDiagnosticsNamespace();
  void updateSize();
  void updateAlpha();
  void updateTop();
  void updateLeft();
  void updateStallDuration();

private:
};
}  // namespace jsk_rviz_plugins

#endif
