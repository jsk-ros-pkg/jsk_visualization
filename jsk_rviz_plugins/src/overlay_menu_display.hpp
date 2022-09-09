// -*- mode: c++; -*-
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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <QPainter>
#include <jsk_rviz_plugin_msgs/msg/overlay_menu.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "overlay_utils.hpp"
#endif

namespace jsk_rviz_plugins
{
class OverlayMenuDisplay
: public rviz_common::RosTopicDisplay<jsk_rviz_plugin_msgs::msg::OverlayMenu>
{
  Q_OBJECT
public:
  OverlayMenuDisplay();
  ~OverlayMenuDisplay();

  enum AnimationState {
    CLOSED,
    OPENED,
    OPENING,
    CLOSING,
  };

  // methods for OverlayPickerTool
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  void setPosition(int x, int y);
  int getX() { return left_; };
  int getY() { return top_; };

protected:
  std::mutex mutex_;
  OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::BoolProperty * keep_centered_property_;
  rviz_common::properties::BoolProperty * overtake_fg_color_properties_property_;
  rviz_common::properties::BoolProperty * overtake_bg_color_properties_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  AnimationState animation_state_;
  bool require_update_texture_;
  bool keep_centered_;
  int left_, top_;
  jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr current_menu_;
  jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr next_menu_;
  double animation_t_;
  bool overtake_fg_color_properties_;
  bool overtake_bg_color_properties_;
  QColor bg_color_;
  QColor fg_color_;

  void prepareOverlay();
  void openingAnimation();
  std::string getMenuString(
    jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr msg, size_t index);
  QFont font();
  QFontMetrics fontMetrics();
  int drawAreaWidth(jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr msg);
  int drawAreaHeight(jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr msg);
  bool isNeedToResize();
  bool isNeedToRedraw();
  void redraw();
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void processMessage(jsk_rviz_plugin_msgs::msg::OverlayMenu::ConstSharedPtr msg);
  void setMenuLocation();
protected Q_SLOTS:
  void updateTopic();
  void updateLeft();
  void updateTop();
  void updateKeepCentered();
  void updateOvertakeFGColorProperties();
  void updateOvertakeBGColorProperties();
  void updateFGColor();
  void updateFGAlpha();
  void updateBGColor();
  void updateBGAlpha();
};

}  // namespace jsk_rviz_plugins

#endif
