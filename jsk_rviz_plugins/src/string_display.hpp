// -*- mode: c++; -*-
// Copyright (c) 2019, JSK Lab
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

#ifndef JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreColourValue.h>
#include <OgreMaterial.h>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "overlay_utils.hpp"
#include "visibility_control.hpp"
#endif

namespace jsk_rviz_plugins
{
class StringDisplay : public rviz_common::RosTopicDisplay<std_msgs::msg::String>
{
  Q_OBJECT
public:
  StringDisplay();
  ~StringDisplay();
  // methods for OverlayPickerTool
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  void setPosition(int x, int y);
  int getX() { return left_; }
  int getY() { return top_; }

protected:
  OverlayObject::Ptr overlay_;

  int texture_width_;
  int texture_height_;

  bool overtake_color_properties_;
  bool overtake_position_properties_;
  bool align_bottom_;
  QColor bg_color_;
  QColor fg_color_;
  int text_size_;
  int line_width_;
  std::string text_;
  QStringList font_families_;
  std::string font_;
  int left_;
  int top_;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  bool require_update_texture_;
  rviz_common::properties::BoolProperty * overtake_position_properties_property_;
  rviz_common::properties::BoolProperty * overtake_color_properties_property_;
  rviz_common::properties::BoolProperty * align_bottom_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * text_size_property_;
  rviz_common::properties::IntProperty * line_width_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  rviz_common::properties::EnumProperty * font_property_;
protected Q_SLOTS:
  void updateTopic();
  void updateOvertakePositionProperties();
  void updateOvertakeColorProperties();
  void updateAlignBottom();
  void updateTop();
  void updateLeft();
  void updateWidth();
  void updateHeight();
  void updateTextSize();
  void updateFGColor();
  void updateFGAlpha();
  void updateBGColor();
  void updateBGAlpha();
  void updateFont();
  void updateLineWidth();

private:
  void processMessage(std_msgs::msg::String::ConstSharedPtr msg) override;
};
}  // namespace jsk_rviz_plugins

#endif  // JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_
