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
#ifndef JSK_RVIZ_PLUGIN_PLOTTER_2D_H_
#define JSK_RVIZ_PLUGIN_PLOTTER_2D_H_

#include <std_msgs/msg/float32.hpp>
#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
// #include <rviz_default_plugins/visibility_control.hpp>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include "overlay_utils.hpp"
#endif

namespace jsk_rviz_plugins
{
class Plotter2DDisplay : public rviz_common::RosTopicDisplay<std_msgs::msg::Float32>
{
  Q_OBJECT
public:
  Plotter2DDisplay();
  ~Plotter2DDisplay();
  // methods for OverlayPickerTool
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  void setPosition(int x, int y);
  int getX() { return left_; };
  int getY() { return top_; };

protected:
  ////////////////////////////////////////////////////////
  // methods
  ////////////////////////////////////////////////////////
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;
  void initializeBuffer();
  void onInitialize() override;
  void processMessage(std_msgs::msg::Float32::ConstSharedPtr msg);
  void drawPlot();
  ////////////////////////////////////////////////////////
  // properties
  ////////////////////////////////////////////////////////
  rviz_common::properties::BoolProperty * show_value_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::FloatProperty * update_interval_property_;
  rviz_common::properties::BoolProperty * show_border_property_;
  rviz_common::properties::IntProperty * buffer_length_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::IntProperty * line_width_property_;
  rviz_common::properties::BoolProperty * auto_color_change_property_;
  rviz_common::properties::ColorProperty * max_color_property_;
  rviz_common::properties::BoolProperty * show_caption_property_;
  rviz_common::properties::IntProperty * text_size_property_;
  rviz_common::properties::BoolProperty * auto_scale_property_;
  rviz_common::properties::FloatProperty * max_value_property_;
  rviz_common::properties::FloatProperty * min_value_property_;

  OverlayObject::Ptr overlay_;
  QColor fg_color_;
  QColor max_color_;
  QColor bg_color_;

  double fg_alpha_;
  double bg_alpha_;
  bool auto_scale_;
  bool show_border_;
  bool auto_color_change_;
  bool show_value_;
  bool show_caption_;
  bool draw_required_;
  float last_time_;
  float update_interval_;

  int buffer_length_;
  std::vector<double> buffer_;
  uint16_t texture_width_;
  uint16_t texture_height_;
  int left_;
  int top_;
  int line_width_;
  int text_size_;
  int caption_offset_;
  double min_value_;
  double max_value_;

  ////////////////////////////////////////////////////////
  // ROS variables
  ////////////////////////////////////////////////////////
  std::mutex mutex_;

protected Q_SLOTS:
  void updateTopic();
  void updateShowValue();
  void updateBufferSize();
  void updateBGColor();
  void updateFGColor();
  void updateFGAlpha();
  void updateBGAlpha();
  void updateWidth();
  void updateHeight();
  void updateTop();
  void updateLeft();
  void updateLineWidth();
  void updateShowBorder();
  void updateAutoColorChange();
  void updateMaxColor();
  void updateUpdateInterval();
  void updateShowCaption();
  void updateTextSize();
  void updateAutoScale();
  void updateMinValue();
  void updateMaxValue();

private:
};
}  // namespace jsk_rviz_plugins

#endif
