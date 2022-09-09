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

#ifndef JSK_RVIZ_PLUGIN_OVERLAY_IMAGE_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_IMAGE_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <QPainter>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "visibility_control.hpp"

//#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>

#include "overlay_utils.hpp"
//#include <image_transport_hints_property.hpp>
#endif

namespace jsk_rviz_plugins
{
class OverlayImageDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT
public:
  OverlayImageDisplay();
  ~OverlayImageDisplay();

  // methods for OverlayPickerTool
  bool isInRegion(int x, int y);
  void movePosition(int x, int y);
  void setPosition(int x, int y);
  int getX() { return left_; };
  int getY() { return top_; };

protected:
  std::mutex mutex_;
  OverlayObject::Ptr overlay_;
  //ImageTransportHintsProperty* transport_hint_property_;
  rviz_common::properties::BoolProperty * keep_aspect_ratio_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * overwrite_alpha_property_;
  int width_, height_, left_, top_;
  double alpha_;

  //sensor_msgs::msg::Image::ConstSharedPtr it_;
  //image_transport::Subscriber sub_;
  sensor_msgs::msg::Image::ConstSharedPtr msg_;
  bool is_msg_available_;
  bool require_update_;
  bool keep_aspect_ratio_;
  bool overwrite_alpha_;

  void redraw();
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void setImageSize();
  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg);
protected Q_SLOTS:
  void updateTopic();
  void updateWidth();
  void updateHeight();
  void updateLeft();
  void updateTop();
  void updateAlpha();
  void updateKeepAspectRatio();
  void updateOverwriteAlpha();
};
}  // namespace jsk_rviz_plugins

#endif
