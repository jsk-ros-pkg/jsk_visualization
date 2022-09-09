// -*- mode: c++; -*-
// Copyright (c) 2016, JSK Lab
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

#ifndef JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreSceneNode.h>

#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include "bounding_box_display_common.hpp"
#endif

namespace jsk_rviz_plugins
{
class BoundingBoxDisplay : public BoundingBoxDisplayCommon<jsk_recognition_msgs::msg::BoundingBox>
{
  Q_OBJECT
public:
  BoundingBoxDisplay();
  ~BoundingBoxDisplay();

protected:
  void onInitialize() override;
  void reset();

  bool only_edge_;
  bool show_coords_;
  // Properties
  rviz_common::properties::EnumProperty * coloring_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * only_edge_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::BoolProperty * show_coords_property_;

  jsk_recognition_msgs::msg::BoundingBox::ConstSharedPtr latest_msg_;
protected Q_SLOTS:
  void updateColor();
  void updateAlpha();
  void updateOnlyEdge();
  void updateColoring();
  void updateLineWidth();
  void updateShowCoords();

private:
  void processMessage(jsk_recognition_msgs::msg::BoundingBox::ConstSharedPtr msg);
};

}  // namespace jsk_rviz_plugins

#endif  // JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_H_
