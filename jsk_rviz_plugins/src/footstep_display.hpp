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

#ifndef FOOTSTEP_DISPLAY_H
#define FOOTSTEP_DISPLAY_H

#include <jsk_footstep_msgs/msg/footstep_array.hpp>
#ifndef Q_MOC_RUN
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/shape.hpp>
#endif

namespace jsk_rviz_plugins
{
class FootstepDisplay : public rviz_common::RosTopicDisplay<jsk_footstep_msgs::msg::FootstepArray>
{
  Q_OBJECT
public:
  FootstepDisplay();
  ~FootstepDisplay();

protected:
  void onInitialize() override;
  void reset();
  void update(float wall_dt, float ros_dt) override;

private:
  void allocateCubes(size_t num);
  void allocateTexts(size_t num);
  double estimateTextSize(const jsk_footstep_msgs::msg::Footstep & footstep);
  double minNotZero(double a, double b);
  void processMessage(jsk_footstep_msgs::msg::FootstepArray::ConstSharedPtr msg) override;
  bool validateFloats(const jsk_footstep_msgs::msg::FootstepArray & msg);

  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::FloatProperty * height_property_;
  rviz_common::properties::FloatProperty * depth_property_;
  rviz_common::properties::BoolProperty * show_name_property_;
  rviz_common::properties::BoolProperty * use_group_coloring_property_;
  jsk_footstep_msgs::msg::FootstepArray::ConstSharedPtr latest_footstep_;
  typedef std::shared_ptr<rviz_rendering::Shape> ShapePtr;
  std::vector<ShapePtr> shapes_;
  std::vector<rviz_rendering::MovableText *> texts_;
  std::vector<Ogre::SceneNode *> text_nodes_;
  rviz_rendering::BillboardLine * line_;
  double width_, height_, depth_;
  double alpha_;
  bool show_name_;
  bool use_group_coloring_;
  //Ogre::SceneNode* scene_node_;
private Q_SLOTS:
  void updateAlpha();
  void updateWidth();
  void updateHeight();
  void updateDepth();
  void updateShowName();
  void updateUseGroupColoring();
};
}  // namespace jsk_rviz_plugins

#endif
