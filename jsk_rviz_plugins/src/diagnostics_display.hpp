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

#ifndef JSK_RVIZ_PLUGIN_DIAGNOSTICS_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_DIAGNOSTICS_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
// #include <rviz_default_plugins/visibility_control.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
//#include <rviz_common/frame_manager.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_rendering/objects/movable_text.hpp>
#endif

namespace jsk_rviz_plugins
{
class DiagnosticsDisplay
: public rviz_common::RosTopicDisplay<diagnostic_msgs::msg::DiagnosticArray>
{
  Q_OBJECT
public:
  DiagnosticsDisplay();
  ~DiagnosticsDisplay();

protected:
  void onEnable() override;
  void onDisable() override;
  void onInitialize() override;
  void updateLine();
  void update(float wall_dt, float ros_dt) override;
  void processMessage(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);

  rviz_common::properties::EditableEnumProperty * diagnostics_namespace_property_;
  rviz_common::properties::TfFrameProperty * frame_id_property_;
  rviz_common::properties::FloatProperty * radius_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::FloatProperty * font_size_property_;
  rviz_common::properties::EnumProperty * axis_property_;

  double radius_;
  double line_width_;
  std::string frame_id_;
  std::string diagnostics_namespace_;
  rviz_rendering::MovableText * msg_;
  rviz_rendering::BillboardLine * line_;
  Ogre::SceneNode * orbit_node_;
  std::set<std::string> namespaces_;
  int axis_;
  double orbit_theta_;
  double font_size_;
  bool line_update_required_;
protected Q_SLOTS:
  void updateRosTopic();
  void updateDiagnosticsNamespace();
  void updateRadius();
  void updateLineWidth();
  void updateAxis();
  void updateFontSize();
  void fillNamespaceList();

private:
};

}  // namespace jsk_rviz_plugins

#endif
