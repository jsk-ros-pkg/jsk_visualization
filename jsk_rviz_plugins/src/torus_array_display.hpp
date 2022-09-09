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

#ifndef JSK_RVIZ_PLUGINS_TORUS_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_TORUS_ARRAY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <jsk_recognition_msgs/msg/torus_array.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/arrow.hpp>

#include "mesh_shape.hpp"
#include "visibility_control.hpp"
#endif

namespace jsk_rviz_plugins
{
struct Triangle
{
  unsigned v1, v2, v3;  // index for the 3 vertices that make up a triangle
};

class TorusArrayDisplay : public rviz_common::RosTopicDisplay<jsk_recognition_msgs::msg::TorusArray>
{
  Q_OBJECT
public:
  typedef std::shared_ptr<rviz_rendering::Arrow> ArrowPtr;
  typedef std::shared_ptr<MeshShape> ShapePtr;

  TorusArrayDisplay();
  ~TorusArrayDisplay();

protected:
  void onInitialize() override;
  void reset();
  void allocateShapes(jsk_recognition_msgs::msg::TorusArray::ConstSharedPtr msg);
  void allocateShapes(const size_t num);
  QColor getColor(size_t index);
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * uv_property_;
  rviz_common::properties::BoolProperty * auto_color_property_;
  rviz_common::properties::BoolProperty * show_normal_property_;
  rviz_common::properties::FloatProperty * normal_length_property_;
  QColor color_;
  double alpha_;
  bool auto_color_;
  bool show_normal_;
  double normal_length_;
  int uv_dimension_;
  std::vector<Ogre::SceneNode *> arrow_nodes_;
  std::vector<ArrowPtr> arrow_objects_;
  std::vector<ShapePtr> shapes_;
private Q_SLOTS:
  void updateColor();
  void updateAlpha();
  void updateUVdimension();
  void updateAutoColor();
  void updateShowNormal();
  void updateNormalLength();

  void calcurateTriangleMesh(
    int large_dimension, int small_dimension, float large_radius, float small_radius,
    Ogre::Vector3 pos, Ogre::Quaternion q, std::vector<Triangle> & triangles,
    std::vector<Ogre::Vector3> & vertices, std::vector<Ogre::Vector3> & normals);

private:
  void processMessage(jsk_recognition_msgs::msg::TorusArray::ConstSharedPtr msg) override;
};

}  // namespace jsk_rviz_plugins
#endif
