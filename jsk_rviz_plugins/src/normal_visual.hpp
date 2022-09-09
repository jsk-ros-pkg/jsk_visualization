// -*- mode:c++ -*-
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
#ifndef NORMAL_VISUAL_H
#define NORMAL_VISUAL_H

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <ros/ros.hpp>

namespace jsk_rviz_plugins
{
class NormalVisual
{
public:
  NormalVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  ~NormalVisual();

  void setValues(float x, float y, float z, float normal_x, float normal_y, float normal_z);
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);
  void setColor(float r, float g, float b, float a);
  void setScale(float scale);

private:
  std::shared_ptr<rviz_rendering::Arrow> normal_arrow_;

  Ogre::SceneNode * frame_node_;
  Ogre::SceneManager * scene_manager_;
};
}  // namespace jsk_rviz_plugins
#endif
