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
#include "normal_visual.hpp"

#include <iomanip>

namespace jsk_rviz_plugins
{
NormalVisual::NormalVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  normal_arrow_.reset(new rviz_rendering::Arrow(scene_manager_, frame_node_));
}

NormalVisual::~NormalVisual() { scene_manager_->destroySceneNode(frame_node_); }

void NormalVisual::setValues(
  float x, float y, float z, float normal_x, float normal_y, float normal_z)
{
  Ogre::Vector3 dir(normal_x, normal_y, normal_z);
  Ogre::Vector3 pos(x, y, z);

  float length = dir.length() / 10;

  Ogre::Vector3 scale(length, length, length);
  normal_arrow_->setScale(scale);
  normal_arrow_->setDirection(dir);
  normal_arrow_->setPosition(pos);
}

void NormalVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void NormalVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

void NormalVisual::setColor(float r, float g, float b, float a)
{
  normal_arrow_->setColor(r, g, b, a);
}

void NormalVisual::setScale(float scale)
{
  normal_arrow_->setScale(Ogre::Vector3(scale, scale, scale));
}

}  // namespace jsk_rviz_plugins
