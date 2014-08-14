// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "facing_visualizer.h"
#include <rviz/uniform_string_stream.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <QPainter>

namespace jsk_rviz_plugin
{
  SquareObject::SquareObject(Ogre::SceneManager* manager,
                             double outer_radius,
                             double inner_radius,
                             std::string name):
    manager_(manager), outer_radius_(outer_radius),
    inner_radius_(inner_radius), name_(name)
  {
    manual_ = manager->createManualObject();
    rebuildPolygon();
  }

  SquareObject::~SquareObject()
  {
    manual_->detachFromParent();
    manager_->destroyManualObject(manual_);
  }

  Ogre::ManualObject* SquareObject::getManualObject()
  {
    return manual_;
  }

  void SquareObject::setOuterRadius(double outer_radius)
  {
    outer_radius_ = outer_radius;
  }

  void SquareObject::setInnerRadius(double inner_radius)
  {
    inner_radius_ = inner_radius;
  }

  void SquareObject::rebuildPolygon()
  {
    manual_->clear();
    manual_->begin(name_,
                   Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    const size_t resolution = 100;
    const double radius_ratio = inner_radius_ / outer_radius_;
    const double inner_offset = - outer_radius_ * 0.0;
    int counter = 0;
    for (size_t i = 0; i < resolution; i++) {
      double theta = 2.0 * M_PI / resolution * i;
      double next_theta = 2.0 * M_PI / resolution * (i + 1);
      
      manual_->position(inner_radius_ * cos(theta) + inner_offset,
                        inner_radius_ * sin(theta) + inner_offset,
                        0.0f);
      manual_->textureCoord((1 + radius_ratio *  cos(theta)) / 2.0, (1.0 - radius_ratio * sin(theta)) / 2.0);
      manual_->index(counter++);
      manual_->position(outer_radius_ * cos(theta),
                        outer_radius_ * sin(theta),
                        0.0f);
      manual_->textureCoord((1 + cos(theta)) / 2.0, (1.0 -sin(theta)) / 2.0);
      manual_->index(counter++);
      manual_->position(inner_radius_ * cos(next_theta) + inner_offset,
                        inner_radius_ * sin(next_theta) + inner_offset,
                        0.0f);
      manual_->textureCoord((1 + radius_ratio *  cos(next_theta)) / 2.0, (1.0 - radius_ratio * sin(next_theta)) / 2.0);
      manual_->index(counter++);
      manual_->position(outer_radius_ * cos(next_theta),
                        outer_radius_ * sin(next_theta),
                        0.0f);
      manual_->textureCoord((1 + cos(next_theta)) / 2.0, (1.0 -sin(next_theta)) / 2.0);
      manual_->index(counter++);
      
    }
    // for (size_t i = 0; i < resolution; i++) {
    // }
    // manual_->index(0);
    manual_->end();
  }
  
  TextureObject::TextureObject(const int width, const int height,
                               const std::string name):
    width_(width), height_(height), name_(name)
  {
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width, height,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DEFAULT
      );
    material_ = Ogre::MaterialManager::getSingleton().create(
      getMaterialName(), // name
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
 
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(
      texture_->getName());
    material_->getTechnique(0)->getPass(0)
      ->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
     // material_->getTechnique(0)->getPass(0)
     //   ->setSceneBlending(Ogre::SBT_MODULATE);

  }

  TextureObject::~TextureObject()
  {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
  }
  
  ScopedPixelBuffer TextureObject::getBuffer()
  {
    return ScopedPixelBuffer(texture_->getBuffer());
  }

  std::string TextureObject::getMaterialName()
  {
    return name_ + "Material";
  }

  FacingObject::FacingObject(Ogre::SceneManager* manager,
                             Ogre::SceneNode* parent,
                             double size):
    scene_manager_(manager), size_(size)
  {
    node_ = parent->createChildSceneNode();
  }

  FacingObject::~FacingObject()
  {
    node_->detachAllObjects();
    scene_manager_->destroySceneNode(node_);
  }
  
  void FacingObject::setPosition(Ogre::Vector3& pos)
  {
    node_->setPosition(pos);
  }
  
  void FacingObject::setOrientation(rviz::DisplayContext* context)
  {
    rviz::ViewManager* manager = context->getViewManager();
    rviz::RenderPanel* panel = manager->getRenderPanel();
    Ogre::Camera* camera = panel->getCamera();
    node_->setOrientation(camera->getDerivedOrientation());
  }

  void FacingObject::setSize(double size)
  {
    size_ = size;
  }
  
  FacingTexturedObject::FacingTexturedObject(Ogre::SceneManager* manager,
                                             Ogre::SceneNode* parent,
                                             double size):
    FacingObject(manager, parent, size)
  {
    rviz::UniformStringStream ss;
    static int count = 0;
    ss << "FacingVisualizer" << count++;
    texture_object_.reset(new TextureObject(128, 128, ss.str()));
    square_object_.reset(new SquareObject(manager, size, 0,
                                          texture_object_->getMaterialName()));
    node_->attachObject(square_object_->getManualObject());
  }
  

  void FacingTexturedObject::setSize(double size)
  {
    FacingObject::setSize(size);
    square_object_->setOuterRadius(size_);
    square_object_->rebuildPolygon();
  }

  GISCircleVisualizer::GISCircleVisualizer(Ogre::SceneManager* manager,
                                           Ogre::SceneNode* parent,
                                           double size,
                                           std::string text):
    FacingTexturedObject(manager, parent, size), text_(text)
  {

  }
  
  void GISCircleVisualizer::update(float wall_dt, float ros_dt)
  {
    ros::WallTime now = ros::WallTime::now();
    std::string text = text_ + " ";
    {
      ScopedPixelBuffer buffer = texture_object_->getBuffer();
      QColor transparent(0, 0, 0, 0);
      QColor foreground(33, 73, 140, 255);
      QColor white(255, 255, 255, 255);
      QImage Hud = buffer.getQImage(128, 128, transparent);
      double line_width = 5;
      double inner_line_width = 10;
      double l = 128;
      //double cx = l / 2 - line_width / 4.0;
      double cx = l / 2;
      //double cy = l / 2 - line_width / 4.0;
      double cy = l / 2;
      double r = 48;
      double inner_r = 40;
      double mouse_r = 30;
      double mouse_cy_offset = 5;
      
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
      painter.setBrush(white);
      painter.drawEllipse(line_width / 2.0, line_width / 2.0,
                          l - line_width, l - line_width);
      double offset_rate = fmod(now.toSec(), 10) / 10.0;
      double theta_offset = offset_rate * M_PI * 2.0;
      for (size_t ci = 0; ci < text.length(); ci++) {
        double theta = M_PI * 2.0 / text.length() * ci + theta_offset;
        painter.save();
        QFont font("DejaVu Sans Mono");
        font.setPointSize(10);
        font.setBold(true);
        painter.setFont(font);
        painter.translate(cx + r * cos(theta),
                          cy + r * sin(theta));
        painter.rotate(theta / M_PI * 180 + 90);
        painter.drawText(0, 0, text.substr(ci, 1).c_str());
        painter.restore();
      }
      painter.setPen(QPen(foreground, inner_line_width, Qt::SolidLine));
      painter.setBrush(transparent);
      painter.drawEllipse(cx - inner_r, cy - inner_r,
                          inner_r * 2, inner_r * 2);
      // if (!anonymous_) {
      //   // force to clear the inside of the circle
      //   for (unsigned int i = 0; i < Hud.width(); i++) {
      //     for (unsigned int j = 0; j < Hud.height(); j++) {
      //       double dist = sqrt((i - cx) * (i - cx) + (j - cy) * (j - cy));
      //       if (dist < inner_r) {
      //         Hud.setPixel(i, j, transparent.rgba());
      //       }
      //     }
      //   }
      // }
      // else {                    // overlay face ^^
        double mouse_c_x = cx;
        double mouse_c_y = cy - mouse_cy_offset;
        double start_angle = -25 * M_PI / 180;
        double end_angle = -125 * M_PI / 180;
        painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
        painter.drawChord(mouse_c_x - mouse_r, mouse_c_y - mouse_r,
                        2.0 * mouse_r, 2.0 * mouse_r,
                        start_angle * 180 / M_PI * 16,
                        end_angle * 180 / M_PI * 16);
        //}
      painter.end();
    }
  }

  void GISCircleVisualizer::setAnonymous(bool anonymous)
  {
    anonymous_ = anonymous;
    if (!anonymous_) {
      square_object_->setInnerRadius(size_ * 0.6);
    }
    else {
      square_object_->setInnerRadius(0.0);
      
    }
    square_object_->rebuildPolygon();
  }
  
}
