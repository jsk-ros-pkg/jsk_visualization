// -*- mode: c++; -*-
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
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMaterialManager.h>
#include "target_visualizer_display.h"
#include <OGRE/OgreManualObject.h>

namespace jsk_rviz_plugin
{
  const float arrow_animation_duration = 1.0;
  const double minimum_font_size = 0.2;
  
  TargetVisualizerDisplay::TargetVisualizerDisplay()
    : facing_node_(NULL), target_text_node_(NULL), t_(0.0)
  {
    target_name_property_ = new rviz::StringProperty(
      "target name", "target",
      "name of the target",
      this, SLOT(updateTargetName())
      );
    radius_property_ = new rviz::FloatProperty(
      "radius", 1.0,
      "radius of the target mark",
      this, SLOT(updateRadius()));
    radius_property_->setMin(0.0);
    alpha_property_ = new rviz::FloatProperty(
      "alpha", 0.8,
      "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
    color_property_ = new rviz::ColorProperty(
      "color", QColor(25, 255, 240),
      "color of the target",
      this, SLOT(updateColor()));
  }

  TargetVisualizerDisplay::~TargetVisualizerDisplay()
  {
    delete target_name_property_;
    delete alpha_property_;
    delete color_property_;
    delete radius_property_;
    delete line_;
    delete text_under_line_;
    delete msg_;
    context_->getSceneManager()->destroyManualObject(upper_arrow_);
    context_->getSceneManager()->destroyManualObject(lower_arrow_);
    context_->getSceneManager()->destroyManualObject(left_arrow_);
    context_->getSceneManager()->destroyManualObject(right_arrow_);
    upper_material_->unload();
    lower_material_->unload();
    left_material_->unload();
    right_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(upper_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(lower_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(left_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(right_material_->getName());
  }

  void TargetVisualizerDisplay::updateArrowsObjects(Ogre::ColourValue color)
  {
    const double size_factor = 0.15;
    upper_arrow_node_->setPosition(Ogre::Vector3(0, radius_ * 1.0, 0.0));
    upper_arrow_->clear();
    upper_arrow_->estimateVertexCount(3);
    upper_arrow_->begin(upper_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(0, radius_ * size_factor, 0));
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(radius_ * size_factor,
                                         radius_ * size_factor * 2,
                                         0));
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(-radius_ * size_factor,
                                         radius_ * size_factor * 2,
                                         0));
    upper_arrow_->end();
    
    lower_arrow_node_->setPosition(Ogre::Vector3(0, -radius_ * 1.0, 0.0));
    lower_arrow_->clear();
    lower_arrow_->estimateVertexCount(3);
    lower_arrow_->begin(lower_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(0,
                                         -radius_ * size_factor,
                                         0));
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(radius_ * size_factor,
                                         -radius_ * size_factor * 2,
                                         0));
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(-radius_ * size_factor,
                                         -radius_ * size_factor * 2,
                                         0));
    lower_arrow_->end();
    left_arrow_node_->setPosition(Ogre::Vector3(radius_ * 1.0, 0.0, 0.0));
    left_arrow_->clear();
    left_arrow_->estimateVertexCount(3);
    left_arrow_->begin(left_material_name_,
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(radius_ * size_factor,
                                        0.0,
                                        0));
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(radius_ * size_factor * 2,
                                        radius_ * size_factor,
                                        0));
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(radius_ * size_factor * 2,
                                        - radius_ * size_factor,
                                        0));
    left_arrow_->end();
    
    right_arrow_node_->setPosition(Ogre::Vector3(-radius_ * 1.0, 0.0, 0.0));
    right_arrow_->clear();
    right_arrow_->estimateVertexCount(3);
    right_arrow_->begin(right_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-radius_ * size_factor,
                                         0.0,
                                         0));
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-radius_ * size_factor * 2,
                                         radius_ * size_factor,
                                         0));
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-radius_ * size_factor * 2,
                                         - radius_ * size_factor,
                                         0));
    right_arrow_->end();
    
    
    upper_material_->getTechnique(0)->setLightingEnabled(false);
    upper_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    upper_material_->getTechnique(0)->setDepthWriteEnabled( false );
    lower_material_->getTechnique(0)->setLightingEnabled(false);
    lower_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    lower_material_->getTechnique(0)->setDepthWriteEnabled( false );
    left_material_->getTechnique(0)->setLightingEnabled(false);
    left_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    left_material_->getTechnique(0)->setDepthWriteEnabled( false );
    right_material_->getTechnique(0)->setLightingEnabled(false);
    right_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    right_material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  
  void TargetVisualizerDisplay::processMessage(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    if (!isEnabled()) {
      return;
    }
    if (isEnabled()) {
      msg_->setVisible(true);
      upper_arrow_node_->setVisible(true);
      lower_arrow_node_->setVisible(true);
      left_arrow_node_->setVisible(true);
      right_arrow_node_->setVisible(true);
    }
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->transform(msg->header,
                                               msg->pose,
                                               position, orientation))
    {
      ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    
    // update line_
    if (require_update_line_) {
      require_update_line_ = false;
      const int resolution = 100;
      Ogre::ColourValue color(color_.red() / 255.0,
                              color_.green() / 255.0,
                              color_.blue() / 255.0,
                              alpha_);
      msg_->setColor(color);
      line_->clear();
      line_->setColor(color.r, color.g, color.b, color.a);
      
      line_->setLineWidth(0.1 * radius_);
      line_->setNumLines(1);
      line_->setMaxPointsPerLine(1024);
      for (size_t i = 0; i < resolution + 1; i++) {
        double x = radius_ * cos(i * 2 * M_PI / resolution);
        double y = radius_ * sin(i * 2 * M_PI / resolution);
        double z = 0;
        Ogre::Vector3 p;
        p[0] = x;
        p[1] = y;
        p[2] = z;
        line_->addPoint(p);
      }
      // update the positoin of the text
      Ogre::Vector3 text_position(radius_ * cos(45.0 / 180.0 * M_PI) + radius_ / 2.0,
                                  radius_ * sin(45.0 / 180.0 * M_PI) + radius_ / 2.0,
                                  0);
      target_text_node_->setPosition(text_position);
      Ogre::Vector3 msg_size = msg_->GetAABB().getSize();
      text_under_line_->clear();
      text_under_line_->setColor(color.r, color.g, color.b, color.a);
      
      text_under_line_->setLineWidth(0.01);
      text_under_line_->setNumLines(1);
      text_under_line_->setMaxPointsPerLine(1024);
      Ogre::Vector3 A(radius_ * cos(45.0 / 180.0 * M_PI),
                      radius_ * sin(45.0 / 180.0 * M_PI),
                      0);
      Ogre::Vector3 B(text_position + Ogre::Vector3(- radius_ / 4.0, 0, 0));
      Ogre::Vector3 C(text_position + Ogre::Vector3(msg_size[0], 0, 0));
      text_under_line_->addPoint(A);
      text_under_line_->addPoint(B);
      text_under_line_->addPoint(C);

      // update manual objects, arrows
      updateArrowsObjects(color);
    }
  }

  
  void TargetVisualizerDisplay::update(float wall_dt, float ros_dt)
  {
    rviz::ViewManager* manager = context_->getViewManager();
    rviz::RenderPanel* panel = manager->getRenderPanel();
    Ogre::Camera* camera = panel->getCamera();
    scene_node_->setOrientation(camera->getDerivedOrientation());

    msg_->setCharacterHeight(std::max(0.2 * radius_, minimum_font_size));
    msg_->setCaption(target_name_);
    t_ += wall_dt;
    double t_rate
      = fmod(t_, arrow_animation_duration) / arrow_animation_duration;
    upper_arrow_node_->setPosition(0, (1.3 - 0.3 * t_rate) * radius_, 0);
    lower_arrow_node_->setPosition(0, (-1.3 + 0.3 * t_rate) * radius_, 0);
    left_arrow_node_->setPosition((1.3 - 0.3 * t_rate) * radius_, 0, 0);
    right_arrow_node_->setPosition((-1.3 + 0.3 * t_rate) * radius_, 0, 0);
    if (t_rate > 1.0) {
      t_ = 0.0;
    }
  }

  void TargetVisualizerDisplay::createArrows()
  {
    static uint32_t count = 0;
    rviz::UniformStringStream ss;
    ss << "TargetVisualizerDisplayTriangle" << count++;
    ss << "Material";
    ss << "0";
    upper_material_name_ = std::string(ss.str());
    ss << "1";
    lower_material_name_ = std::string(ss.str());
    ss << "2";
    left_material_name_ = std::string(ss.str());
    ss << "3";
    right_material_name_ = std::string(ss.str());
    upper_material_ = Ogre::MaterialManager::getSingleton().create(
      upper_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    lower_material_ = Ogre::MaterialManager::getSingleton().create(
      lower_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    left_material_ = Ogre::MaterialManager::getSingleton().create(
      left_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    right_material_ = Ogre::MaterialManager::getSingleton().create(
      right_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    upper_material_->setReceiveShadows(false);
    upper_material_->getTechnique(0)->setLightingEnabled(true);
    upper_material_->setCullingMode(Ogre::CULL_NONE);
    lower_material_->setReceiveShadows(false);
    lower_material_->getTechnique(0)->setLightingEnabled(true);
    lower_material_->setCullingMode(Ogre::CULL_NONE);
    left_material_->setReceiveShadows(false);
    left_material_->getTechnique(0)->setLightingEnabled(true);
    left_material_->setCullingMode(Ogre::CULL_NONE);
    right_material_->setReceiveShadows(false);
    right_material_->getTechnique(0)->setLightingEnabled(true);
    right_material_->setCullingMode(Ogre::CULL_NONE);

    upper_arrow_ = context_->getSceneManager()->createManualObject(
      upper_material_name_);
    upper_arrow_node_ = facing_node_->createChildSceneNode();
    upper_arrow_node_->attachObject(upper_arrow_);
    lower_arrow_ = context_->getSceneManager()->createManualObject(
      lower_material_name_);
    lower_arrow_node_ = facing_node_->createChildSceneNode();
    lower_arrow_node_->attachObject(lower_arrow_);
    left_arrow_ = context_->getSceneManager()->createManualObject(
      left_material_name_);
    left_arrow_node_ = facing_node_->createChildSceneNode();
    left_arrow_node_->attachObject(left_arrow_);
    right_arrow_ = context_->getSceneManager()->createManualObject(
      right_material_name_);
    right_arrow_node_ = facing_node_->createChildSceneNode();
    right_arrow_node_->attachObject(right_arrow_);
  }
  
  void TargetVisualizerDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    //scene_node_->setVisible(false);
    facing_node_ = scene_node_->createChildSceneNode();
    //facing_node_->setVisible(false);
    target_text_node_ = facing_node_->createChildSceneNode();
    line_ = new rviz::BillboardLine(context_->getSceneManager(),
                                    facing_node_);
    text_under_line_ = new rviz::BillboardLine(context_->getSceneManager(),
                                               facing_node_);
    msg_ = new rviz::MovableText("not initialized", "Arial", 0.05);
    msg_->setVisible(false);
    msg_->setTextAlignment(rviz::MovableText::H_LEFT,
                           rviz::MovableText::V_ABOVE);
    //msg_->setVisible(false);
    target_text_node_->attachObject(msg_);
    updateTargetName();
    updateColor();
    updateAlpha();
    updateRadius();
    //facing_node_);
    // create manual object
    createArrows();
  }

  void TargetVisualizerDisplay::reset()
  {
    MFDClass::reset();
    line_->clear();
    text_under_line_->clear();
    require_update_line_ = true;
    msg_->setVisible(false);
    upper_arrow_node_->setVisible(false);
    lower_arrow_node_->setVisible(false);
    left_arrow_node_->setVisible(false);
    right_arrow_node_->setVisible(false);
  }

  void TargetVisualizerDisplay::updateTargetName()
  {
    boost::mutex::scoped_lock(mutex_);
    target_name_ = target_name_property_->getStdString();
    require_update_line_ = true;
  }
  
  void TargetVisualizerDisplay::updateRadius()
  {
    boost::mutex::scoped_lock(mutex_);
    radius_ = radius_property_->getFloat();
    require_update_line_ = true;
  }

  void TargetVisualizerDisplay::updateAlpha()
  {
    boost::mutex::scoped_lock(mutex_);
    alpha_ = alpha_property_->getFloat();
    require_update_line_ = true;
  }

  void TargetVisualizerDisplay::updateColor()
  {
    boost::mutex::scoped_lock(mutex_);
    color_ = color_property_->getColor();
    require_update_line_ = true;
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::TargetVisualizerDisplay, rviz::Display )

