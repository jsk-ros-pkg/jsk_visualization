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

#include "people_position_measurement_array_display.h"
#include <rviz/uniform_string_stream.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <OGRE/OgreCamera.h>
#include <QPainter>
#include <rviz/ogre_helpers/render_system.h>
#include <OGRE/OgreRenderSystem.h>

#include <algorithm>
#include <boost/lambda/lambda.hpp>

namespace jsk_rviz_plugin
{

  SquareObject::SquareObject(Ogre::SceneManager* manager, double size,
                             std::string name):
    manager_(manager)
  {
    manual_ = manager->createManualObject();
    manual_->begin(name,
                   Ogre::RenderOperation::OT_TRIANGLE_FAN);
    const size_t resolution = 100;
    for (size_t i = 0; i < resolution; i++) {
      double theta = 2.0 * M_PI / resolution * i;
      manual_->position(size * cos(theta), size * sin(theta), 0.0f);
      manual_->textureCoord((1 + cos(theta)) / 2.0, (1.0 -sin(theta)) / 2.0);
    }
    for (size_t i = 0; i < resolution; i++) {
      manual_->index(i);
    }
    manual_->index(0);
    // manual_->position(-size, -size, 0.0f);
    // manual_->textureCoord(0, 1);
    // manual_->position( size, -size, 0.0f);
    // manual_->textureCoord(1, 1);
    // manual_->position( size,  size, 0.0f);
    // manual_->textureCoord(1, 0);
    // manual_->position(-size,  size, 0.0f);
    // manual_->textureCoord(0, 0);
    // manual_->index(0);//Front Side
    // manual_->index(1);
    // manual_->index(2);
    // manual_->index(3);
    // manual_->index(0);//Back Side
    // manual_->index(1);
    manual_->end();
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
  
  TextureObject::TextureObject(const int width, const int height,
                               const std::string name):
    width_(width), height_(height), name_(name),
    created_at_(ros::WallTime::now())
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

  ros::WallTime TextureObject::getCreatedAt()
  {
    return created_at_;
  }
  
  PeoplePositionMeasurementArrayDisplay::PeoplePositionMeasurementArrayDisplay()
  {
    size_property_ = new rviz::FloatProperty("size", 0.3,
                                             "size of the visualizer", this,
                                             SLOT(updateSize()));
    timeout_property_ = new rviz::FloatProperty(
      "timeout", 10.0, "timeout seconds", this, SLOT(updateTimeout()));
    anonymous_property_ = new rviz::BoolProperty(
      "anonymous", false,
      "anonymous",
      this, SLOT(updateAnonymous()));
    text_property_ = new rviz::StringProperty(
      "text", "person found here person found here",
      "text to rotate",
      this, SLOT(updateText()));
  }

  
  PeoplePositionMeasurementArrayDisplay::~PeoplePositionMeasurementArrayDisplay()
  {
    delete size_property_;
  }

  void PeoplePositionMeasurementArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    updateSize();
    updateTimeout();
    updateAnonymous();
    updateText();
  }

  void PeoplePositionMeasurementArrayDisplay::clearObjects()
  {
    faces_.clear();
    //textures_.clear();
    squares_.clear();
    for (size_t i = 0; i < nodes_.size(); i++) {
      scene_manager_->destroySceneNode(nodes_[i]);
    }
    nodes_.clear();
  }
  
  void PeoplePositionMeasurementArrayDisplay::reset()
  {
    MFDClass::reset();
    clearObjects();
  }

  void PeoplePositionMeasurementArrayDisplay::processMessage(
    const people_msgs::PositionMeasurementArray::ConstPtr& msg)
  {
    // 1. allocate texture
    // 2. allocate/move scene nodes
    boost::mutex::scoped_lock lock(mutex_);
    static int count = 0;
    static int square_count = 0;
    faces_ = msg->people;
    // check texture is ready or not
    if (faces_.size() > textures_.size()) {
      ROS_DEBUG("need to allocate %lu more textures", faces_.size() - textures_.size());
      for (size_t i = textures_.size(); i < faces_.size(); i++) {
        rviz::UniformStringStream ss;
        ss << "PeoplePositionMeasurementArrayDisplayObject" << count++;
        textures_.push_back(TextureObject::Ptr(
                              new TextureObject(128, 128, ss.str())));
      }
    }
    // else if (faces_.size() < textures_.size()) { // too match
    //   textures_.resize(faces_.size());
    // }

    if (faces_.size() > nodes_.size()) {
      ROS_DEBUG("need to allocate %lu more scene nodes", faces_.size() - nodes_.size());
      for (size_t i = nodes_.size(); i < faces_.size(); i++) {
        nodes_.push_back(scene_node_->createChildSceneNode());
      }
    }
    else if (faces_.size() < nodes_.size()) {
      for (size_t i = faces_.size(); i < nodes_.size(); i++) {
        scene_manager_->destroySceneNode(nodes_[i]);
      }
      nodes_.resize(faces_.size()); // need to destory here
    }

    if (faces_.size() > squares_.size()) {
      ROS_DEBUG("need to allocate %lu more scene squares", faces_.size() - squares_.size());
      for (size_t i = squares_.size(); i < faces_.size(); i++) {
        // rviz::UniformStringStream ss;
        // ss << "OverlayImageDisplayObjectManualObject" << square_count++;
        squares_.push_back(SquareObject::Ptr(
                             new SquareObject(scene_manager_, size_,
                                              textures_[i]->getMaterialName())));
      }
    }
    else if (faces_.size() < squares_.size()) {
      squares_.resize(faces_.size()); // need to destory here
    }
    
    // move scene_node
    for (size_t i = 0; i < faces_.size(); i++) {
      Ogre::Quaternion orientation;
      Ogre::Vector3 position;
      geometry_msgs::Pose pose;
      pose.position = faces_[i].pos;
      pose.orientation.w = 1.0;
      if(!context_->getFrameManager()->transform(msg->header,
                                                 pose,
                                                 position, orientation))
      {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        
      }
      else {
        nodes_[i]->setPosition(position);
        nodes_[i]->setOrientation(orientation);
      }
      // squares_[i]->getManualObject()->detachFromParent();
      // nodes_[i]->attachObject(squares_[i]->getManualObject());
    }

    latest_time_ = msg->header.stamp;
  }

  bool compareSceneNode(const Ogre::SceneNode* a, const Ogre::SceneNode* b,
                        const Ogre::Vector3& camera_pos)
  {
    double adist = (a->getPosition() - camera_pos).length();
    double bdist = (b->getPosition() - camera_pos).length();
    if (adist > bdist) {
      return false;
    }
    else {
      return true;
    }
    
  }
  
  void PeoplePositionMeasurementArrayDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (faces_.size() == 0) {
      return;
    }
    if ((ros::Time::now() - latest_time_).toSec() > timeout_) {
      ROS_WARN("timeout face recognition result");
      clearObjects();
      return;
      //faces_.clear();
      //return;
    }
    rviz::ViewManager* manager = context_->getViewManager();
    rviz::RenderPanel* panel = manager->getRenderPanel();
    rviz::RenderSystem::RenderSystem::get()->root()->getRenderSystem()->_setAlphaRejectSettings(Ogre::CMPF_GREATER_EQUAL, 1, true);
    Ogre::Camera* camera = panel->getCamera();
    for (size_t i = 0; i < nodes_.size(); i++) {
      nodes_[i]->setOrientation(camera->getDerivedOrientation());
    }
    
    // sort textures_ according to faces_ z-order.
    Ogre::Vector3 camera_pos = camera->getDerivedPosition();
    // sort scene_nodes according to the distance from camera
    std::sort(nodes_.begin(), nodes_.end(), boost::bind(compareSceneNode, _1, _2, camera_pos));
    for (size_t i = 0; i < nodes_.size(); i++) {
      squares_[i]->getManualObject()->detachFromParent();
      nodes_[i]->attachObject(squares_[i]->getManualObject());
    }
    // draw texture
    ros::WallTime now = ros::WallTime::now();
    for (size_t i = 0; i < nodes_.size(); i++) {
      ScopedPixelBuffer buffer = textures_[i]->getBuffer();
      QColor transparent(0, 0, 0, 0);
      QColor foreground(33, 73, 140, 255);
      QColor white(255, 255, 255, 255);
      QImage Hud = buffer.getQImage(128, 128, transparent);
      double line_width = 5;
      double inner_line_width = 10;
      double l = 128;
      double cx = l / 2 - line_width / 4.0;
      double r = 48;
      double inner_r = 40;
      double mouse_r = 30;
      double mouse_cy_offset = 5;
      double cy = l / 2 - line_width / 4.0;
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      //painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
      painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
      painter.setBrush(white);
      //painter.setBrush(transparent);
      painter.drawEllipse(line_width / 2.0, line_width / 2.0,
                          l - line_width, l - line_width);
      std::string text = text_;
      double offset_rate = fmod((now - textures_[i]->getCreatedAt()).toSec(), 10) / 10.0;
      double theta_offset = offset_rate * M_PI * 2.0;
      //std::string text = "AAAAAAAAAAAAAAAAAAAAaA";
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
      //painter.setBrush(white);
      painter.setBrush(transparent);
      painter.drawEllipse(cx - inner_r, cy - inner_r,
                          inner_r * 2, inner_r * 2);
      if (!anonymous_) {
        // force to clear the inside of the circle
        for (unsigned int i = 0; i < Hud.width(); i++) {
          for (unsigned int j = 0; j < Hud.height(); j++) {
            double dist = sqrt((i - cx) * (i - cx) + (j - cy) * (j - cy));
            if (dist < inner_r) {
              Hud.setPixel(i, j, transparent.rgba());
            }
          }
        }
      }
      else {                    // overlay face ^^
        double mouse_c_x = cx;
        double mouse_c_y = cy - mouse_cy_offset;
        double start_angle = -25 * M_PI / 180;
        double end_angle = -125 * M_PI / 180;
        painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
        painter.drawChord(mouse_c_x - mouse_r, mouse_c_y - mouse_r,
                        2.0 * mouse_r, 2.0 * mouse_r,
                        start_angle * 180 / M_PI * 16,
                        end_angle * 180 / M_PI * 16);
      }
      painter.end();
    }
  }

  void PeoplePositionMeasurementArrayDisplay::updateTimeout()
  {
    boost::mutex::scoped_lock lock(mutex_);
    timeout_ = timeout_property_->getFloat();
  }
  
  void PeoplePositionMeasurementArrayDisplay::updateSize()
  {
    boost::mutex::scoped_lock lock(mutex_);
    size_ = size_property_->getFloat();
    // force to clear squares
    squares_.clear();
  }

  void PeoplePositionMeasurementArrayDisplay::updateAnonymous()
  {
    boost::mutex::scoped_lock lock(mutex_);
    anonymous_ = anonymous_property_->getBool();
  }

  void PeoplePositionMeasurementArrayDisplay::updateText()
  {
    boost::mutex::scoped_lock lock(mutex_);
    text_ = text_property_->getStdString();
  }
  
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::PeoplePositionMeasurementArrayDisplay, rviz::Display )

