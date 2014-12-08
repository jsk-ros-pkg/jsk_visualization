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

#include "pictogram_display.h"
#include <QPainter>
#include <QFontDatabase>
#include <ros/package.h>

////////////////////////////////////////////////////////
// read Entypo fonts
// http://mempko.wordpress.com/2014/11/28/using-entypo-fonts-as-icons-in-your-qt-application/
////////////////////////////////////////////////////////

#include "Entypo.dat"
#include "Entypo_Social.dat"
#include "fontawesome.dat"
#include "pictogram_font_mapping.h"

namespace jsk_rviz_plugin
{  
  bool PictogramObject::isCharacterSupported(std::string character)
  {
    return ((entypo_social_character_map_.find(character)
             != entypo_social_character_map_.end()) ||
            (entypo_character_map_.find(character)
             != entypo_character_map_.end()) ||
            (fontawesome_character_map_.find(character)
             != fontawesome_character_map_.end()));
  }
  
  QFont PictogramObject::getFont(std::string character)
  {
    if (entypo_social_character_map_.find(character)
        != entypo_social_character_map_.end()) {
      return QFont("Entypo Social");
    }
    else if (entypo_character_map_.find(character)
             != entypo_character_map_.end()) {
      return QFont("Entypo");
    }
    else {
      return QFont("FontAwesome");
    }
  }
  
  QString PictogramObject::lookupPictogramText(std::string character)
  {
    if (entypo_social_character_map_.find(character)
        != entypo_social_character_map_.end()) {
      return entypo_social_character_map_[character];
    }
    else if (entypo_character_map_.find(character)
             != entypo_character_map_.end()){
      return entypo_character_map_[character];
    }
    else {
      return fontawesome_character_map_[character];
    }
  }
  
  PictogramObject::PictogramObject(Ogre::SceneManager* manager,
                                   Ogre::SceneNode* parent,
                                   double size,
                                   int entypo_font_id,
                                   int entypo_social_font_id):
    FacingTexturedObject(manager, parent, size),
    entypo_font_id_(entypo_font_id),
    entypo_social_font_id_(entypo_social_font_id),
    need_to_update_(false)
  {
    square_object_->setPolygonType(SquareObject::SQUARE);
    square_object_->rebuildPolygon();
    setupCharacterMap();
    
    // for (std::map<std::string, QString>::iterator it = fontawesome_character_map_.begin();
    //      it != fontawesome_character_map_.end();
    //      ++it) {
    //   ROS_INFO("%s", it->first.c_str());
    // }
  }

  void PictogramObject::setEnable(bool enable)
  {
    FacingTexturedObject::setEnable(enable);
    if (enable) {
      need_to_update_ = true;
    }
  }

  bool PictogramObject::isEntypo(std::string text) {
    return ((entypo_social_character_map_.find(text)
             != entypo_social_character_map_.end()) ||
            (entypo_character_map_.find(text)
             != entypo_character_map_.end()));
  }

  bool PictogramObject::isFontAwesome(std::string text) {
    return (fontawesome_character_map_.find(text)
            != fontawesome_character_map_.end());
  }
  
  void PictogramObject::update(float wall_dt, float ros_dt)
  {
    if (!need_to_update_) {
      return;
    }
    need_to_update_ = false;
    ScopedPixelBuffer buffer = texture_object_->getBuffer();
    QColor transparent(255, 255, 255, 0);
    QImage Hud = buffer.getQImage(128, 128, transparent); // should change according to size
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);
    QColor foreground = rviz::ogreToQt(color_);
    painter.setPen(QPen(foreground, 5, Qt::SolidLine));
    if (text_.empty()) {
      // not yet setted
      return;
    }
    else if (isCharacterSupported(text_)) {
      QFont font = getFont(text_);
      QString pictogram_text = lookupPictogramText(text_);
      if (isEntypo(text_)) {
        font.setPointSize(100);
      }
      else if (isFontAwesome(text_)) {
        font.setPointSize(45);
      }
      painter.setFont(font);
      painter.drawText(0, 0, 128, 128,
                       Qt::AlignHCenter | Qt::AlignVCenter,
                       pictogram_text);
      painter.end();
    }
    else {
      ROS_WARN("%s is not supported", text_.c_str());
    }
  }

  void PictogramObject::updateColor()
  {
    need_to_update_ = true;
  }
  void PictogramObject::updateText()
  {
    need_to_update_ = true;
  }

  int PictogramDisplay::addFont(unsigned char* data, unsigned int data_len)
  {
    // register font
    QByteArray entypo =
      QByteArray::fromRawData(
        reinterpret_cast<const char*>(data), data_len);
    int id =
      QFontDatabase::addApplicationFontFromData(entypo);
    if (id == -1) {
      ROS_WARN("failed to load font");
    }
    else {
      return id;
    }

  }
  
  PictogramDisplay::PictogramDisplay()
  {
    entypo_id_ = addFont(Entypo_ttf, Entypo_ttf_len);
    entypo_social_id_ = addFont(Entypo_Social_ttf, Entypo_Social_ttf_len);
    addFont(fontawesome_webfont_ttf,
            font_awesome_4_2_0_fonts_fontawesome_webfont_ttf_len);
  }

  PictogramDisplay::~PictogramDisplay()
  {
    
  }
  
  void PictogramDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    pictogram_.reset(new PictogramObject(scene_manager_,
                                         scene_node_,
                                         1.0,
                                         entypo_id_,
                                         entypo_social_id_));
    pictogram_->setEnable(false);
    // initial setting
    pictogram_->setColor(QColor(25, 255, 240));
    pictogram_->setAlpha(1.0);
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  void PictogramDisplay::reset()
  {
    MFDClass::reset();
  }

  void PictogramDisplay::onEnable()
  {
    subscribe();
    if (pictogram_) {
      // keep false, it will be true
      // in side of processMessae callback.
      pictogram_->setEnable(false);
    }
  }

  void PictogramDisplay::processMessage(const jsk_rviz_plugins::Pictogram::ConstPtr& msg)
  {
    boost::mutex::scoped_lock (mutex_);
    pictogram_->setEnable(isEnabled());
    if (!isEnabled()) {
      return;
    }
    if (msg->action == jsk_rviz_plugins::Pictogram::DELETE) {
      pictogram_->setEnable(false);
      return;
    }
    Ogre::Vector3 position;
    Ogre::Quaternion quaternion;
    if(!context_->getFrameManager()->transform(msg->header,
                                               msg->pose,
                                               position,
                                               quaternion)) {
      ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                 qPrintable( getName() ), msg->header.frame_id.c_str(),
                 qPrintable( fixed_frame_ ));
      return;
    }
    if (msg->size <= 0.0) {
      pictogram_->setSize(0.5);
    }
    else {
      pictogram_->setSize(msg->size / 2.0);
    }
    pictogram_->setColor(QColor(msg->color.r * 255,
                                msg->color.g * 255,
                                msg->color.b * 255));
    pictogram_->setAlpha(msg->color.a);
    pictogram_->setPosition(position);
    pictogram_->setOrientation(quaternion);
    pictogram_->setText(msg->character);
  }

  void PictogramDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock (mutex_);
    if (pictogram_) {
      pictogram_->update(wall_dt, ros_dt);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugin::PictogramDisplay, rviz::Display);
