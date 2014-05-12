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

#include "overlay_text_display.h"
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <QPainter>

namespace jsk_rviz_plugin
{
  OverlayTextDisplay::OverlayTextDisplay() : Display(),
                                             texture_width_(0), texture_height_(0),
                                             text_size_(14),
                                             line_width_(2),
                                             text_(""), font_(""),
                                             bg_color_(0, 0, 0, 0),
                                             fg_color_(255, 255, 255, 255.0),
                                             require_update_texture_(false)
  {
    update_topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                                         ros::message_traits::datatype<jsk_rviz_plugins::OverlayText>(),
                                                         "jsk_rviz_plugins::OverlayText topic to subscribe to.",
                                                         this, SLOT( updateTopic() ));
  }

  
  OverlayTextDisplay::~OverlayTextDisplay()
  {
    onDisable();
    //delete overlay_;
    delete update_topic_property_;
  }

  void OverlayTextDisplay::onEnable()
  {
    overlay_->show();
    subscribe();
  }

  void OverlayTextDisplay::onDisable()
  {
    overlay_->hide();
    unsubscribe();
  }
  
  void OverlayTextDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void OverlayTextDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = ros::NodeHandle().subscribe(topic_name, 1, &OverlayTextDisplay::processMessage, this);
    }
  }
  
  void OverlayTextDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
    
  // only the first time
  void OverlayTextDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "OverlayTextDisplayObject" << count++;
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(ss.str());
    //panel_ = static_cast<Ogre::OverlayContainer*> (
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("BorderPanel", ss.str() + "Panel"));
    material_name_ = ss.str() + "Material";
    texture_name_ = ss.str() + "Texture";
    // panel_ = static_cast<Ogre::PanelOverlayElement*> (
    //   mOverlayMgr->createOverlayElement("BorderPanel", ss.str() + "Panel"));
    
    // panel_material_
    //   = Ogre::MaterialManager::getSingleton().create(material_name_,
    //                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    
    // panel_->setMaterialName(panel_material_->getName());
    // panel_->setMetricsMode(Ogre::GMM_PIXELS);
    // overlay_->add2D(panel_);
    require_update_texture_ = false;
  }

  void OverlayTextDisplay::update(float wall_dt, float ros_dt)
  {
    if (!require_update_texture_) {
      return;
    }
    if (!isEnabled()) {
      return;
    }
    
    if (panel_material_.isNull()) {
      Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
      panel_material_
        = Ogre::MaterialManager::getSingleton().create(material_name_,
                                                       Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
      panel_->setMaterialName(panel_material_->getName());
      panel_->setMetricsMode(Ogre::GMM_PIXELS);
      overlay_->add2D(panel_);
    }
    
    updateTextureSize(texture_width_, texture_height_);
    // draw
    if (texture_.isNull()) {
      ROS_WARN("failed to crate texture");
      return;
    }
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );
    {
      memset( pDest, 0, texture_->getWidth() * texture_->getHeight() );
      QImage Hud( pDest, texture_->getWidth(), texture_->getHeight(), QImage::Format_ARGB32 );
      // initilize by the background color
      for (int i = 0; i < texture_->getWidth(); i++) {
        for (int j = 0; j < texture_->getHeight(); j++) {
          Hud.setPixel(i, j, bg_color_.rgba());
        }
      }
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));
      uint16_t w = texture_->getWidth();
      uint16_t h = texture_->getHeight();

      // font
      if (text_size_ != 0) {
        //QFont font = painter.font();
        QFont font(font_.length() > 0 ? font_.c_str(): "Arial");
        font.setPointSize(text_size_);
        font.setBold(true);
        painter.setFont(font);
      }
      if (text_.length() > 0) {
        //painter.drawText(0, 0, w, h, Qt::TextWordWrap | Qt::AlignLeft,
        painter.drawText(0, 0, w, h,
                         Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                         text_.c_str());
      }
      painter.end();
    }
    pixelBuffer->unlock();
    panel_->setDimensions(texture_->getWidth(), texture_->getHeight());
    require_update_texture_ = false;
  }
  
  void OverlayTextDisplay::processMessage
  (const jsk_rviz_plugins::OverlayText::ConstPtr& msg)
  {
    if (!isEnabled()) {
      return;
    }
    if (msg->action == jsk_rviz_plugins::OverlayText::DELETE) {
      if (overlay_->isVisible()) {
        overlay_->hide();
      }
    }
    else if (msg->action == jsk_rviz_plugins::OverlayText::ADD) {
      if (!overlay_->isVisible()) {
        overlay_->show();
      }
    }
    texture_width_ = msg->width;
    texture_height_ = msg->height;
    panel_->setPosition(msg->top, msg->left);
    // store message for update method
    text_ = msg->text;
    font_ = msg->font;
    bg_color_ = QColor(msg->bg_color.r * 255.0,
                       msg->bg_color.g * 255.0,
                       msg->bg_color.b * 255.0,
                       msg->bg_color.a * 255.0);
    fg_color_ = QColor(msg->fg_color.r * 255.0,
                       msg->fg_color.g * 255.0,
                       msg->fg_color.b * 255.0,
                       msg->fg_color.a * 255.0);
    text_size_ = msg->text_size;
    line_width_ = msg->line_width;
    require_update_texture_ = true;
  }

  void OverlayTextDisplay::updateTextureSize(int width, int height)
  {
    if ((width == 0) || (height == 0)) {
      ROS_DEBUG("width or height is set to 0");
      return;
    }
    
    if (texture_.isNull() ||
        ((width != texture_->getWidth()) ||
         (height != texture_->getHeight()))) {
      if (!texture_.isNull()) {
        Ogre::TextureManager::getSingleton().remove(texture_name_);
        panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
      }
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name_,        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,   // type
        width, height,   // width & height of the render window 
        0,                   // number of mipmaps
        Ogre::PF_A8R8G8B8,   // pixel format chosen to match a format Qt can use
        Ogre::TU_DEFAULT     // usage
        );
      panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
      panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayTextDisplay, rviz::Display )
