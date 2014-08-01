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
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
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
    if (overlay_) {
      overlay_->show();
    }
    subscribe();
  }

  void OverlayTextDisplay::onDisable()
  {
    if (overlay_) {
      overlay_->hide();
    }
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
    onEnable();
    updateTopic();
    require_update_texture_ = true;
  }
  
  void OverlayTextDisplay::update(float wall_dt, float ros_dt)
  {
    if (!require_update_texture_) {
      return;
    }
    if (!isEnabled()) {
      return;
    }
    if (!overlay_) {
      return;
    }
    overlay_->updateTextureSize(texture_width_, texture_height_);
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = overlay_->getBuffer();
    pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );
    {
      memset( pDest, 0,
              overlay_->getTextureWidth() * overlay_->getTextureHeight() );
      QImage Hud( pDest,
                  overlay_->getTextureWidth(), overlay_->getTextureHeight(), QImage::Format_ARGB32 );
      // initilize by the background color
      for (int i = 0; i < overlay_->getTextureWidth(); i++) {
        for (int j = 0; j < overlay_->getTextureHeight(); j++) {
          Hud.setPixel(i, j, bg_color_.rgba());
        }
      }
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));
      uint16_t w = overlay_->getTextureWidth();
      uint16_t h = overlay_->getTextureHeight();

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
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    require_update_texture_ = false;
  }
  
  void OverlayTextDisplay::processMessage
  (const jsk_rviz_plugins::OverlayText::ConstPtr& msg)
  {
    if (!isEnabled()) {
      return;
    }
    if (!overlay_) {
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "OverlayTextDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    if (overlay_) {
      if (msg->action == jsk_rviz_plugins::OverlayText::DELETE) {
        overlay_->hide();
      }
      else if (msg->action == jsk_rviz_plugins::OverlayText::ADD) {
        overlay_->show();
      }
    }
    texture_width_ = msg->width;
    texture_height_ = msg->height;
    if (overlay_) {
      overlay_->setPosition(msg->left, msg->top);
    }
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
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayTextDisplay, rviz::Display )
