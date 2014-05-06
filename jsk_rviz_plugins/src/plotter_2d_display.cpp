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

#include "plotter_2d_display.h"
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <QPainter>

namespace jsk_rviz_plugin
{
  Plotter2DDisplay::Plotter2DDisplay()
    : rviz::Display()
  {
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<std_msgs::Float32>(),
      "std_msgs::Float32 topic to subscribe to.",
      this, SLOT( updateTopic() ));
    buffer_length_property_ = new rviz::IntProperty(
      "Buffer length", 100,
      ros::message_traits::datatype<std_msgs::Float32>(),
      this, SLOT(updateBufferSize()));
    width_property_ = new rviz::IntProperty("width", 128,
                                            "width of the plotter window",
                                            this, SLOT(updateWidth()));
    height_property_ = new rviz::IntProperty("height", 128,
                                             "height of the plotter window",
                                             this, SLOT(updateHeight()));
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the plotter window",
                                           this, SLOT(updateLeft()));
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the plotter window",
                                          this, SLOT(updateTop()));
    fg_color_property_ = new rviz::ColorProperty("foreground color", QColor(25, 255, 240),
                                                 "color to draw line",
                                                 this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty("foreground alpha", 0.7,
                                                 "alpha belnding value for foreground",
                                                 this, SLOT(updateFGAlpha()));
    bg_color_property_ = new rviz::ColorProperty("background color", QColor(0, 0, 0),
                                                 "background color",
                                                 this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty("backround alpha", 0.0,
                                                 "alpha belnding value for background",
                                                 this, SLOT(updateBGAlpha()));
    line_width_property_ = new rviz::IntProperty("linewidth", 1,
                                                 "linewidth of the plot",
                                                 this, SLOT(updateLineWidth()));
    show_border_property_ = new rviz::BoolProperty("border", true,
                                                   "show border or not",
                                                   this, SLOT(updateShowBorder()));
  }

  Plotter2DDisplay::~Plotter2DDisplay()
  {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
    delete update_topic_property_;
    delete buffer_length_property_;
    delete fg_color_property_;
    delete bg_color_property_;
    delete fg_alpha_property_;
    delete bg_alpha_property_;
    delete top_property_;
    delete left_property_;
    delete width_property_;
    delete height_property_;
    delete line_width_property_;
    delete show_border_property_;
  }

  void Plotter2DDisplay::initializeBuffer()
  {
    buffer_.resize(buffer_length_);
    min_value_ = -1.0;
    max_value_ = 1.0;
    for (size_t i = 0; i < buffer_length_; i++) {
      buffer_[i] = 0.0;
    }
  }

  void Plotter2DDisplay::onInitialize()
  {
    static int count = 0;
    updateBufferSize();
    rviz::UniformStringStream ss;
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    ss << "Plotter2DDisplayObject" << count++;
    material_name_ = ss.str() + "Material";
    texture_name_ = ss.str() + "Texture";
    overlay_ = mOverlayMgr->create(ss.str());
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("Panel", ss.str() + "Panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    panel_material_
      = Ogre::MaterialManager::getSingleton().create(
        material_name_,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    panel_->setMaterialName(panel_material_->getName());
    overlay_->add2D(panel_);
    onEnable();
    updateTextureSize(width_property_->getInt(), height_property_->getInt());
    updateWidth();
    updateHeight();
    updateLeft();
    updateTop();
    updateFGColor();
    updateBGColor();
    updateFGAlpha();
    updateBGAlpha();
    updateLineWidth();
    updateShowBorder();
  }

  void Plotter2DDisplay::updateTextureSize(uint16_t width, uint16_t height)
  {
    //boost::mutex::scoped_lock lock(mutex_);
    
    if (texture_.isNull() ||
        ((width != texture_->getWidth()) || (height != texture_->getHeight()))) {
      if (!texture_.isNull()) {
        // remove the texture first if previous texture exists
        Ogre::TextureManager::getSingleton().remove(texture_name_);
      }
      texture_width_ = width;
      texture_height_ = height;
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

  void Plotter2DDisplay::drawPlot()
  {
    
    QColor fg_color(fg_color_);
    QColor bg_color(bg_color_);
    fg_color.setAlpha(fg_alpha_);
    bg_color.setAlpha(bg_alpha_);
    
    // Get the pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );

    // construct HUD image directly in the texture buffer
    {
      // fill to get 100% transparent image
      // the buffer content is the colors R,G,B,A. Filling with zeros gets a 100% transparent image
      memset( pDest, 0, texture_->getWidth() * texture_->getHeight() );
      
      // tell QImage to use OUR buffer and a compatible image buffer format
      QImage Hud( pDest, texture_->getWidth(), texture_->getHeight(), QImage::Format_ARGB32 );
      // initilize by the background color
      for (int i = 0; i < texture_->getWidth(); i++) {
        for (int j = 0; j < texture_->getHeight(); j++) {
          Hud.setPixel(i, j, bg_color.rgba());
        }
      }
      // paste in HUD speedometer. I resize the image and offset it by 8 pixels from
      // the bottom left edge of the render window
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color, line_width_, Qt::SolidLine));
      
      uint8_t Offset = 8;
      uint16_t w = texture_->getWidth();
      uint16_t h = texture_->getHeight();
      for (size_t i = 1; i < buffer_length_; i++) {
        double v_prev = (max_value_ - buffer_[i - 1]) / (max_value_ - min_value_);
        double v = (max_value_ - buffer_[i]) / (max_value_ - min_value_);
        double u_prev = (i - 1) / (float)buffer_length_;
        double u = i / (float)buffer_length_;

        uint16_t x_prev = (int)(u_prev * texture_width_);
        uint16_t x = (int)(u * texture_width_);
        uint16_t y_prev = (int)(v_prev * texture_height_);
        uint16_t y = (int)(v * texture_height_);
        painter.drawLine(x_prev, y_prev, x, y);
      }
      // draw border
      if (show_border_) {
        painter.drawLine(0, 0, 0, texture_height_);
        painter.drawLine(0, texture_height_, texture_width_, texture_height_);
        painter.drawLine(texture_width_, texture_height_, texture_width_, 0);
        painter.drawLine(texture_width_, 0, 0, 0);
      }
      
      //painter.drawLine(0, 0, 128, 128);
      //painter.drawImage( QRect( Offset, texture_->getHeight() - h - Offset, w, h ), ImageHudSpeed, QRect( 0, 0, ImageHudSpeed.width(), ImageHudSpeed.height() ) );

      // do any other drawing you would like here using painter

      // done
      painter.end();
    }
    // note the QImage is destroyed. Bad things will happen if you reuse the QImage!
    
    // Unlock the pixel buffer
    pixelBuffer->unlock();

  }
  
  void Plotter2DDisplay::processMessage(const std_msgs::Float32::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // add the message to the buffer
    double min_value = buffer_[0];
    double max_value = buffer_[0];
    for (size_t i = 0; i < buffer_length_ - 1; i++) {
      buffer_[i] = buffer_[i + 1];
      if (min_value > buffer_[i]) {
        min_value = buffer_[i];
      }
      if (max_value < buffer_[i]) {
        max_value = buffer_[i];
      }
    }
    buffer_[buffer_length_ - 1] = msg->data;
    if (min_value > msg->data) {
      min_value = msg->data;
    }
    if (max_value < msg->data) {
      max_value = msg->data;
    }
    min_value_ = min_value;
    max_value_ = max_value;
    if (min_value_ == max_value_) {
      min_value_ = min_value_ - 0.5;
      max_value_ = max_value_ + 0.5;
    }

    if (!overlay_->isVisible()) {
      return;
    }
    
    // draw it
    //if (!textire_.isNull()) {
    updateTextureSize(texture_width_, texture_height_);
    drawPlot();
    panel_->setPosition(left_, top_);
    panel_->setDimensions(texture_->getWidth(), texture_->getHeight());
      //}
  }
  
  void Plotter2DDisplay::subscribe()
  {
    initializeBuffer();
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      ros::NodeHandle n;
      sub_ = n.subscribe(topic_name, 1, &Plotter2DDisplay::processMessage, this);
    }
  }

  void Plotter2DDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void Plotter2DDisplay::onEnable()
  {
    subscribe();
    overlay_->show();
  }

  void Plotter2DDisplay::onDisable()
  {
    unsubscribe();
    overlay_->hide();
  }

  void Plotter2DDisplay::updateWidth()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_width_ = width_property_->getInt();
  }
  
  void Plotter2DDisplay::updateHeight()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_height_ = height_property_->getInt();
  }
  
  void Plotter2DDisplay::updateTop()
  {
    top_ = top_property_->getInt();
  }
  
  void Plotter2DDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
  }
  
  void Plotter2DDisplay::updateBGColor()
  {
    bg_color_ = bg_color_property_->getColor();
  }

  void Plotter2DDisplay::updateFGColor()
  {
    fg_color_ = fg_color_property_->getColor();
  }

  void Plotter2DDisplay::updateFGAlpha()
  {
    fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  }

  void Plotter2DDisplay::updateBGAlpha()
  {
    bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  }
  
  void Plotter2DDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  void Plotter2DDisplay::updateShowBorder()
  {
    show_border_ = show_border_property_->getBool();
  }
  
  void Plotter2DDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getInt();
  }
  
  void Plotter2DDisplay::updateBufferSize()
  {
    buffer_length_ = buffer_length_property_->getInt();
    initializeBuffer();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::Plotter2DDisplay, rviz::Display )
