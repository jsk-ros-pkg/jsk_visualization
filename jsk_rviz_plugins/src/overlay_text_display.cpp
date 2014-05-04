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

namespace jsk_rviz_plugin
{
  OverlayTextDisplay::OverlayTextDisplay() : Display()
  {
    update_topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                                         ros::message_traits::datatype<jsk_rviz_plugins::OverlayText>(),
                                                         "jsk_rviz_plugins::OverlayText topic to subscribe to.",
                                                         this, SLOT( updateTopic() ));

  }

  
  OverlayTextDisplay::~OverlayTextDisplay()
  {
    //delete overlay_;
    delete update_topic_property_;
  }

  void OverlayTextDisplay::onEnable()
  {
    subscribe();
  }

  void OverlayTextDisplay::onDisable()
  {
    reset();
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
  
  // this method is defined to generate texture map manually.
  // because in order to specify background color of the overlay panel,
  // using texture mapping is the only way to do that as far as I know.
  // Thus, this method generate 1x1 texture mapping specified by `color'
  // argument.
  void OverlayTextDisplay::updateTexture(const std_msgs::ColorRGBA& color,
                                         bool force_to_create)
  {
    if (force_to_create ||
        ((color.r != bg_color_.r) ||
         (color.g != bg_color_.g) ||
         (color.b != bg_color_.b) ||
         (color.a != bg_color_.a))) {
      bg_color_.r = color.r;
      bg_color_.g = color.g;
      bg_color_.b = color.b;
      bg_color_.a = color.a;
      // here we manually generate the texture
      Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().getByName(texture_name_);
      if (texture.isNull()) {
        texture = Ogre::TextureManager::getSingleton().createManual(
          texture_name_, // name
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_2D,      // type
          1, 1,         // width & height
          0,                // number of mipmaps
          Ogre::PF_BYTE_BGRA,     // pixel format
          Ogre::TU_DEFAULT);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
      }
      // textures updated very often (e.g. each frame)
      Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();
      // Lock the pixel buffer and get a pixel box
      pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
      const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
      uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
      
      for (size_t j = 0; j < 1; j++)
      {
        for(size_t i = 0; i < 1; i++)
        {
          *pDest++ = bg_color_.b * 255; // B
          *pDest++ =   bg_color_.g * 255; // G
          *pDest++ =   bg_color_.r * 255; // R
          *pDest++ = bg_color_.a * 255; // A
        }
        
        pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
      }
      
      pixelBuffer->unlock();
    }
  }
  
  // only the first time
  void OverlayTextDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    //bg_color_.r = bg_color_.g = bg_color_.b = bg_color_.a = 1.0;
    bg_color_.r = bg_color_.g = bg_color_.b = bg_color_.a = 1.0;
    ss << "OverlayTextDisplayObject" << count++;
    //MFDClass::onInitialize();
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create("foo");
    
    //panel_ = static_cast<Ogre::OverlayContainer*> (
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("BorderPanel", ss.str() + "Panel"));
    textArea_ = static_cast<Ogre::TextAreaOverlayElement*>(
      mOverlayMgr->createOverlayElement("TextArea", ss.str() + "TextArea"));
    material_name_ = ss.str() + "Material";
    texture_name_ = ss.str() + "Texture";
    
    panel_material_
      = Ogre::MaterialManager::getSingleton().create(material_name_,
                                                     Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    

    updateTexture(bg_color_, true);
    
    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

    panel_->setMaterialName(panel_material_->getName());
    
    textArea_->setPosition(0, 0);
    textArea_->setDimensions(1.0, 1.0);
    textArea_->setFontName("Arial");
    overlay_->add2D(panel_);
    panel_->addChild(textArea_);
    onEnable();
  }

  // This method is called everytime users uncheck the box
  void OverlayTextDisplay::reset()
  {
    overlay_->hide();
    // todo: clear panel_ and textArea_
  }

  void OverlayTextDisplay::processMessage
  (const jsk_rviz_plugins::OverlayText::ConstPtr& msg)
  {
    if (!overlay_->isVisible()) {
      overlay_->show();
    }
    panel_->setPosition(msg->top, msg->left);
    panel_->setDimensions(msg->width, msg->height);
    //panel_->setMaterialName(panel_material_->getName());
    updateTexture(msg->bg_color, false);
    // panel_->setColour(Ogre::ColourValue(msg->bg_color.r,
    //                                     msg->bg_color.g,
    //                                     msg->bg_color.b,
    //                                     msg->bg_color.a));
    
    textArea_->setColourBottom(Ogre::ColourValue(msg->fg_color.r,
                                                 msg->fg_color.g,
                                                 msg->fg_color.b,
                                                 msg->fg_color.a));
    textArea_->setColourTop(Ogre::ColourValue(msg->fg_color.r,
                                              msg->fg_color.g,
                                              msg->fg_color.b,
                                              msg->fg_color.a));
    textArea_->setCharHeight(msg->text_size);
    textArea_->setCaption(msg->text);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayTextDisplay, rviz::Display )
