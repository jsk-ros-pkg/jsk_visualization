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

#include "overlay_utils.h"

namespace jsk_rviz_plugin
{
  OverlayObject::OverlayObject(const std::string& name)
    : name_(name)
  {
    std::string material_name = name_ + "Material";
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(name_);
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("Panel", name_ + "Panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    
    panel_material_
      = Ogre::MaterialManager::getSingleton().create(
        material_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    panel_->setMaterialName(panel_material_->getName());
    overlay_->add2D(panel_);
  }

  OverlayObject::~OverlayObject()
  {
    hide();
    // Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    // mOverlayMgr->destroyOverlayElement(panel_);
    //delete panel_;
    //delete overlay_;
  }

  std::string OverlayObject::getName()
  {
    return name_;
  }

  void OverlayObject::hide()
  {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
  }

  void OverlayObject::show()
  {
    if (!overlay_->isVisible()) {
      overlay_->show();
    }
  }

  bool OverlayObject::isTextureReady()
  {
    return !texture_.isNull();
  }

  bool OverlayObject::updateTextureSize(unsigned int width, unsigned int height)
  {
    const std::string texture_name = name_ + "Texture";
    if (!isTextureReady() ||
        ((width != texture_->getWidth()) ||
         (height != texture_->getHeight()))) {
      if (!isTextureReady()) {
        Ogre::TextureManager::getSingleton().remove(texture_name);
        panel_material_->getTechnique(0)->getPass(0)
          ->removeAllTextureUnitStates();
      }
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name,        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,   // type
        width, height,   // width & height of the render window 
        0,                   // number of mipmaps
        Ogre::PF_A8R8G8B8,   // pixel format chosen to match a format Qt can use
        Ogre::TU_DEFAULT     // usage
        );
      panel_material_->getTechnique(0)->getPass(0)
        ->createTextureUnitState(texture_name);
        
      panel_material_->getTechnique(0)->getPass(0)
        ->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }

  Ogre::HardwarePixelBufferSharedPtr OverlayObject::getBuffer()
  {
    if (isTextureReady()) {
      return texture_->getBuffer();
    }
    else {
      return Ogre::HardwarePixelBufferSharedPtr();
    }
  }

  void OverlayObject::setPosition(double left, double top)
  {
    panel_->setPosition(left, top);
  }

  void OverlayObject::setDimensions(double width, double height)
  {
    panel_->setDimensions(width, height);
  }

  bool OverlayObject::isVisible()
  {
    return overlay_->isVisible();
  }

  unsigned int OverlayObject::getTextureWidth()
  {
    if (isTextureReady()) {
      return texture_->getWidth();
    }
    else {
      return 0;
    }
  }

  unsigned int OverlayObject::getTextureHeight()
  {
    if (isTextureReady()) {
      return texture_->getHeight();
    }
    else {
      return 0;
    }
  }

  
}
