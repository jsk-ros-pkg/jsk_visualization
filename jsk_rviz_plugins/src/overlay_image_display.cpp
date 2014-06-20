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

#include "overlay_image_display.h"

#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreTechnique.h>

#include <rviz/uniform_string_stream.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_rviz_plugin
{

  OverlayImageDisplay::OverlayImageDisplay()
    : Display(), width_(128), height_(128), left_(128), top_(128), alpha_(0.8),
      require_update_(false)
  {
    // setup properties
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<sensor_msgs::Image>(),
      "sensor_msgs::Image topic to subscribe to.",
      this, SLOT( updateTopic() ));
    width_property_ = new rviz::IntProperty("width", 128,
                                            "width of the image window",
                                            this, SLOT(updateWidth()));
    height_property_ = new rviz::IntProperty("height", 128,
                                             "height of the image window",
                                             this, SLOT(updateHeight()));
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the image window",
                                           this, SLOT(updateLeft()));
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the image window",
                                          this, SLOT(updateTop()));
    alpha_property_ = new rviz::FloatProperty("alpha", 0.8,
                                              "alpha belnding value",
                                              this, SLOT(updateAlpha()));
  }

  OverlayImageDisplay::~OverlayImageDisplay()
  {
    delete update_topic_property_;
    delete width_property_;
    delete height_property_;
    delete left_property_;
    delete top_property_;
    delete alpha_property_;
  }
  
  void OverlayImageDisplay::onInitialize()
  {
    ros::NodeHandle nh;
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));

    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "OverlayImageDisplayObject" << count++;
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(ss.str());
    //panel_ = static_cast<Ogre::OverlayContainer*> (
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("BorderPanel", ss.str() + "Panel"));
    material_name_ = ss.str() + "Material";
    texture_name_ = ss.str() + "Texture";
    updateWidth();
    updateHeight();
    updateTop();
    updateLeft();
    updateAlpha();
    updateTopic();
  }
  
  void OverlayImageDisplay::onEnable()
  {
    overlay_->show();
    subscribe();
  }
  void OverlayImageDisplay::onDisable()
  {
    overlay_->hide();
    unsubscribe();
  }

  void OverlayImageDisplay::unsubscribe()
  {
    sub_.shutdown();
    // clear clear clear...
  }

  void OverlayImageDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = it_->subscribe(topic_name, 1, &OverlayImageDisplay::processMessage, this);
    }
  }

  void OverlayImageDisplay::processMessage(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    msg_ = msg;
    require_update_ = true;
  }
  

  void OverlayImageDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock(mutex_);

    if (require_update_) {
      if (panel_material_.isNull()) {
        Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
        panel_material_
          = Ogre::MaterialManager::getSingleton().create(material_name_,
                                                         Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
        panel_->setMaterialName(panel_material_->getName());
        panel_->setMetricsMode(Ogre::GMM_PIXELS);
        overlay_->add2D(panel_);
      }
      updateTextureSize(msg_->width, msg_->height);
      redraw();
      require_update_ = false;
    }
    panel_->setDimensions(width_, height_);
    panel_->setPosition(left_, top_);
  }

  void OverlayImageDisplay::redraw()
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_, sensor_msgs::image_encodings::RGB8);
      cv::Mat mat = cv_ptr->image;
      Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
      pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
      const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
      Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );
      memset( pDest, 0, texture_->getWidth() * texture_->getHeight() );
      QImage Hud( pDest, texture_->getWidth(), texture_->getHeight(), QImage::Format_ARGB32 );
      for (int i = 0; i < texture_->getWidth(); i++) {
        for (int j = 0; j < texture_->getHeight(); j++) {
          QColor color(mat.data[j * mat.step + i * mat.elemSize() + 0],
                       mat.data[j * mat.step + i * mat.elemSize() + 1],
                       mat.data[j * mat.step + i * mat.elemSize() + 2],
                       alpha_ * 255.0);
          Hud.setPixel(i, j, color.rgba());
        }
      }
    pixelBuffer->unlock();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  
  void OverlayImageDisplay::updateTextureSize(int width, int height)
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
      ROS_DEBUG("texture size: (%d, %d)", width, height);
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

  
  void OverlayImageDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
  
  void OverlayImageDisplay::updateWidth()
  {
    boost::mutex::scoped_lock lock(mutex_);
    width_ = width_property_->getInt();
  }
  
  void OverlayImageDisplay::updateHeight()
  {
    boost::mutex::scoped_lock lock(mutex_);
    height_ = height_property_->getInt();
  }

  void OverlayImageDisplay::updateTop()
  {
    boost::mutex::scoped_lock lock(mutex_);
    top_ = top_property_->getInt();
  }

  void OverlayImageDisplay::updateLeft()
  {
    boost::mutex::scoped_lock lock(mutex_);
    left_ = left_property_->getInt();
  }
  
  void OverlayImageDisplay::updateAlpha()
  {
    boost::mutex::scoped_lock lock(mutex_);
    alpha_ = alpha_property_->getFloat();
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayImageDisplay, rviz::Display )
