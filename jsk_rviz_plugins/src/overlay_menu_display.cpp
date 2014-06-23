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

#include "overlay_menu_display.h"

#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreTechnique.h>

#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>

namespace jsk_rviz_plugin
{

  const int menu_padding_x = 100;
  const int menu_padding_y = 5;
  const int menu_last_padding_y = 30;
  const double animate_duration = 0.2;
  OverlayMenuDisplay::OverlayMenuDisplay() : Display()
  {
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<jsk_rviz_plugins::OverlayMenu>(),
      "jsk_rviz_plugins::OverlayMenu topic to subscribe to.",
      this, SLOT( updateTopic() ));
 
  }

  OverlayMenuDisplay::~OverlayMenuDisplay()
  {
    onDisable();
    delete update_topic_property_;
  }

  void OverlayMenuDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "OverlayMenuDisplayObject" << count++;
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(ss.str());
    //panel_ = static_cast<Ogre::OverlayContainer*> (
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("BorderPanel", ss.str() + "Panel"));
    material_name_ = ss.str() + "Material";
    texture_name_ = ss.str() + "Texture";
    require_update_texture_ = false;
    animation_state_ = CLOSED;
  }
  
  void OverlayMenuDisplay::onEnable()
  {
    overlay_->show();
    subscribe();
  }
  void OverlayMenuDisplay::onDisable()
  {
    overlay_->hide();
    unsubscribe();
  }

  void OverlayMenuDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void OverlayMenuDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = ros::NodeHandle().subscribe(topic_name, 1,
                                         &OverlayMenuDisplay::processMessage,
                                         this);
    }
  }

  void OverlayMenuDisplay::processMessage
  (const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    next_menu_ = msg;
  }

  bool OverlayMenuDisplay::isNeedToResize()
  {
    if (!current_menu_ && next_menu_) { // first time
      return true;
    }
    else if (!current_menu_ && !next_menu_) {
      return false;
    }
    else if (current_menu_ && !next_menu_) {
      return false;
    }
    else {
      if (current_menu_->menus.size() != next_menu_->menus.size()) {
        return true;
      }
      else {
        // check all the menu is same or not
        for (size_t i = 0; i < current_menu_->menus.size(); i++) {
          if (current_menu_->menus[i] != next_menu_->menus[i]) {
            return true;
          }
        }
        return false;
      }
    }
  }

  QFont OverlayMenuDisplay::font()
  {
    QFont font;
    font.setPointSize(20);
    return font;
  }
  
  QFontMetrics OverlayMenuDisplay::fontMetrics()
  {
    QFontMetrics fm(font());
    return fm;
  }
  
  int OverlayMenuDisplay::drawAreaWidth(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    QFontMetrics fm = fontMetrics();
    int max_width = 0;
    for (size_t i = 0; i < msg->menus.size(); i++) {
      int w = fm.width(getMenuString(msg, i).c_str());
      if (max_width < w) {
        max_width = w;
      }
    }
    return max_width + menu_padding_x * 2;
  }

  int OverlayMenuDisplay::drawAreaHeight(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    QFontMetrics fm = fontMetrics();
    return fm.height() * (msg->menus.size() + 1)
      + menu_padding_y * (msg->menus.size() + 1 - 1)
      + menu_last_padding_y * 2;
  }
  
  void OverlayMenuDisplay::updateTextureSize(int width, int height)
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

  
  void OverlayMenuDisplay::update(float wall_dt, float ros_dt)
  {
    if (!next_menu_) {
      ROS_DEBUG("next_menu_ is null, no need to update");
      return;
    }
    //ROS_DEBUG_STREAM("wall_dt: " << wall_dt);
    if (panel_material_.isNull()) {
      Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
      panel_material_
        = Ogre::MaterialManager::getSingleton().create(material_name_,
                                                       Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
      panel_->setMaterialName(panel_material_->getName());
      panel_->setMetricsMode(Ogre::GMM_PIXELS);
      overlay_->add2D(panel_);
    }
    
    if (isNeedToResize()) {
      updateTextureSize(drawAreaWidth(next_menu_), drawAreaHeight(next_menu_));
    }
    else {
      ROS_DEBUG("no need to update texture size");
    }
    if (texture_.isNull()) {
      ROS_WARN("failed to create texture");
      return;
    }
    if (next_menu_->action == jsk_rviz_plugins::OverlayMenu::ACTION_CLOSE) {
      // need to close...
      if (animation_state_ == CLOSED) {
        // do nothing
      }
      else if (animation_state_ == OPENED) {
        animation_state_ = CLOSING;
        animation_t_ = animate_duration;
      }
      else if (animation_state_ == CLOSING) {
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) {
          openingAnimation();
        }
        else {
          animation_state_ = CLOSED;
        }
      }
      else if (animation_state_ == OPENING) {
        animation_t_ += wall_dt;
        if (animation_t_ < animate_duration) {
          openingAnimation();
        }
        else {
          redraw();
          animation_state_ = OPENED;
        }
      }
    }
    else {
      if (animation_state_ == CLOSED) {
        animation_t_ = 0.0;
        animation_state_ = OPENING;
      }
      else if (animation_state_ == OPENING) {
        animation_t_ += wall_dt;
        if (animation_t_ < animate_duration) {
          openingAnimation();
        }
        else {
          redraw();
          animation_state_ = OPENED;
        }
      }
      else if (animation_state_ == OPENED) {
        if (isNeedToRedraw()) {
          redraw();
        }
      }
      else if (animation_state_ == CLOSING) {
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) {
          openingAnimation();
        }
        else {
          animation_state_ = CLOSED;
        }
      }
    }
    current_menu_ = next_menu_;
  }

  bool OverlayMenuDisplay::isNeedToRedraw() {
    return true;
  }

  std::string OverlayMenuDisplay::getMenuString(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg,
    size_t index)
  {
    if (index >= msg->menus.size()) {
      return "";
    }
    else {
      return msg->menus[index];
    }
  }

  void OverlayMenuDisplay::openingAnimation()
  {
    int current_width = animation_t_ / animate_duration * texture_->getWidth();
    int current_height = animation_t_ / animate_duration * texture_->getHeight();
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    QColor bg_color(0, 0, 0, 255.0 / 2.0);
    QColor transparent(0, 0, 0, 0.0);
    pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );
    memset( pDest, 0, texture_->getWidth() * texture_->getHeight() );
    QImage Hud( pDest, texture_->getWidth(), texture_->getHeight(), QImage::Format_ARGB32 );
    
    for (int i = 0; i < texture_->getWidth(); i++) {
      for (int j = 0; j < texture_->getHeight(); j++) {
        if (i > (texture_->getWidth() - current_width) / 2.0 &&
            i < texture_->getWidth() - (texture_->getWidth() - current_width) / 2.0 &&
            j > (texture_->getHeight() - current_height) / 2.0 &&
            j < texture_->getHeight() - (texture_->getHeight() - current_height) / 2.0) {
          Hud.setPixel(i, j, bg_color.rgba());
        }
        else {
          Hud.setPixel(i, j, transparent.rgba());
        }
      }
    }
    pixelBuffer->unlock();
    panel_->setDimensions(texture_->getWidth(), texture_->getHeight());
    int window_width = context_->getViewManager()->getRenderPanel()->width();
    int window_height = context_->getViewManager()->getRenderPanel()->height();
    panel_->setPosition((window_width - texture_->getWidth()) / 2.0,
                        (window_height - texture_->getHeight()) / 2.0);
  }
  
  void OverlayMenuDisplay::redraw()
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    QColor bg_color(0, 0, 0, 255.0 / 2.0);
    QColor fg_color(25, 255, 240, 255.0);
    pixelBuffer->lock( Ogre::HardwareBuffer::HBL_NORMAL ); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> ( pixelBox.data );
    memset( pDest, 0, texture_->getWidth() * texture_->getHeight() );
    QImage Hud( pDest, texture_->getWidth(), texture_->getHeight(), QImage::Format_ARGB32 );
    for (int i = 0; i < texture_->getWidth(); i++) {
      for (int j = 0; j < texture_->getHeight(); j++) {
        Hud.setPixel(i, j, bg_color.rgba());
      }
    }
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color, 1, Qt::SolidLine));
    painter.setFont(font());
    int line_height = fontMetrics().height();
    int w = drawAreaWidth(next_menu_);
    painter.drawText(menu_padding_x,  menu_padding_y,
                     w, line_height,
                     Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                     next_menu_->title.c_str());
    for (size_t i = 0; i < next_menu_->menus.size(); i++) {
      std::string menu = getMenuString(next_menu_, i);
      painter.drawText(menu_padding_x, line_height * ( 1 + i ) + menu_padding_y + menu_last_padding_y,
                       w, line_height,
                       Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                       menu.c_str());
    }
    if (next_menu_->current_index <= next_menu_->menus.size()) {
      // draw '>'
      painter.drawText(menu_padding_x - fontMetrics().width(">") * 2,
                       line_height * ( 1 + next_menu_->current_index ) + menu_padding_y + menu_last_padding_y,
                       w, line_height,
                       Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                       ">");
    }
    // draw line
    int texture_width = texture_->getWidth();
    int texture_height = texture_->getHeight();
    painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                     menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
    painter.drawLine(texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                     texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
    painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                     texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height);
    painter.drawLine(menu_padding_x / 2, texture_height - menu_last_padding_y / 2,
                     texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      
    painter.end();
    pixelBuffer->unlock();
    panel_->setDimensions(texture_->getWidth(), texture_->getHeight());
    int window_width = context_->getViewManager()->getRenderPanel()->width();
    int window_height = context_->getViewManager()->getRenderPanel()->height();
    panel_->setPosition((window_width - texture_->getWidth()) / 2.0,
                        (window_height - texture_->getHeight()) / 2.0);
  }
  
  void OverlayMenuDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugin::OverlayMenuDisplay, rviz::Display )
