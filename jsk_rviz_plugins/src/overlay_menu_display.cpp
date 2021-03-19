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
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreTechnique.h>

#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>

namespace jsk_rviz_plugins
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
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the image window",
                                           this, SLOT(updateLeft()));
    left_property_->setMin(0);
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the image window",
                                          this, SLOT(updateTop()));
    top_property_->setMin(0);
    keep_centered_property_ = new rviz::BoolProperty("keep centered", true,
                                                     "enable automatic center adjustment",
                                                     this, SLOT(updateKeepCentered()));

    // NOTE: Overtaking FG/BG Color Properties defaults to TRUE for backward compatibility.
    overtake_fg_color_properties_property_ = new rviz::BoolProperty(
      "Overtake FG Color Properties", true,
      "overtake color properties specified by message such as foreground color and alpha",
      this, SLOT(updateOvertakeFGColorProperties()));
    overtake_bg_color_properties_property_ = new rviz::BoolProperty(
      "Overtake BG Color Properties", true,
      "overtake color properties specified by message such as background color and alpha",
      this, SLOT(updateOvertakeBGColorProperties()));

    fg_color_property_ = new rviz::ColorProperty(
      "Foreground Color", QColor(25, 255, 240),
      "Foreground Color",
      this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty(
      "Foreground Alpha", 1.0, "Foreground Alpha",
      this, SLOT(updateFGAlpha()));
    fg_alpha_property_->setMin(0.0);
    fg_alpha_property_->setMax(1.0);

    bg_color_property_ = new rviz::ColorProperty(
      "Background Color", QColor(0, 0, 0),
      "Background Color",
      this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty(
      "Background Alpha", 0.5, "Background Alpha",
      this, SLOT(updateBGAlpha()));
    bg_alpha_property_->setMin(0.0);
    bg_alpha_property_->setMax(1.0);
  }

  OverlayMenuDisplay::~OverlayMenuDisplay()
  {
    onDisable();
    delete update_topic_property_;
    delete left_property_;
    delete top_property_;
    delete keep_centered_property_;
    delete overtake_fg_color_properties_property_;
    delete overtake_bg_color_properties_property_;
    delete bg_color_property_;
    delete bg_alpha_property_;
    delete fg_color_property_;
    delete fg_alpha_property_;
  }

  void OverlayMenuDisplay::onInitialize()
  {
    updateKeepCentered();
    updateLeft();
    updateTop();
    updateOvertakeFGColorProperties();
    updateOvertakeBGColorProperties();
    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();
    require_update_texture_ = false;
    animation_state_ = CLOSED;
  }
  
  void OverlayMenuDisplay::onEnable()
  {
    if (overlay_) {
      overlay_->show();
    }
    subscribe();
  }

  void OverlayMenuDisplay::onDisable()
  {
    if (overlay_) {
      overlay_->hide();
    }
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
    if (!overtake_bg_color_properties_)
      bg_color_ = QColor(msg->bg_color.r * 255.0,
                         msg->bg_color.g * 255.0,
                         msg->bg_color.b * 255.0,
                         msg->bg_color.a * 255.0);
    if (!overtake_fg_color_properties_)
      fg_color_ = QColor(msg->fg_color.r * 255.0,
                         msg->fg_color.g * 255.0,
                         msg->fg_color.b * 255.0,
                         msg->fg_color.a * 255.0);
  }

  bool OverlayMenuDisplay::isNeedToResize()
  {
    if (!current_menu_ && next_menu_) { // first time
      ROS_DEBUG("need to resize because this is the first time to draw");
      return true;
    }
    else if (!current_menu_ && !next_menu_) {
      // both are null, it means that ...
      // the plugin tries to draw without message reception
      ROS_DEBUG("no need to resize because the plugin tries to draw without message reception");
      return false;
    }
    else if (current_menu_ && !next_menu_) {
      // this is unexpected case
      ROS_DEBUG("no need to resize, this is unexpected case. please debug");
      return false;
    }
    else {
      if (current_menu_->menus.size() != next_menu_->menus.size()) {
        ROS_DEBUG("need to resize because the length of menu is different");
        return true;
      }
      else if (current_menu_->title != next_menu_->title) {
        return true;
      }
      else {
        // check all the menu is same or not
        for (size_t i = 0; i < current_menu_->menus.size(); i++) {
          if (current_menu_->menus[i] != next_menu_->menus[i]) {
            ROS_DEBUG("need to resize because the content of menu is different");
            return true;
          }
        }
        ROS_DEBUG("no need to resize because the content of menu is same");
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
    int w = fm.width(msg->title.c_str());
    
    if (max_width < w) {
      max_width = w;
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
  
  void OverlayMenuDisplay::update(float wall_dt, float ros_dt)
  {
    if (!next_menu_) {
      ROS_DEBUG("next_menu_ is null, no need to update");
      return;
    }
    if (next_menu_->action == jsk_rviz_plugins::OverlayMenu::ACTION_CLOSE &&
        animation_state_ == CLOSED) {
      ROS_DEBUG("request is close and state is closed, we ignore it completely");
      return;
    }

    if (next_menu_->action == jsk_rviz_plugins::OverlayMenu::ACTION_CLOSE) {
      // need to close...
      if (animation_state_ == CLOSED) {
        // do nothing, it should be ignored above if sentence
        ROS_WARN("request is CLOSE and state is CLOSED, it should be ignored before...");
      }
      else if (animation_state_ == OPENED) { // OPENED -> CLOSING
        animation_state_ = CLOSING;
        animation_t_ = animate_duration;
      }
      else if (animation_state_ == CLOSING) {
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) { // CLOSING -> CLOSING
          openingAnimation();
        }
        else { // CLOSING -> CLOSED
          animation_t_ = 0;
          openingAnimation();
          animation_state_ = CLOSED;
        }
      }
      else if (animation_state_ == OPENING) { // if the status is OPENING, we open it anyway...??
        animation_t_ += wall_dt;
        if (animation_t_ < animate_duration) { // OPENING -> OPENING
          openingAnimation();
        }
        else {                  // OPENING -> OPENED
          redraw();
          animation_state_ = OPENED;
        }
      }
    }
    else {                      // OPEN request
      if (animation_state_ == CLOSED) { // CLOSED -> OPENING, do nothing just change the state
        animation_t_ = 0.0;
        animation_state_ = OPENING;
      }
      else if (animation_state_ == OPENING) {
        animation_t_ += wall_dt;
        ROS_DEBUG("animation_t: %f", animation_t_);
        if (animation_t_ < animate_duration) { // OPENING -> OPENING
          openingAnimation();
        }
        else {                  // OPENING -> OPENED
          redraw();
          animation_state_ = OPENED;
        }
      }
      else if (animation_state_ == OPENED) { // OPENED -> OPENED
        if (isNeedToRedraw()) {
          redraw();
        }
      }
      else if (animation_state_ == CLOSING) { // CLOSING, we close it anyway...
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) {
          openingAnimation();
        }
        else {
          animation_t_ = 0;
          openingAnimation();
          animation_state_ = CLOSED;
        }
      }
    }
    //redraw();
    //current_menu_ = next_menu_;
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

  void OverlayMenuDisplay::prepareOverlay()
  {
    if (!overlay_) {
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "OverlayMenuDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    if (!overlay_->isTextureReady() || isNeedToResize()) {
      overlay_->updateTextureSize(drawAreaWidth(next_menu_), drawAreaHeight(next_menu_));
    }
    else {
      ROS_DEBUG("no need to update texture size");
    }
  }
  
  void OverlayMenuDisplay::openingAnimation()
  {
    ROS_DEBUG("openningAnimation");
    prepareOverlay();
    int current_width = animation_t_ / animate_duration * overlay_->getTextureWidth();
    int current_height = animation_t_ / animate_duration * overlay_->getTextureHeight();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QColor transparent(0, 0, 0, 0.0);
      QImage Hud = buffer.getQImage(*overlay_);
      for (int i = 0; i < overlay_->getTextureWidth(); i++) {
        for (int j = 0; j < overlay_->getTextureHeight(); j++) {
          if (i > (overlay_->getTextureWidth() - current_width) / 2.0 &&
              i < overlay_->getTextureWidth() - (overlay_->getTextureWidth() - current_width) / 2.0 &&
              j > (overlay_->getTextureHeight() - current_height) / 2.0 &&
              j < overlay_->getTextureHeight() - (overlay_->getTextureHeight() - current_height) / 2.0) {
            Hud.setPixel(i, j, bg_color_.rgba());
          }
          else {
            Hud.setPixel(i, j, transparent.rgba());
          }
        }
      }
    }
    setMenuLocation();
    current_menu_ = next_menu_;
  }
  
  void OverlayMenuDisplay::redraw()
  {
    ROS_DEBUG("redraw");
    prepareOverlay();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_, bg_color_);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color_, 1, Qt::SolidLine));
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
      int texture_width = overlay_->getTextureWidth();
      int texture_height = overlay_->getTextureHeight();
      painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      painter.drawLine(texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height);
      painter.drawLine(menu_padding_x / 2, texture_height - menu_last_padding_y / 2,
                       texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      
      painter.end();
      current_menu_ = next_menu_;
    }
    setMenuLocation();
  }

  void OverlayMenuDisplay::setMenuLocation()
  {
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    int window_width = context_->getViewManager()->getRenderPanel()->width();
    int window_height = context_->getViewManager()->getRenderPanel()->height();
    if (keep_centered_)
    {
      left_ = (window_width - (int)overlay_->getTextureWidth()) / 2.0;
      top_ = (window_height - (int)overlay_->getTextureHeight()) / 2.0;
    }
    left_ = std::max(0, std::min(window_width - (int)overlay_->getTextureWidth(), left_));
    top_  = std::max(0, std::min(window_height - (int)overlay_->getTextureHeight(), top_));
    overlay_->setPosition(left_, top_);
  }
  
  void OverlayMenuDisplay::updateTopic()
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsubscribe();
    subscribe();
  }

  void OverlayMenuDisplay::updateLeft()
  {
    boost::mutex::scoped_lock lock(mutex_);
    left_ = left_property_->getInt();
  }

  void OverlayMenuDisplay::updateTop()
  {
    boost::mutex::scoped_lock lock(mutex_);
    top_ = top_property_->getInt();
  }

  void OverlayMenuDisplay::updateKeepCentered()
  {
    if (keep_centered_ &&
        !keep_centered_property_->getBool()) {
      updateLeft();
      updateTop();
    }
    boost::mutex::scoped_lock lock(mutex_);
    keep_centered_ = keep_centered_property_->getBool();
  }

  void OverlayMenuDisplay::updateOvertakeFGColorProperties()
  {
    if (!overtake_fg_color_properties_ &&
        overtake_fg_color_properties_property_->getBool()) {
      // read all the parameters from properties
      updateFGColor();
      updateFGAlpha();
      require_update_texture_ = true;
    }
    overtake_fg_color_properties_ = overtake_fg_color_properties_property_->getBool();
    if (overtake_fg_color_properties_) {
      fg_color_property_->show();
      fg_alpha_property_->show();
    }
    else {
      fg_color_property_->hide();
      fg_alpha_property_->hide();
    }
  }

  void OverlayMenuDisplay::updateOvertakeBGColorProperties()
  {
    if (!overtake_bg_color_properties_ &&
        overtake_bg_color_properties_property_->getBool()) {
      // read all the parameters from properties
      updateBGColor();
      updateBGAlpha();
      require_update_texture_ = true;
    }
    overtake_bg_color_properties_ = overtake_bg_color_properties_property_->getBool();
    if (overtake_bg_color_properties_) {
      bg_color_property_->show();
      bg_alpha_property_->show();
    }
    else {
      bg_color_property_->hide();
      bg_alpha_property_->hide();
    }
  }

  void OverlayMenuDisplay::updateFGColor()
  {
    QColor c = fg_color_property_->getColor();
    fg_color_.setRed(c.red());
    fg_color_.setGreen(c.green());
    fg_color_.setBlue(c.blue());
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayMenuDisplay::updateFGAlpha()
  {
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayMenuDisplay::updateBGColor()
  {
    QColor c = bg_color_property_->getColor();
    bg_color_.setRed(c.red());
    bg_color_.setGreen(c.green());
    bg_color_.setBlue(c.blue());
    if (overtake_bg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayMenuDisplay::updateBGAlpha()
  {
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
    if (overtake_bg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  bool OverlayMenuDisplay::isInRegion(int x, int y)
  {
    return (overlay_ && overlay_->isTextureReady() &&
            top_ < y && top_ + overlay_->getTextureHeight() > y &&
            left_ < x && left_ + overlay_->getTextureWidth() > x);
  }

  void OverlayMenuDisplay::movePosition(int x, int y)
  {
    if (!keep_centered_)
    {
      top_ = y;
      left_ = x;
    }
  }

  void OverlayMenuDisplay::setPosition(int x, int y)
  {
    if (!keep_centered_)
    {
      top_property_->setValue(y);
      left_property_->setValue(x);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayMenuDisplay, rviz::Display )
