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

#include "overlay_text_display.h"
#include <OGRE/OgreMaterialManager.h>
#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <regex>

namespace jsk_rviz_plugins
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
    overtake_position_properties_property_ = new rviz::BoolProperty(
      "Overtake Position Properties", false,
      "overtake position properties specified by message such as left, top and font",
      this, SLOT(updateOvertakePositionProperties()));
    overtake_fg_color_properties_property_ = new rviz::BoolProperty(
      "Overtake FG Color Properties", false,
      "overtake color properties specified by message such as foreground color and alpha",
      this, SLOT(updateOvertakeFGColorProperties()));
    overtake_bg_color_properties_property_ = new rviz::BoolProperty(
      "Overtake BG Color Properties", false,
      "overtake color properties specified by message such as background color and alpha",
      this, SLOT(updateOvertakeBGColorProperties()));
    align_bottom_property_ = new rviz::BoolProperty(
      "Align Bottom", false,
      "align text with the bottom of the overlay region",
      this, SLOT(updateAlignBottom()));
    invert_shadow_property_ = new rviz::BoolProperty(
      "Invert Shadow", false,
      "make shadow lighter than original text",
      this, SLOT(updateInvertShadow()));
    top_property_ = new rviz::IntProperty(
      "top", 0,
      "top position",
      this, SLOT(updateTop()));
    top_property_->setMin(0);
    left_property_ = new rviz::IntProperty(
      "left", 0,
      "left position",
      this, SLOT(updateLeft()));
    left_property_->setMin(0);
    width_property_ = new rviz::IntProperty(
      "width", 128,
      "width position",
      this, SLOT(updateWidth()));
    width_property_->setMin(0);
    height_property_ = new rviz::IntProperty(
      "height", 128,
      "height position",
      this, SLOT(updateHeight()));
    height_property_->setMin(0);
    text_size_property_ = new rviz::IntProperty(
      "text size", 12,
      "text size",
      this, SLOT(updateTextSize()));
    text_size_property_->setMin(0);
    line_width_property_ = new rviz::IntProperty(
      "line width", 2,
      "line width",
      this, SLOT(updateLineWidth()));
    line_width_property_->setMin(0);
    fg_color_property_ = new rviz::ColorProperty(
      "Foreground Color", QColor(25, 255, 240),
      "Foreground Color",
      this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty(
      "Foreground Alpha", 0.8, "Foreground Alpha",
      this, SLOT(updateFGAlpha()));
    fg_alpha_property_->setMin(0.0);
    fg_alpha_property_->setMax(1.0);
    bg_color_property_ = new rviz::ColorProperty(
      "Background Color", QColor(0, 0, 0),
      "Background Color",
      this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty(
      "Background Alpha", 0.8, "Background Alpha",
      this, SLOT(updateBGAlpha()));
    bg_alpha_property_->setMin(0.0);
    bg_alpha_property_->setMax(1.0);

    QFontDatabase database;
    font_families_ = database.families();
    font_property_ = new rviz::EnumProperty(
      "font", "DejaVu Sans Mono",
      "font", this,
      SLOT(updateFont()));
    for (size_t i = 0; i < font_families_.size(); i++) {
      font_property_->addOption(font_families_[i], (int)i);
    }
  }
  
  OverlayTextDisplay::~OverlayTextDisplay()
  {
    onDisable();
    //delete overlay_;
    delete update_topic_property_;
    delete overtake_fg_color_properties_property_;
    delete overtake_bg_color_properties_property_;
    delete overtake_position_properties_property_;
    delete align_bottom_property_;
    delete invert_shadow_property_;
    delete top_property_;
    delete left_property_;
    delete width_property_;
    delete height_property_;
    delete text_size_property_;
    delete line_width_property_;
    delete bg_color_property_;
    delete bg_alpha_property_;
    delete fg_color_property_;
    delete fg_alpha_property_;
    delete font_property_;
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
    updateOvertakePositionProperties();
    updateOvertakeFGColorProperties();
    updateOvertakeBGColorProperties();
    updateAlignBottom();
    updateInvertShadow();
    updateTop();
    updateLeft();
    updateWidth();
    updateHeight();
    updateTextSize();
    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();
    updateFont();
    updateLineWidth();
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
    {
      jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_, bg_color_);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color_, std::max(line_width_,1), Qt::SolidLine));
      uint16_t w = overlay_->getTextureWidth();
      uint16_t h = overlay_->getTextureHeight();

      // font
      if (text_size_ != 0) {
        //QFont font = painter.font();
        QFont font(font_.length() > 0 ? font_.c_str(): "Liberation Sans");
        font.setPointSize(text_size_);
        font.setBold(true);
        painter.setFont(font);
      }
      if (text_.length() > 0) {

	QColor shadow_color;
	if (invert_shadow_)
	  shadow_color = Qt::white; //fg_color_.lighter();
	else
	  shadow_color = Qt::black; //fg_color_.darker();
	shadow_color.setAlpha(fg_color_.alpha());
	
	std::string color_wrapped_text
          = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % text_ % fg_color_.red() % fg_color_.green() % fg_color_.blue() %
             fg_color_.alpha()).str();

        // find a remove "color: XXX;" regex match to generate a proper shadow 
        std::regex color_tag_re("color:.+?;");
        std::string null_char("");
        std::string formatted_text_ = std::regex_replace(text_, color_tag_re, null_char);
        std::string color_wrapped_shadow
          = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % formatted_text_ % shadow_color.red() % shadow_color.green() % shadow_color.blue() % shadow_color.alpha()).str();
	
        QStaticText static_text(
          boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());
        static_text.setTextWidth(w);

	painter.setPen(QPen(shadow_color, std::max(line_width_,1), Qt::SolidLine));
        QStaticText static_shadow(
          boost::algorithm::replace_all_copy(color_wrapped_shadow, "\n", "<br >").c_str());
        static_shadow.setTextWidth(w);

	if (!align_bottom_) {
	  painter.drawStaticText(1, 1, static_shadow);
          painter.drawStaticText(0, 0, static_text);
        } else {
          QStaticText only_wrapped_text(color_wrapped_text.c_str());
          QFontMetrics fm(painter.fontMetrics());
          QRect text_rect = fm.boundingRect(0, 0, w, h,
                                            Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                                            only_wrapped_text.text().remove(QRegExp("<[^>]*>")));
          painter.drawStaticText(1, h - text_rect.height()+1, static_shadow);
          painter.drawStaticText(0, h - text_rect.height(), static_text);
        }
      }
      painter.end();
    }
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
      overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));
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
    
    // store message for update method
    text_ = msg->text;
    if (!overtake_position_properties_) {
      texture_width_ = msg->width;
      texture_height_ = msg->height;
      text_size_ = msg->text_size;
      left_ = msg->left;
      top_ = msg->top;
    }
    if (!overtake_bg_color_properties_) 
      bg_color_ = QColor(msg->bg_color.r * 255.0,
                         msg->bg_color.g * 255.0,
                         msg->bg_color.b * 255.0,
                         msg->bg_color.a * 255.0);
    if (!overtake_fg_color_properties_) {
      fg_color_ = QColor(msg->fg_color.r * 255.0,
                         msg->fg_color.g * 255.0,
                         msg->fg_color.b * 255.0,
                         msg->fg_color.a * 255.0);
      font_ = msg->font;
      line_width_ = msg->line_width;
    }
    if (overlay_) {
      overlay_->setPosition(left_, top_);
    }
    require_update_texture_ = true;
  }

  void OverlayTextDisplay::updateOvertakePositionProperties()
  {
    
    if (!overtake_position_properties_ &&
        overtake_position_properties_property_->getBool()) {
      updateTop();
      updateLeft();
      updateWidth();
      updateHeight();
      updateTextSize();
      require_update_texture_ = true;
    }
    overtake_position_properties_
      = overtake_position_properties_property_->getBool();
    if (overtake_position_properties_) {
      top_property_->show();
      left_property_->show();
      width_property_->show();
      height_property_->show();
      text_size_property_->show();
    }
    else {
      top_property_->hide();
      left_property_->hide();
      width_property_->hide();
      height_property_->hide();
      text_size_property_->hide();
    }
  }
  
  void OverlayTextDisplay::updateOvertakeFGColorProperties()
  {
    if (!overtake_fg_color_properties_ &&
        overtake_fg_color_properties_property_->getBool()) {
      // read all the parameters from properties
      updateFGColor();
      updateFGAlpha();
      updateFont();
      updateLineWidth();
      require_update_texture_ = true;
    }
    overtake_fg_color_properties_ = overtake_fg_color_properties_property_->getBool();
    if (overtake_fg_color_properties_) {
      fg_color_property_->show();
      fg_alpha_property_->show();
      line_width_property_->show();
      font_property_->show();
    }
    else {
      fg_color_property_->hide();
      fg_alpha_property_->hide();
      line_width_property_->hide();
      font_property_->hide();
    }
  }

  void OverlayTextDisplay::updateOvertakeBGColorProperties()
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
  
  void OverlayTextDisplay::updateAlignBottom()
  {
    if (align_bottom_ != align_bottom_property_->getBool()) {
      require_update_texture_ = true;
    }
    align_bottom_ = align_bottom_property_->getBool();
  }

  void OverlayTextDisplay::updateInvertShadow()
  {
    if (invert_shadow_ != invert_shadow_property_->getBool()) {
      require_update_texture_ = true;
    }
    invert_shadow_ = invert_shadow_property_->getBool();
  }

  
  void OverlayTextDisplay::updateTop()
  {
    top_ = top_property_->getInt();
    if (overtake_position_properties_) {
      require_update_texture_ = true;
    }
  }
  
  void OverlayTextDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
    if (overtake_position_properties_) {
      require_update_texture_ = true;
    }
  }
  
  void OverlayTextDisplay::updateWidth()
  {
    texture_width_ = width_property_->getInt();
    if (overtake_position_properties_) {
      require_update_texture_ = true;
    }
  }
  
  void OverlayTextDisplay::updateHeight()
  {
    texture_height_ = height_property_->getInt();
    if (overtake_position_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateTextSize()
  {
    text_size_ = text_size_property_->getInt();
    if (overtake_position_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateBGColor()
  {
    QColor c = bg_color_property_->getColor();
    bg_color_.setRed(c.red());
    bg_color_.setGreen(c.green());
    bg_color_.setBlue(c.blue());
    if (overtake_bg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateBGAlpha()
  {
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
    if (overtake_bg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateFGColor()
  {
    QColor c = fg_color_property_->getColor();
    fg_color_.setRed(c.red());
    fg_color_.setGreen(c.green());
    fg_color_.setBlue(c.blue());
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateFGAlpha()
  {
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateFont()
  {
    int font_index = font_property_->getOptionInt();
    if (font_index < font_families_.size()) {
      font_ = font_families_[font_index].toStdString();
    } else {
      ROS_FATAL("Unexpected error at selecting font index %d.", font_index);
      return;
    }
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  void OverlayTextDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getInt();
    if (overtake_fg_color_properties_) {
      require_update_texture_ = true;
    }
  }

  bool OverlayTextDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + texture_height_ > y &&
            left_ < x && left_ + texture_width_ > x);
  }

  void OverlayTextDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
  }

  void OverlayTextDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayTextDisplay, rviz::Display )
