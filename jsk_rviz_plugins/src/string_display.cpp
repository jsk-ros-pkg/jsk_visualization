// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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

#include "string_display.h"

#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/render_system.hpp>
#include <QFontDatabase>
#include <QPainter>
#include <QRegularExpression>
#include <QStaticText>
#include <rviz_common/uniform_string_stream.hpp>

namespace jsk_rviz_plugins
{
  StringDisplay::StringDisplay() : Display(),
                                   texture_width_(0), texture_height_(0),
                                   overtake_color_properties_(false),
                                   overtake_position_properties_(false),
                                   align_bottom_(false),
                                   text_size_(14),
                                   line_width_(2),
                                   text_(""), font_(""),
                                   bg_color_(0, 0, 0, 0),
                                   fg_color_(255, 255, 255, 255.0),
                                   left_(0), top_(0),
                                   require_update_texture_(false)
  {
    update_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
      "Topic", "",
      rosidl_generator_traits::name<std_msgs::msg::String>(),
      "std_msgs::String topic to subscribe to.",
      this, SLOT(updateTopic()));
    overtake_position_properties_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Overtake Position Properties", false,
      "overtake position properties specified by message such as left, top and font",
      this, SLOT(updateOvertakePositionProperties()));
    overtake_color_properties_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Overtake Color Properties", false,
      "overtake color properties specified by message such as foreground/background color and alpha",
      this, SLOT(updateOvertakeColorProperties()));
    align_bottom_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
      "Align Bottom", false,
      "align text with the bottom of the overlay region",
      this, SLOT(updateAlignBottom()));
    top_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "top", 0,
      "top position",
      nullptr, SLOT(updateTop()), this);
    top_property_->setMin(0);
    left_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "left", 0,
      "left position",
      nullptr, SLOT(updateLeft()), this);
    left_property_->setMin(0);
    width_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "width", 128,
      "width position",
      nullptr, SLOT(updateWidth()), this);
    width_property_->setMin(0);
    height_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "height", 128,
      "height position",
      nullptr, SLOT(updateHeight()), this);
    height_property_->setMin(0);
    text_size_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "text size", 12,
      "text size",
      nullptr, SLOT(updateTextSize()), this);
    text_size_property_->setMin(0);

    // Color sub-properties: created without parent (not shown by default)
    line_width_property_ = std::make_unique<rviz_common::properties::IntProperty>(
      "line width", 2,
      "line width",
      nullptr, SLOT(updateLineWidth()), this);
    line_width_property_->setMin(0);
    fg_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Foreground Color", QColor(25, 255, 240),
      "Foreground Color",
      nullptr, SLOT(updateFGColor()), this);
    fg_alpha_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Foreground Alpha", 0.8, "Foreground Alpha",
      nullptr, SLOT(updateFGAlpha()), this);
    fg_alpha_property_->setMin(0.0);
    fg_alpha_property_->setMax(1.0);
    bg_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Background Color", QColor(0, 0, 0),
      "Background Color",
      nullptr, SLOT(updateBGColor()), this);
    bg_alpha_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Background Alpha", 0.8, "Background Alpha",
      nullptr, SLOT(updateBGAlpha()), this);
    bg_alpha_property_->setMin(0.0);
    bg_alpha_property_->setMax(1.0);

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // QFontDatabase is static-only from Qt6 on, its constructor was removed
    font_families_ = QFontDatabase::families();
#else
    QFontDatabase database;
    font_families_ = database.families();
#endif
    font_property_ = std::make_unique<rviz_common::properties::EnumProperty>(
      "font", "DejaVu Sans Mono",
      "font", nullptr,
      SLOT(updateFont()), this);
    for (size_t i = 0; i < font_families_.size(); i++)
    {
      font_property_->addOption(font_families_[i], static_cast<int>(i));
    }

  }

  StringDisplay::~StringDisplay()
  {
    onDisable();
    // Remove dynamically-added children before unique_ptrs destroy them,
    // to prevent Property::~Property() from double-deleting.
    takeChild(top_property_.get());
    takeChild(left_property_.get());
    takeChild(width_property_.get());
    takeChild(height_property_.get());
    takeChild(text_size_property_.get());
    takeChild(fg_color_property_.get());
    takeChild(fg_alpha_property_.get());
    takeChild(bg_color_property_.get());
    takeChild(bg_alpha_property_.get());
    takeChild(line_width_property_.get());
    takeChild(font_property_.get());
  }

  void StringDisplay::onEnable()
  {
    if (overlay_)
    {
      overlay_->show();
    }
    subscribe();
  }

  void StringDisplay::onDisable()
  {
    if (overlay_)
    {
      overlay_->hide();
    }
    unsubscribe();
  }

  void StringDisplay::unsubscribe()
  {
    sub_.reset();
  }

  void StringDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/")
    {
      auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
      sub_ = raw_node->create_subscription<std_msgs::msg::String>(topic_name, 1, std::bind(&StringDisplay::processMessage, this, std::placeholders::_1));
    }
  }

  void StringDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  // only the first time
  void StringDisplay::onInitialize()
  {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
    update_topic_property_->initialize(context_->getRosNodeAbstraction());
    onEnable();
    updateTopic();
    updateOvertakePositionProperties();
    updateOvertakeColorProperties();
    updateAlignBottom();
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

  void StringDisplay::update(float wall_dt, float ros_dt)
  {
    if (!require_update_texture_)
    {
      return;
    }
    if (!isEnabled())
    {
      return;
    }
    if (!overlay_)
    {
      static int count = 0;
      rviz_common::UniformStringStream ss;
      ss << "StringDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    overlay_->setPosition(left_, top_);
    overlay_->updateTextureSize(texture_width_, texture_height_);
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_, bg_color_);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));
      uint16_t w = overlay_->getTextureWidth();
      uint16_t h = overlay_->getTextureHeight();

      // font
      if (text_size_ != 0)
      {
        QFont font(font_.length() > 0 ? font_.c_str(): "Liberation Sans");
        font.setPointSize(text_size_);
        font.setBold(true);
        painter.setFont(font);
      }
      if (text_.length() > 0)
      {
        QString color_wrapped_text
          = QString("<span style=\"color: rgba(%1, %2, %3, %4)\">%5</span>")
            .arg(fg_color_.red()).arg(fg_color_.green()).arg(fg_color_.blue())
            .arg(fg_color_.alpha()).arg(QString::fromStdString(text_));
        QStaticText static_text(
          QString(color_wrapped_text).replace("\n", "<br >"));
        static_text.setTextWidth(w);
        if (!align_bottom_)
        {
          painter.drawStaticText(0, 0, static_text);
        }
        else
        {
          QStaticText only_wrapped_text(color_wrapped_text);
          QFontMetrics fm(painter.fontMetrics());
          QRect text_rect = fm.boundingRect(0, 0, w, h,
                                            Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                                            only_wrapped_text.text().remove(
                                              QRegularExpression("<[^>]*>")));
          painter.drawStaticText(0, h - text_rect.height(), static_text);
        }
      }
      painter.end();
    }
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    require_update_texture_ = false;
  }

  void StringDisplay::processMessage
  (const std_msgs::msg::String::SharedPtr msg)
  {
    if (!isEnabled())
    {
      return;
    }

    // store message for update method
    // Ogre objects (overlay_) must be created on the render thread (in update()),
    // not here on the ROS callback thread.
    text_ = msg->data;
    require_update_texture_ = true;
  }

  void StringDisplay::updateOvertakePositionProperties()
  {
    bool new_val = overtake_position_properties_property_->getBool();
    if (!overtake_position_properties_ && new_val)
    {
      updateTop();
      updateLeft();
      updateWidth();
      updateHeight();
      updateTextSize();
      require_update_texture_ = true;
    }
    if (new_val && !overtake_position_properties_)
    {
      // Add properties to the Display tree
      addChild(top_property_.get());
      addChild(left_property_.get());
      addChild(width_property_.get());
      addChild(height_property_.get());
      addChild(text_size_property_.get());
    }
    else if (!new_val && overtake_position_properties_)
    {
      // Remove properties from the Display tree
      takeChild(top_property_.get());
      takeChild(left_property_.get());
      takeChild(width_property_.get());
      takeChild(height_property_.get());
      takeChild(text_size_property_.get());
    }
    overtake_position_properties_ = new_val;
  }

  void StringDisplay::updateOvertakeColorProperties()
  {
    bool new_val = overtake_color_properties_property_->getBool();
    if (!overtake_color_properties_ && new_val)
    {
      // read all the parameters from properties
      updateFGColor();
      updateFGAlpha();
      updateBGColor();
      updateBGAlpha();
      updateFont();
      updateLineWidth();
      require_update_texture_ = true;
    }
    if (new_val && !overtake_color_properties_)
    {
      // Add properties to the Display tree
      addChild(fg_color_property_.get());
      addChild(fg_alpha_property_.get());
      addChild(bg_color_property_.get());
      addChild(bg_alpha_property_.get());
      addChild(line_width_property_.get());
      addChild(font_property_.get());
    }
    else if (!new_val && overtake_color_properties_)
    {
      // Remove properties from the Display tree
      takeChild(fg_color_property_.get());
      takeChild(fg_alpha_property_.get());
      takeChild(bg_color_property_.get());
      takeChild(bg_alpha_property_.get());
      takeChild(line_width_property_.get());
      takeChild(font_property_.get());
    }
    overtake_color_properties_ = new_val;
  }

  void StringDisplay::updateAlignBottom()
  {
    if (align_bottom_ != align_bottom_property_->getBool())
    {
      require_update_texture_ = true;
    }
    align_bottom_ = align_bottom_property_->getBool();
  }

  void StringDisplay::updateTop()
  {
    top_ = top_property_->getInt();
    if (overtake_position_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
    if (overtake_position_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateWidth()
  {
    texture_width_ = width_property_->getInt();
    if (overtake_position_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateHeight()
  {
    texture_height_ = height_property_->getInt();
    if (overtake_position_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateTextSize()
  {
    text_size_ = text_size_property_->getInt();
    if (overtake_position_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateBGColor()
  {
    QColor c = bg_color_property_->getColor();
    bg_color_.setRed(c.red());
    bg_color_.setGreen(c.green());
    bg_color_.setBlue(c.blue());
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateBGAlpha()
  {
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateFGColor()
  {
    QColor c = fg_color_property_->getColor();
    fg_color_.setRed(c.red());
    fg_color_.setGreen(c.green());
    fg_color_.setBlue(c.blue());
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateFGAlpha()
  {
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateFont()
  {
    int font_index = font_property_->getOptionInt();
    if (font_index < font_families_.size())
    {
      font_ = font_families_[font_index].toStdString();
    }
    else
    {
      RCLCPP_FATAL(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                   "Unexpected error at selecting font index %d.", font_index);
      return;
    }
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  void StringDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getInt();
    if (overtake_color_properties_)
    {
      require_update_texture_ = true;
    }
  }

  bool StringDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + texture_height_ > y &&
            left_ < x && left_ + texture_width_ > x);
  }

  void StringDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
  }

  void StringDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
  }

}  // namespace jsk_rviz_plugins

PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::StringDisplay, rviz_common::Display )
