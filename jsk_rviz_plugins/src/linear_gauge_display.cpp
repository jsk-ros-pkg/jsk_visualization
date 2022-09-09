// -*- mode: c++; -*-
// Copyright (c) 2014, JSK Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the JSK Lab nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "linear_gauge_display.hpp"

#include <QPainter>
#include <iomanip>
#include <rviz_common/display_context.hpp>
#include <rviz_common/uniform_string_stream.hpp>

namespace jsk_rviz_plugins
{
LinearGaugeDisplay::LinearGaugeDisplay()
: data_(0.0), first_time_(true), width_padding_(5), height_padding_(5)
{
  show_value_property_ = new rviz_common::properties::BoolProperty(
    "Show Value", true, "Show value on plotter", this, SLOT(updateShowValue()));

  vertical_gauge_property_ = new rviz_common::properties::BoolProperty(
    "Vertical Gauge", false, "set gauge vertical", this, SLOT(updateVerticalGauge()));

  width_property_ = new rviz_common::properties::IntProperty(
    "width", 500, "width of the plotter window", this, SLOT(updateWidth()));
  width_property_->setMin(1);
  width_property_->setMax(2000);
  height_property_ = new rviz_common::properties::IntProperty(
    "height", 50, "height of the plotter window", this, SLOT(updateHeight()));
  height_property_->setMin(1);
  height_property_->setMax(2000);
  left_property_ = new rviz_common::properties::IntProperty(
    "left", 128, "left of the plotter window", this, SLOT(updateLeft()));
  left_property_->setMin(0);
  top_property_ = new rviz_common::properties::IntProperty(
    "top", 128, "top of the plotter window", this, SLOT(updateTop()));
  top_property_->setMin(0);

  max_value_property_ = new rviz_common::properties::FloatProperty(
    "max value", 100.0, "max value, used only if auto scale is disabled", this,
    SLOT(updateMaxValue()));
  min_value_property_ = new rviz_common::properties::FloatProperty(
    "min value", 0.0, "min value, used only if auto scale is disabled", this,
    SLOT(updateMinValue()));
  fg_color_property_ = new rviz_common::properties::ColorProperty(
    "foreground color", QColor(25, 255, 240), "color to draw line", this, SLOT(updateFGColor()));
  fg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "foreground alpha", 0.7, "alpha belnding value for foreground", this, SLOT(updateFGAlpha()));
  fg_alpha_property_->setMin(0);
  fg_alpha_property_->setMax(1.0);
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "background color", QColor(0, 0, 0), "background color", this, SLOT(updateBGColor()));
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "backround alpha", 0.0, "alpha belnding value for background", this, SLOT(updateBGAlpha()));
  bg_alpha_property_->setMin(0);
  bg_alpha_property_->setMax(1.0);
  line_width_property_ = new rviz_common::properties::IntProperty(
    "linewidth", 1, "linewidth of the plot", this, SLOT(updateLineWidth()));
  line_width_property_->setMin(1);
  line_width_property_->setMax(1000);
  show_border_property_ = new rviz_common::properties::BoolProperty(
    "border", true, "show border or not", this, SLOT(updateShowBorder()));
  text_size_property_ = new rviz_common::properties::IntProperty(
    "text size", 12, "text size of the caption", this, SLOT(updateTextSize()));
  text_size_property_->setMin(1);
  text_size_property_->setMax(1000);
  show_caption_property_ = new rviz_common::properties::BoolProperty(
    "show caption", true, "show caption or not", this, SLOT(updateShowCaption()));
  update_interval_property_ = new rviz_common::properties::FloatProperty(
    "update interval", 0.04, "update interval of the plotter", this, SLOT(updateUpdateInterval()));
  update_interval_property_->setMin(0.0);
  update_interval_property_->setMax(100);
  auto_color_change_property_ = new rviz_common::properties::BoolProperty(
    "auto color change", false, "change the color automatically", this,
    SLOT(updateAutoColorChange()));
  max_color_property_ = new rviz_common::properties::ColorProperty(
    "max color", QColor(255, 0, 0), "only used if auto color change is set to True.", this,
    SLOT(updateMaxColor()));
}

LinearGaugeDisplay::~LinearGaugeDisplay()
{
  onDisable();
  //
  // delete buffer_length_property_;
  // delete fg_color_property_;
  // delete bg_color_property_;
  // delete fg_alpha_property_;
  // delete bg_alpha_property_;
  // delete top_property_;
  // delete left_property_;
  // delete width_property_;
  // delete height_property_;
  // delete line_width_property_;
  // delete show_border_property_;
  // delete auto_color_change_property_;
  // delete max_color_property_;
  // delete update_interval_property_;
  // delete show_caption_property_;
  // delete text_size_property_;
  // delete min_value_property_;
  // delete max_value_property_;
  // delete auto_color_change_property_;
}

void LinearGaugeDisplay::onInitialize()
{
  overlay_->prepareOverlays(scene_manager_);
  RTDClass::onInitialize();
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "LinearGaugeDisplayObject" << count++;
  overlay_.reset(new OverlayObject(ss.str()));
  onEnable();
  updateShowValue();
  updateVerticalGauge();
  updateWidth();
  updateHeight();
  updateLeft();
  updateTop();
  updateFGColor();
  updateBGColor();
  updateFGAlpha();
  updateBGAlpha();
  updateLineWidth();
  updateUpdateInterval();
  updateShowBorder();
  updateAutoColorChange();
  updateMaxColor();
  updateShowCaption();
  updateTextSize();
  updateMinValue();
  updateMaxValue();
  overlay_->updateTextureSize(
    width_property_->getInt(), height_property_->getInt() + caption_offset_);
}

void LinearGaugeDisplay::drawPlot()
{
  QColor fg_color(fg_color_);
  QColor bg_color(bg_color_);
  double max_gauge_length = 0.0;

  fg_color.setAlpha(fg_alpha_);
  bg_color.setAlpha(bg_alpha_);

  if (auto_color_change_) {
    double r = std::min(std::max(data_ / (max_value_ - min_value_), 0.0), 1.0);
    if (r > 0.3) {
      double r2 = (r - 0.3) / 0.7;
      fg_color.setRed((max_color_.red() - fg_color_.red()) * r2 + fg_color_.red());
      fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2 + fg_color_.green());
      fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2 + fg_color_.blue());
    }
  }

  {
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_);
    // initilize by the background color
    for (int i = 0; i < overlay_->getTextureWidth(); i++) {
      for (int j = 0; j < overlay_->getTextureHeight(); j++) {
        Hud.setPixel(i, j, bg_color.rgba());
      }
    }
    // paste in HUD speedometer. I resize the image and offset it by 8 pixels from
    // the bottom left edge of the render window
    QPainter painter(&Hud);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color, line_width_, Qt::SolidLine));

    uint16_t w = overlay_->getTextureWidth();
    uint16_t h = overlay_->getTextureHeight() - caption_offset_;

    //draw gauge
    if (vertical_gauge_) {
      double normalised_value =
        std::min(std::max((double)data_, 0.0), max_value_) * (h - 2 * height_padding_) / max_value_;
      painter.fillRect(
        width_padding_, h - normalised_value - height_padding_, w - 2 * width_padding_,
        normalised_value, fg_color);
    } else {
      double normalised_value =
        std::min(std::max((double)data_, 0.0), max_value_) * (w - 2 * width_padding_) / max_value_;
      painter.fillRect(
        width_padding_, height_padding_, normalised_value, h - (2 * height_padding_), fg_color);
    }

    // draw border
    if (show_border_) {
      painter.drawLine(0, 0, 0, h);
      painter.drawLine(0, h, w, h);
      painter.drawLine(w, h, w, 0);
      painter.drawLine(w, 0, 0, 0);
    }
    // draw caption
    if (show_caption_) {
      QFont font = painter.font();
      font.setPointSize(text_size_);
      font.setBold(true);
      painter.setFont(font);
      painter.drawText(0, h, w, caption_offset_, Qt::AlignCenter | Qt::AlignVCenter, getName());
    }

    //draw value
    if (show_value_) {
      QFont font = painter.font();
      font.setPointSize(std::min(w - 2 * width_padding_, h - 2 * height_padding_));
      font.setBold(true);
      painter.setFont(font);
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << data_;

      if (w < h)  //rotate text to fit gauge if needed
      {
        painter.translate(0, h);
        painter.rotate(-90);
        painter.drawText(0, 0, h, w, Qt::AlignCenter | Qt::AlignVCenter, ss.str().c_str());
        painter.rotate(90);
        painter.translate(0, -h);
      } else {
        painter.drawText(0, 0, w, h, Qt::AlignCenter | Qt::AlignVCenter, ss.str().c_str());
      }
    }

    // done
    painter.end();
  }
}

void LinearGaugeDisplay::processMessage(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!isEnabled() || !overlay_->isVisible()) {
    return;
  }
  if (data_ != msg->data || first_time_) {
    first_time_ = false;
    data_ = msg->data;
    draw_required_ = true;
  }
}

void LinearGaugeDisplay::update(float wall_dt, float ros_dt)
{
  if (draw_required_) {
    if (wall_dt + last_time_ > update_interval_) {
      overlay_->updateTextureSize(texture_width_, texture_height_ + caption_offset_);
      overlay_->setPosition(left_, top_);
      overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
      last_time_ = 0;
      drawPlot();
      draw_required_ = false;
    } else {
      last_time_ = last_time_ + wall_dt;
    }
  }
}

void LinearGaugeDisplay::reset() { RTDClass::reset(); }

void LinearGaugeDisplay::onEnable()
{
  RTDClass::onEnable();
  last_time_ = 0;
  draw_required_ = false;
  subscribe();
  overlay_->show();
}

void LinearGaugeDisplay::onDisable()
{
  unsubscribe();
  overlay_->hide();
}

void LinearGaugeDisplay::updateWidth()
{
  std::lock_guard<std::mutex> lock(mutex_);
  texture_width_ = width_property_->getInt();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateHeight()
{
  std::lock_guard<std::mutex> lock(mutex_);
  texture_height_ = height_property_->getInt();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateTop()
{
  top_ = top_property_->getInt();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateLeft()
{
  left_ = left_property_->getInt();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateBGColor()
{
  bg_color_ = bg_color_property_->getColor();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateFGColor()
{
  fg_color_ = fg_color_property_->getColor();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateFGAlpha()
{
  fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  draw_required_ = true;
}

void LinearGaugeDisplay::updateBGAlpha()
{
  bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  draw_required_ = true;
}

void LinearGaugeDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void LinearGaugeDisplay::updateShowValue()
{
  show_value_ = show_value_property_->getBool();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateVerticalGauge()
{
  vertical_gauge_ = vertical_gauge_property_->getBool();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateShowBorder()
{
  show_border_ = show_border_property_->getBool();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateLineWidth()
{
  line_width_ = line_width_property_->getInt();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateAutoColorChange()
{
  auto_color_change_ = auto_color_change_property_->getBool();
  if (auto_color_change_) {
    max_color_property_->show();
  } else {
    max_color_property_->hide();
  }
  draw_required_ = true;
}

void LinearGaugeDisplay::updateMaxColor()
{
  max_color_ = max_color_property_->getColor();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateUpdateInterval()
{
  update_interval_ = update_interval_property_->getFloat();
}

void LinearGaugeDisplay::updateTextSize()
{
  text_size_ = text_size_property_->getInt();
  QFont font;
  font.setPointSize(text_size_);
  caption_offset_ = QFontMetrics(font).height();
  draw_required_ = true;
}

void LinearGaugeDisplay::updateShowCaption()
{
  show_caption_ = show_caption_property_->getBool();
  if (show_caption_) {
    text_size_property_->show();
  } else {
    text_size_property_->hide();
  }
  draw_required_ = true;
}

void LinearGaugeDisplay::updateMinValue() { min_value_ = min_value_property_->getFloat(); }

void LinearGaugeDisplay::updateMaxValue() { max_value_ = max_value_property_->getFloat(); }

bool LinearGaugeDisplay::isInRegion(int x, int y)
{
  return (top_ < y && top_ + texture_height_ > y && left_ < x && left_ + texture_width_ > x);
}

void LinearGaugeDisplay::movePosition(int x, int y)
{
  top_ = y;
  left_ = x;
}

void LinearGaugeDisplay::setPosition(int x, int y)
{
  top_property_->setValue(y);
  left_property_->setValue(x);
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::LinearGaugeDisplay, rviz_common::Display)
