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

#include "overlay_image_display.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <cv_bridge/cv_bridge.h>

#include <rviz_common/uniform_string_stream.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "rviz_utils.hpp"

namespace jsk_rviz_plugins
{
OverlayImageDisplay::OverlayImageDisplay()
: width_(128),
  height_(128),
  left_(128),
  top_(128),
  alpha_(0.8),
  is_msg_available_(false),
  require_update_(false),
  overwrite_alpha_(false)
{
  // setup properties
  // transport_hint_property_ = new ImageTransportHintsProperty("transport hint",
  //                                                           "transport hint to subscribe topic",
  //                                                           this, SLOT(updateTopic()));
  keep_aspect_ratio_property_ = new rviz_common::properties::BoolProperty(
    "keep aspect ratio", false, "keep aspect ratio of original image", this,
    SLOT(updateKeepAspectRatio()));
  width_property_ = new rviz_common::properties::IntProperty(
    "width", 128, "width of the image window", this, SLOT(updateWidth()));
  height_property_ = new rviz_common::properties::IntProperty(
    "height", 128, "height of the image window", this, SLOT(updateHeight()));
  left_property_ = new rviz_common::properties::IntProperty(
    "left", 128, "left of the image window", this, SLOT(updateLeft()));
  top_property_ = new rviz_common::properties::IntProperty(
    "top", 128, "top of the image window", this, SLOT(updateTop()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "alpha", 0.8, "alpha belnding value", this, SLOT(updateAlpha()));
  overwrite_alpha_property_ = new rviz_common::properties::BoolProperty(
    "overwrite alpha value", false,
    "overwrite alpha value by alpha property "
    "and ignore alpha channel of the image",
    this, SLOT(updateOverwriteAlpha()));
}

OverlayImageDisplay::~OverlayImageDisplay()
{
  // delete transport_hint_property_;
  delete keep_aspect_ratio_property_;
  delete overwrite_alpha_property_;
  delete width_property_;
  delete height_property_;
  delete left_property_;
  delete top_property_;
  delete alpha_property_;
}

void OverlayImageDisplay::onInitialize()
{
  overlay_->prepareOverlays(scene_manager_);
  RTDClass::onInitialize();
  //it_ = std::shared_ptr<image_transport::msg::ImageTransport>(new image_transport::msg::ImageTransport(nh));

  updateWidth();
  updateHeight();
  updateKeepAspectRatio();
  updateOverwriteAlpha();
  updateTop();
  updateLeft();
  updateAlpha();
  updateTopic();
}

void OverlayImageDisplay::onEnable()
{
  RTDClass::onEnable();
  if (overlay_) {
    overlay_->show();
  }
  subscribe();
}
void OverlayImageDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
  unsubscribe();
}

void OverlayImageDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  msg_ = msg;
  is_msg_available_ = true;
  require_update_ = true;
  if ((width_property_->getInt() < 0) || (height_property_->getInt() < 0) || keep_aspect_ratio_) {
    // automatically setup display size
    updateWidth();
    updateHeight();
  }
}

void OverlayImageDisplay::update(float wall_dt, float ros_dt)
{
  if (!isEnabled()) {
    return;
  }

  if (require_update_ && is_msg_available_) {
    if (!overlay_) {
      static int count = 0;
      rviz_common::UniformStringStream ss;
      ss << "OverlayImageDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    overlay_->updateTextureSize(msg_->width, msg_->height);
    // When aspect_ratio being kept, the size is specified by width;
    height_property_->setHidden(keep_aspect_ratio_);
    setImageSize();
    redraw();
    require_update_ = false;
  }
  if (overlay_) {
    overlay_->setDimensions(width_, height_);
    overlay_->setPosition(left_, top_);
  }
}

void OverlayImageDisplay::reset() { RTDClass::reset(); }

void OverlayImageDisplay::redraw()
{
  try {
    if (msg_->width == 0 || msg_->height == 0) {
      // image width/height and texture width/height should be same
      // but they are not when input image width/height is 0
      return;
    } else {
      cv::Mat mat;  // mat should be BGRA8 image

      if (
        (msg_->encoding == sensor_msgs::image_encodings::BGRA8 ||
         msg_->encoding == sensor_msgs::image_encodings::RGBA8) &&
        !overwrite_alpha_) {
        const cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg_, sensor_msgs::image_encodings::BGRA8);
        cv_ptr->image.copyTo(mat);
      } else {
        // If the image does not have alpha channel, use alpha_ value.
        const cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg_, sensor_msgs::image_encodings::BGR8);
        const cv::Mat bgr_image = cv_ptr->image;
        std::vector<cv::Mat> channels;
        // Split BGR image to each channel because cv::merge requires 4 images to create
        // B-G-R-A image. The each 4 image represents each channel.
        cv::split(bgr_image, channels);
        // Create single alpha channel image
        const cv::Mat alpha(bgr_image.rows, bgr_image.cols, CV_8UC1, cv::Scalar(alpha_ * 255.0));
        channels.push_back(alpha);
        cv::merge(channels, mat);
      }

      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_);
      // QImage created from ScopedPixelBuffer has no padding between each line.
      memcpy(Hud.scanLine(0), mat.data, mat.cols * mat.rows * mat.elemSize());
    }
  } catch (cv_bridge::Exception & e) {
    JSK_LOG_ERROR("cv_bridge exception: %s", e.what());
  }
}

void OverlayImageDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void OverlayImageDisplay::setImageSize()
{
  if (width_ == -1) {
    if (is_msg_available_) {
      width_ = msg_->width;
    } else {
      width_ = 128;
    }
  }

  if (height_ == -1) {
    if (is_msg_available_) {
      height_ = msg_->height;
    } else {
      height_ = 128;
    }
  }

  if (keep_aspect_ratio_ && is_msg_available_) {
    // When aspect_ratio being kept, the size is specified by width;
    double aspect_ratio = msg_->height / (double)msg_->width;
    int height_from_width = std::ceil(width_ * aspect_ratio);
    height_ = height_from_width;
  }
}

void OverlayImageDisplay::updateWidth()
{
  std::lock_guard<std::mutex> lock(mutex_);
  width_ = width_property_->getInt();
  require_update_ = true;
}

void OverlayImageDisplay::updateHeight()
{
  std::lock_guard<std::mutex> lock(mutex_);
  height_ = height_property_->getInt();
  require_update_ = true;
}

void OverlayImageDisplay::updateTop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  top_ = top_property_->getInt();
}

void OverlayImageDisplay::updateLeft()
{
  std::lock_guard<std::mutex> lock(mutex_);
  left_ = left_property_->getInt();
}

void OverlayImageDisplay::updateAlpha()
{
  std::lock_guard<std::mutex> lock(mutex_);
  alpha_ = alpha_property_->getFloat();
}

void OverlayImageDisplay::updateKeepAspectRatio()
{
  std::lock_guard<std::mutex> lock(mutex_);
  keep_aspect_ratio_ = keep_aspect_ratio_property_->getBool();
  require_update_ = true;
}

void OverlayImageDisplay::updateOverwriteAlpha()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overwrite_alpha_ = overwrite_alpha_property_->getBool();
  require_update_ = true;
}

bool OverlayImageDisplay::isInRegion(int x, int y)
{
  return (top_ < y && top_ + height_ > y && left_ < x && left_ + width_ > x);
}

void OverlayImageDisplay::movePosition(int x, int y)
{
  top_ = y;
  left_ = x;
}

void OverlayImageDisplay::setPosition(int x, int y)
{
  top_property_->setValue(y);
  left_property_->setValue(x);
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::OverlayImageDisplay, rviz_common::Display)
