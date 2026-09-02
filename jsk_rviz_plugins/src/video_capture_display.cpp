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

#include "video_capture_display.h"

#include <filesystem>
#include <string>

#include <unistd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QGuiApplication>
#include <QImage>
#include <QScreen>

namespace jsk_rviz_plugins
{
  static rclcpp::Logger logger()
  {
    return rclcpp::get_logger("VideoCaptureDisplay");
  }

  VideoCaptureDisplay::VideoCaptureDisplay():
    Display(), capturing_(false), fps_(30.0), use_3d_viewer_size_(true),
    width_(1920), height_(1080), frame_counter_(0), first_time_(true)
  {
    start_capture_property_ = new rviz_common::properties::BoolProperty(
      "start capture", false, "start capture",
      this, SLOT(updateStartCapture()));
    file_name_property_ = new rviz_common::properties::StringProperty(
      "filename", "output.avi",
      "filename", this, SLOT(updateFileName()));
    fps_property_ = new rviz_common::properties::FloatProperty(
      "fps", 30.0,
      "fps", this, SLOT(updateFps()));
    fps_property_->setMin(0.1);
    use_3d_viewer_size_property_ = new rviz_common::properties::BoolProperty(
      "use 3D viewer size", true,
      "Use width and height of 3D viewer for output video or set them manually",
      this, SLOT(updateUse3DViewerSize()));
    width_property_ = new rviz_common::properties::IntProperty(
      "width", 1920,
      "Width of video in pixels", this, SLOT(updateWidth()));
    height_property_ = new rviz_common::properties::IntProperty(
      "height", 1080,
      "Height of video in pixels", this, SLOT(updateHeight()));
  }

  VideoCaptureDisplay::~VideoCaptureDisplay()
  {
    delete start_capture_property_;
    delete file_name_property_;
    delete fps_property_;
    delete use_3d_viewer_size_property_;
    delete width_property_;
    delete height_property_;
  }

  void VideoCaptureDisplay::onInitialize()
  {
    updateFileName();
    updateFps();
    updateUse3DViewerSize();
    updateWidth();
    updateHeight();
    start_capture_property_->setBool(false); // always false when starting up
    context_->queueRender();
  }

  void VideoCaptureDisplay::onEnable()
  {
    start_capture_property_->setBool(false); // always false when starting up
    context_->queueRender();
  }

  void VideoCaptureDisplay::updateFileName()
  {
    if (capturing_) {
      RCLCPP_WARN(logger(), "cannot change name wile recording");
      file_name_property_->setStdString(file_name_);
    }
    else {
      file_name_ = file_name_property_->getStdString();
      int exists_check = access(file_name_.c_str(), F_OK);
      if (exists_check == 0) {
        int access_result = access(file_name_.c_str(), W_OK);
        RCLCPP_INFO(logger(), "access_result to %s: %d", file_name_.c_str(), access_result);
        if (access_result != 0) {
          setStatus(rviz_common::properties::StatusProperty::Error, "File", "NOT Writable");
        }
        else {
          setStatus(rviz_common::properties::StatusProperty::Ok, "File", "Writable");
        }
      }
      else {                    // do not exists, check directory permission
        RCLCPP_INFO(logger(), "%s do not exists", file_name_.c_str());
        std::filesystem::path pathname(file_name_);
        std::string dirname  = pathname.parent_path().string();
        if (dirname.length() == 0) { // Special case for without path
          dirname = ".";
        }
        RCLCPP_INFO(logger(), "dirname: %s", dirname.c_str());
        int directory_access_result = access(dirname.c_str(), W_OK);
        if (directory_access_result != 0) {
          setStatus(rviz_common::properties::StatusProperty::Error, "File",
                    "NOT Writable (direcotry)");
        }
        else {
          setStatus(rviz_common::properties::StatusProperty::Ok, "File", "Writable");
        }
      }
    }
  }

  void VideoCaptureDisplay::updateStartCapture()
  {
    RCLCPP_INFO(logger(), "updateStartCapture");
    if (first_time_) {
      RCLCPP_WARN(logger(), "ignore first time capture enabling");
    }
    else {
      // start capture!
      if (start_capture_property_->getBool()) {
        capturing_ = true;
        startCapture();
      }
      else {
        capturing_ = false;
        stopCapture();
      }
    }
  }

  void VideoCaptureDisplay::updateFps()
  {
    fps_ = fps_property_->getFloat();
  }

  void VideoCaptureDisplay::updateUse3DViewerSize()
  {
    if (use_3d_viewer_size_ && !use_3d_viewer_size_property_->getBool()) {
      updateWidth();
      updateHeight();
    }

    use_3d_viewer_size_ = use_3d_viewer_size_property_->getBool();
    if (use_3d_viewer_size_) {
      width_property_->hide();
      height_property_->hide();
    }
    else {
      width_property_->show();
      height_property_->show();
    }
  }

  void VideoCaptureDisplay::updateWidth()
  {
    width_ = width_property_->getInt();
  }

  void VideoCaptureDisplay::updateHeight()
  {
    height_ = height_property_->getInt();
  }

  void VideoCaptureDisplay::startCapture()
  {
    RCLCPP_INFO(logger(), "start capturing");
    frame_counter_ = 0;
    if (use_3d_viewer_size_) {
      rviz_common::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
      width_ = panel->width();
      height_ = panel->height();
    }
    writer_.open(file_name_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_,
                 cv::Size(width_, height_));
  }

  void VideoCaptureDisplay::stopCapture()
  {
    RCLCPP_INFO(logger(), "stop capturing");
    writer_.release();
    frame_counter_ = 0;
  }

  void VideoCaptureDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
  {
    if (first_time_) {
      RCLCPP_WARN(logger(), "force to disable capturing");
      start_capture_property_->setBool(false); // always false when starting up
      first_time_ = false;
      return;
    }
    if (capturing_) {
      QPixmap screenshot = QGuiApplication::primaryScreen()->grabWindow(
        context_->getViewManager()->getRenderPanel()->getRenderWindow()->winId());
      QImage src = screenshot.toImage().convertToFormat(QImage::Format_RGB888);  // RGB
      cv::Mat image(src.height(), src.width(), CV_8UC3,
                    (uchar*)src.bits(), src.bytesPerLine());  // RGB
      if (image.size().width != width_ || image.size().height != height_) {
        cv::resize(image, image, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
      }
      cv::cvtColor(image, image, cv::COLOR_RGB2BGR);  // RGB -> BGR
      writer_ << image;
      ++frame_counter_;
      if (frame_counter_ % 100 == 0) {
        RCLCPP_INFO(logger(), "taking %d frames as video", frame_counter_);
      }
    }
    // convert QPixmap into cv::Mat
  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::VideoCaptureDisplay, rviz_common::Display)
