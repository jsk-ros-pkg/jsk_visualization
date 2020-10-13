// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Iori Yanokura
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

#include "rviz_scene_publisher.h"
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <QImage>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QScreen>
#include <QGuiApplication>
#endif
#include <boost/filesystem.hpp>

namespace jsk_rviz_plugins
{
  RvizScenePublisher::RvizScenePublisher():
    Display(), it_(nh_), image_id_(0)
  {
    topic_name_property_ = new rviz::StringProperty(
      "topic_name", "/rviz/image",
      "topic_name", this, SLOT(updateTopicName()));
  }

  RvizScenePublisher::~RvizScenePublisher()
  {
    delete topic_name_property_;
  }

  void RvizScenePublisher::onInitialize()
  {
    updateTopicName();
    context_->queueRender();
  }

  void RvizScenePublisher::onEnable()
  {
    context_->queueRender();
  }

  void RvizScenePublisher::updateTopicName()
  {
    topic_name_ = topic_name_property_->getStdString();
    publisher_ = it_.advertise(topic_name_, 1);
  }

  void RvizScenePublisher::update(float wall_dt, float ros_dt)
  {
    rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    QPixmap screenshot
      = QGuiApplication::primaryScreen()->grabWindow(context_->getViewManager()->getRenderPanel()->winId());
#else
    QPixmap screenshot
      = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
#endif
    QImage src = screenshot.toImage().convertToFormat(QImage::Format_RGB888);  // RGB
    cv::Mat image(src.height(), src.width(), CV_8UC3,
                  (uchar*)src.bits(), src.bytesPerLine());  // RGB

    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = image_id_++;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    img_bridge.toImageMsg(img_msg);
    publisher_.publish(img_msg);
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RvizScenePublisher, rviz::Display)
