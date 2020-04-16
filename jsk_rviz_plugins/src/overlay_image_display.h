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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_IMAGE_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_IMAGE_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz/display.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>

#include <QPainter>

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/editable_enum_property.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "overlay_utils.h"
#include "image_transport_hints_property.h"
#endif

namespace jsk_rviz_plugins
{
  class OverlayImageDisplay : public rviz::Display
  {
    Q_OBJECT
  public:
    OverlayImageDisplay();
    virtual ~OverlayImageDisplay();

    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };

  protected:
    boost::mutex mutex_;
    OverlayObject::Ptr overlay_;
    rviz::RosTopicProperty* update_topic_property_;
    ImageTransportHintsProperty* transport_hint_property_;
    rviz::BoolProperty* keep_aspect_ratio_property_;
    rviz::IntProperty* width_property_;
    rviz::IntProperty* height_property_;
    rviz::IntProperty* left_property_;
    rviz::IntProperty* top_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* overwrite_alpha_property_;
    int width_, height_, left_, top_;
    double alpha_;
#if ROS_VERSION_MINIMUM(1,12,0)
    std::shared_ptr<image_transport::ImageTransport> it_;
#else
    boost::shared_ptr<image_transport::ImageTransport> it_;
#endif
    image_transport::Subscriber sub_;
    sensor_msgs::Image::ConstPtr msg_;
    bool is_msg_available_;
    bool require_update_;
    bool keep_aspect_ratio_;
    bool overwrite_alpha_;

    virtual void redraw();
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void setImageSize();
    virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);
  protected Q_SLOTS:
    void updateTopic();
    void updateWidth();
    void updateHeight();
    void updateLeft();
    void updateTop();
    void updateAlpha();
    void updateKeepAspectRatio();
    void updateOverwriteAlpha();
  };

}

#endif
