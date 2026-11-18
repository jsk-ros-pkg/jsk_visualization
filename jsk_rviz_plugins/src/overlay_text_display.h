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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_TEXT_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_TEXT_DISPLAY_H_

#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <jsk_rviz_plugins/msg/overlay_text.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(jsk_rviz_plugins, OverlayText)
#else
#include "jsk_rviz_plugins/OverlayText.h"
#endif
#ifndef Q_MOC_RUN
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/display.hpp>
#else
#include <rviz/display.h>
#endif
#include "overlay_utils.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#if ROS_VERSION_MAJOR != 1
#include <std_msgs/msg/color_rgba.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#else
#include <std_msgs/ColorRGBA.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#endif
#endif

namespace jsk_rviz_plugins
{
  class OverlayTextDisplay
  : public rviz::Display
  {
    Q_OBJECT
  public:
    OverlayTextDisplay();
    virtual ~OverlayTextDisplay();
    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };
  protected:
    jsk_rviz_plugins::OverlayObject::Ptr overlay_;

    int texture_width_;
    int texture_height_;
    
    bool overtake_fg_color_properties_;
    bool overtake_bg_color_properties_;
    bool overtake_position_properties_;
    bool align_bottom_;
    bool invert_shadow_;
    QColor bg_color_;
    QColor fg_color_;
    int text_size_;
    int line_width_;
    std::string text_;
    QStringList font_families_;
    std::string font_;
    int left_;
    int top_;
    
#if ROS_VERSION_MAJOR != 1
    rclcpp::Subscription<jsk_rviz_plugins::OverlayText>::SharedPtr sub_;
#else
    ros::Subscriber sub_;
#endif
    
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);

    bool require_update_texture_;
    rviz::RosTopicProperty* update_topic_property_;
    rviz::BoolProperty* overtake_position_properties_property_;
    rviz::BoolProperty* overtake_fg_color_properties_property_;
    rviz::BoolProperty* overtake_bg_color_properties_property_;
    rviz::BoolProperty* align_bottom_property_;
    rviz::BoolProperty* invert_shadow_property_;
    rviz::IntProperty* top_property_;
    rviz::IntProperty* left_property_;
    rviz::IntProperty* width_property_;
    rviz::IntProperty* height_property_;
    rviz::IntProperty* text_size_property_;
    rviz::IntProperty* line_width_property_;
    rviz::ColorProperty* bg_color_property_;
    rviz::FloatProperty* bg_alpha_property_;
    rviz::ColorProperty* fg_color_property_;
    rviz::FloatProperty* fg_alpha_property_;
    rviz::EnumProperty* font_property_;
  protected Q_SLOTS:
    void updateTopic();
    void updateOvertakePositionProperties();
    void updateOvertakeFGColorProperties();
    void updateOvertakeBGColorProperties();
    void updateAlignBottom();
    void updateInvertShadow();
    void updateTop();
    void updateLeft();
    void updateWidth();
    void updateHeight();
    void updateTextSize();
    void updateFGColor();
    void updateFGAlpha();
    void updateBGColor();
    void updateBGAlpha();
    void updateFont();
    void updateLineWidth();
  private:
#if ROS_VERSION_MAJOR != 1
    void processMessage(const jsk_rviz_plugins::OverlayText::ConstSharedPtr& msg);
#else
    void processMessage(const jsk_rviz_plugins::OverlayText::ConstPtr& msg);
#endif
  };
}

#endif
