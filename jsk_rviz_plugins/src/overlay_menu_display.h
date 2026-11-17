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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_

#ifndef Q_MOC_RUN
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <boost/thread/mutex.hpp>
#include <rviz_common/display.hpp>
#else
#include <rviz/display.h>
#endif
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>

#include <QPainter>

#if ROS_VERSION_MAJOR != 1
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <jsk_rviz_plugins/msg/overlay_menu.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(jsk_rviz_plugins, OverlayMenu)
#else
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <std_msgs/ColorRGBA.h>
#include <jsk_rviz_plugins/OverlayMenu.h>
#endif

#include "overlay_utils.h"
#endif

namespace jsk_rviz_plugins
{
  class OverlayMenuDisplay : public rviz::Display
  {
    Q_OBJECT
  public:
    OverlayMenuDisplay();
    virtual ~OverlayMenuDisplay();

    enum AnimationState
    {
      CLOSED,
      OPENED,
      OPENING,
      CLOSING,
    };

    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };

  protected:
    boost::mutex mutex_;
    OverlayObject::Ptr overlay_;
#if ROS_VERSION_MAJOR != 1
    rclcpp::Subscription<jsk_rviz_plugins::OverlayMenu>::SharedPtr sub_;
#else
    ros::Subscriber sub_;
#endif
    rviz::RosTopicProperty* update_topic_property_;
    rviz::IntProperty* left_property_;
    rviz::IntProperty* top_property_;
    rviz::BoolProperty* keep_centered_property_;
    rviz::BoolProperty* overtake_fg_color_properties_property_;
    rviz::BoolProperty* overtake_bg_color_properties_property_;
    rviz::ColorProperty* bg_color_property_;
    rviz::FloatProperty* bg_alpha_property_;
    rviz::ColorProperty* fg_color_property_;
    rviz::FloatProperty* fg_alpha_property_;
    AnimationState animation_state_;
    bool require_update_texture_;
    bool keep_centered_;
    int left_, top_;
#if ROS_VERSION_MAJOR != 1
    jsk_rviz_plugins::OverlayMenu::ConstSharedPtr current_menu_;
    jsk_rviz_plugins::OverlayMenu::ConstSharedPtr next_menu_;
#else
    jsk_rviz_plugins::OverlayMenu::ConstPtr current_menu_;
    jsk_rviz_plugins::OverlayMenu::ConstPtr next_menu_;
#endif
    double animation_t_;
    bool overtake_fg_color_properties_;
    bool overtake_bg_color_properties_;
    QColor bg_color_;
    QColor fg_color_;

    virtual void prepareOverlay();
    virtual void openingAnimation();
    virtual std::string getMenuString(
#if ROS_VERSION_MAJOR != 1
      const jsk_rviz_plugins::OverlayMenu::ConstSharedPtr& msg,
#else
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg,
#endif
      size_t index);
    virtual QFont font();
    virtual QFontMetrics fontMetrics();
    virtual int drawAreaWidth(
#if ROS_VERSION_MAJOR != 1
      const jsk_rviz_plugins::OverlayMenu::ConstSharedPtr& msg);
#else
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
#endif
    virtual int drawAreaHeight(
#if ROS_VERSION_MAJOR != 1
      const jsk_rviz_plugins::OverlayMenu::ConstSharedPtr& msg);
#else
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
#endif
    virtual bool isNeedToResize();
    virtual bool isNeedToRedraw();
    virtual void redraw();
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void processMessage
#if ROS_VERSION_MAJOR != 1
    (const jsk_rviz_plugins::OverlayMenu::ConstSharedPtr& msg);
#else
    (const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
#endif
    virtual void setMenuLocation();
  protected Q_SLOTS:
    void updateTopic();
    void updateLeft();
    void updateTop();
    void updateKeepCentered();
    void updateOvertakeFGColorProperties();
    void updateOvertakeBGColorProperties();
    void updateFGColor();
    void updateFGAlpha();
    void updateBGColor();
    void updateBGAlpha();
  };

}

#endif

