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
#ifndef JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <string>

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <std_msgs/String.h>
#include "overlay_utils.h"
#endif

namespace jsk_rviz_plugins
{
class StringDisplay : public rviz::Display
  {
    Q_OBJECT
  public:
    StringDisplay();
    virtual ~StringDisplay();
    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; }
    virtual int getY() { return top_; }
  protected:
    OverlayObject::Ptr overlay_;

    int texture_width_;
    int texture_height_;

    bool overtake_color_properties_;
    bool overtake_position_properties_;
    bool align_bottom_;
    QColor bg_color_;
    QColor fg_color_;
    int text_size_;
    int line_width_;
    std::string text_;
    QStringList font_families_;
    std::string font_;
    int left_;
    int top_;

    ros::Subscriber sub_;

    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);

    bool require_update_texture_;
    rviz::RosTopicProperty* update_topic_property_;
    rviz::BoolProperty* overtake_position_properties_property_;
    rviz::BoolProperty* overtake_color_properties_property_;
    rviz::BoolProperty* align_bottom_property_;
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
    void updateOvertakeColorProperties();
    void updateAlignBottom();
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
    void processMessage(const std_msgs::String::ConstPtr& msg);
  };
}  // namespace jsk_rviz_plugins

#endif  // JSK_RVIZ_PLUGIN_STRING_DISPLAY_H_
