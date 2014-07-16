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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef JSK_RVIZ_PLUGIN_TARGET_VISUALIZER_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_TARGET_VISUALIZER_DISPLAY_H_


#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <geometry_msgs/PoseStamped.h>

namespace jsk_rviz_plugin
{
  class TargetVisualizerDisplay:
    public rviz::MessageFilterDisplay<geometry_msgs::PoseStamped>
  {
    Q_OBJECT
  public:
    TargetVisualizerDisplay();
    virtual ~TargetVisualizerDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    void processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void update(float wall_dt, float ros_dt);
    void createArrows();
    void updateArrowsObjects(Ogre::ColourValue color);
    rviz::StringProperty* target_name_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* radius_property_;
    Ogre::SceneNode* facing_node_;
    Ogre::SceneNode* target_text_node_;
    rviz::BillboardLine* line_;
    rviz::BillboardLine* text_under_line_;
    Ogre::ManualObject* upper_arrow_;
    Ogre::ManualObject* lower_arrow_;
    Ogre::ManualObject* left_arrow_;
    Ogre::ManualObject* right_arrow_;
    Ogre::SceneNode* upper_arrow_node_;
    Ogre::SceneNode* lower_arrow_node_;
    Ogre::SceneNode* left_arrow_node_;
    Ogre::SceneNode* right_arrow_node_;
    Ogre::MaterialPtr upper_material_;
    Ogre::MaterialPtr lower_material_;
    Ogre::MaterialPtr left_material_;
    Ogre::MaterialPtr right_material_;
    std::string upper_material_name_;
    std::string left_material_name_;
    std::string lower_material_name_;
    std::string right_material_name_;
    rviz::MovableText* msg_;
    boost::mutex mutex_;
    std::string target_name_;
    float t_;
    double alpha_;
    QColor color_;
    double radius_;
    bool require_update_line_;
  private Q_SLOTS:
    void updateTargetName();
    void updateAlpha();
    void updateColor();
    void updateRadius();
  private:
    
  };
}

#endif
