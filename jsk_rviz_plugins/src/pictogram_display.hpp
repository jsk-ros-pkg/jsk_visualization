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
#ifndef JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
//#include <rviz_common/message_filter_display.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <chrono>
#include <rclcpp/duration.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <std_msgs/msg/float32.hpp>

#include "facing_visualizer.hpp"
#include "jsk_rviz_plugin_msgs/msg/pictogram.hpp"
#endif
#include <mutex>

namespace jsk_rviz_plugins
{
void setupFont();
int addFont(unsigned char * data, unsigned int data_len);
bool isFontAwesome(std::string);
bool isEntypo(std::string);
bool isCharacterSupported(std::string character);
QFont getFont(std::string character);
QString lookupPictogramText(std::string character);
////////////////////////////////////////////////////////
// PictogramObject
////////////////////////////////////////////////////////
class PictogramObject : public FacingTexturedObject
{
public:
  typedef std::shared_ptr<PictogramObject> Ptr;
  PictogramObject(Ogre::SceneManager * manager, Ogre::SceneNode * parent, double size);
  void update(float wall_dt, float ros_dt) override;
  //void reset() override;
  void setEnable(bool enable);
  void setText(std::string text);
  void setAlpha(double alpha);
  void setColor(QColor color);
  void setSize(double size);
  void setSpeed(double speed);
  void setPose(const geometry_msgs::msg::Pose & pose, const std::string & frame_id);
  void start();
  void setContext(rviz_common::DisplayContext * context);
  void setAction(uint8_t action);
  void setMode(uint8_t mode);
  void setTTL(double ttl);

protected:
  void updatePose(float dt);
  void updateColor();
  void updateText();

  bool need_to_update_;
  uint8_t action_;
  geometry_msgs::msg::Pose pose_;
  std::string frame_id_;
  rviz_common::DisplayContext * context_;
  //ros::WallTime time_;
  rclcpp::Time time_;
  rclcpp::Clock clock_;
  double ttl_;
  double speed_;
  uint8_t mode_;

private:
};

////////////////////////////////////////////////////////
// Display to visualize pictogram on rviz
////////////////////////////////////////////////////////
class PictogramDisplay : public rviz_common::RosTopicDisplay<jsk_rviz_plugin_msgs::msg::Pictogram>
{
  Q_OBJECT
public:
  PictogramDisplay();
  ~PictogramDisplay();

protected:
  ////////////////////////////////////////////////////////
  // methods
  ////////////////////////////////////////////////////////
  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void processMessage(jsk_rviz_plugin_msgs::msg::Pictogram::ConstSharedPtr msg);
  void update(float wall_dt, float ros_dt) override;
  //void reset() override;

  ////////////////////////////////////////////////////////
  // parameters
  ////////////////////////////////////////////////////////
  std::mutex mutex_;
  PictogramObject::Ptr pictogram_;
private Q_SLOTS:

private:
};
}  // namespace jsk_rviz_plugins

#endif
