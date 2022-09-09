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
#ifndef OBJECT_FIT_OPERATOR_H
#define OBJECT_FIT_OPERATOR_H

#ifndef Q_MOC_RUN
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
// #include <rviz/panel.h>
#include <QCheckBox>
#include <QToolButton>
#include <QtGlobal>
#include <QtWidgets>
#include <jsk_rviz_plugin_msgs/msg/object_fit_command.hpp>
#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QToolButton;

namespace jsk_rviz_plugins
{
class ObjectFitOperatorAction : public rviz_common::Panel
{
  Q_OBJECT
public:
  ObjectFitOperatorAction(QWidget * parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:
  void commandFit();
  void commandNear();
  void commandOther();
  void checkBoxChanged(bool state);
  void publishObjectFitOder(int type);

protected:
  QToolButton * fit_button_;
  QToolButton * near_button_;
  QToolButton * other_button_;
  QCheckBox * check_box_;

  QHBoxLayout * horizontal_layout1_;
  QHBoxLayout * horizontal_layout2_;
  QVBoxLayout * layout;
  bool reverse_;

  // ros::NodeHandle nh_;
  // ros::Publisher pub_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::ObjectFitCommand>::SharedPtr pub_;
};
}  // namespace jsk_rviz_plugins

#endif
