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
#ifndef RECORD_ACTION_H
#define RECORD_ACTION_H

#ifndef Q_MOC_RUN
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
// #include <rviz/panel.h>
#include <QtWidgets>
#include <jsk_rviz_plugin_msgs/msg/record_command.hpp>
#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QLabel;
class QPushButton;

namespace jsk_rviz_plugins
{
class RecordAction : public rviz_common::Panel
{
  enum RecordState { IDLE = 0, RECORD = 1 };
  Q_OBJECT
public:
  RecordAction(QWidget * parent = 0);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:

  void setTopic(const QString & topic) {}

protected Q_SLOTS:

  void updateTopic(){};

  void commandPlay(){};
  void recordClick();

  void addTopicList(std::string topic_name);

  void OnClickPlayButton(int id);
  void OnClickDeleteButton(int id);

protected:
  QLineEdit * record_motion_name_editor_;

  QString output_topic_;

  QPushButton * record_interface_button_;

  QComboBox * add_topic_box_;

  QSignalMapper * m_delete_sigmap_;
  QSignalMapper * m_play_sigmap_;

  QVBoxLayout * layout;

  struct motionListLayout
  {
    int id;
    QHBoxLayout * layout_;
    QPushButton * play_button_;
    QPushButton * remove_button_;
    QLabel * target_name_;
  };

  std::vector<motionListLayout> motion_list_layouts_;

  // ros::Publisher pub_;
  // ros::NodeHandle nh_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::RecordCommand>::SharedPtr pub_;

  RecordState rstate_;
};

}  // namespace jsk_rviz_plugins

#endif
