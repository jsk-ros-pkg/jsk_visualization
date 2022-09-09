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
#ifndef CANCEL_ACTION_H
#define CANCEL_ACTION_H

#ifndef Q_MOC_RUN
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
// #include <rviz/panel.h>
#include <QtWidgets>
#include <actionlib_msgs/msg/goal_id.hpp>
#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;

namespace jsk_rviz_plugins
{
class CancelAction : public rviz_common::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  CancelAction(QWidget * parent = 0);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:

  void setTopic(const QString & topic){};

protected Q_SLOTS:

  void updateTopic(){};

  void sendTopic();
  void addTopic();
  void initComboBox();

  void addTopicList(std::string topic_name);

  void OnClickDeleteButton(int id);

protected:
  QString output_topic_;

  QPushButton * add_topic_button_;

  QComboBox * add_topic_box_;

  QPushButton * send_topic_button_;

  QSignalMapper * m_sigmap;

  QVBoxLayout * layout;

  struct topicListLayout
  {
    int id;
    QHBoxLayout * layout_;
    QPushButton * remove_button_;
    QLabel * topic_name_;
    //ros::Publisher publisher_;
    rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr publisher_;
  };

  std::vector<topicListLayout> topic_list_layouts_;

  // // The ROS publisher for the command velocity.
  // ros::Publisher velocity_publisher_;

  // // The ROS node handle.
  // ros::NodeHandle nh_;
  rclcpp::Node::SharedPtr nh_;
  // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
};

}  // namespace jsk_rviz_plugins

#endif  // TELEOP_PANEL_H
