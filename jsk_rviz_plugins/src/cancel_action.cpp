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

#include "cancel_action.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <rviz_common/display_context.hpp>

namespace jsk_rviz_plugins
{
CancelAction::CancelAction(QWidget * parent) : rviz_common::Panel(parent) {}

void CancelAction::onInitialize()
{
  layout = new QVBoxLayout;

  //Text Box and Add Button to add new topic
  QHBoxLayout * topic_layout = new QHBoxLayout;

  add_topic_box_ = new QComboBox;
  initComboBox();
  topic_layout->addWidget(add_topic_box_);

  QPushButton * add_topic_button_ = new QPushButton("Add Action");
  topic_layout->addWidget(add_topic_button_);

  layout->addLayout(topic_layout);
  //End of Text Box and Add Button

  m_sigmap = new QSignalMapper(this);

  connect(m_sigmap, SIGNAL(mapped(int)), this, SLOT(OnClickDeleteButton(int)));

  //Button to send cancel topic
  QPushButton * send_topic_button_ = new QPushButton("Cancel Action");
  layout->addWidget(send_topic_button_);

  setLayout(layout);

  connect(send_topic_button_, SIGNAL(clicked()), this, SLOT(sendTopic()));
  connect(add_topic_button_, SIGNAL(clicked()), this, SLOT(addTopic()));

  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void CancelAction::initComboBox()
{
  add_topic_box_->addItem("");

  // 現在アクティブなトピックを取得
  // ros::master::V_TopicInfo topics;
  // ros::master::getTopics(topics);
  // ros::master::V_TopicInfo::iterator it = topics.begin();
  // while (it != topics.end())
  // {
  //   if (it->datatype == "actionlib_msgs/GoalID")
  //   {
  //     std::string action_name = it->name;
  //     std::string delete_string = "/cancel";
  //     std::string::size_type index = action_name.find_last_of(delete_string);
  //     if (index != std::string::npos)
  //     {
  //       action_name.erase(index - delete_string.length() + 1);
  //       add_topic_box_->addItem(action_name.c_str());
  //     }
  //   }
  //   it++;
  // }
}

void CancelAction::OnClickDeleteButton(int id)
{
  std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
  while (it != topic_list_layouts_.end()) {
    if (it->id == id) {
      it->topic_name_->hide();
      delete it->topic_name_;

      it->remove_button_->hide();
      delete it->remove_button_;

      delete it->layout_;
      it->publisher_.reset();
      it = topic_list_layouts_.erase(it);
      Q_EMIT configChanged();
    } else {
      ++it;
    }
  }
}

void CancelAction::addTopic()
{
  output_topic_ = add_topic_box_->currentText();
  if (output_topic_ != "") {
    add_topic_box_->setCurrentIndex(0);
    addTopicList(output_topic_.toStdString());
  }
  Q_EMIT configChanged();
}

void CancelAction::addTopicList(std::string topic_name)
{
  topicListLayout tll;

  if (!topic_list_layouts_.empty()) {
    topicListLayout lastTll = topic_list_layouts_.back();
    tll.id = lastTll.id + 1;
  } else {
    tll.id = 0;
  }

  tll.layout_ = new QHBoxLayout;

  tll.topic_name_ = new QLabel(topic_name.c_str());
  tll.layout_->addWidget(tll.topic_name_);

  tll.remove_button_ = new QPushButton("Delete");
  tll.layout_->addWidget(tll.remove_button_);

  layout->addLayout(tll.layout_);

  //tll.publisher_ = nh_.advertise<actionlib_msgs::msg::GoalID>( topic_name + "/cancel", 1 );
  tll.publisher_ =
    nh_->create_publisher<actionlib_msgs::msg::GoalID>(topic_name + "/cancel", rclcpp::QoS(10));

  topic_list_layouts_.push_back(tll);

  connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
  m_sigmap->setMapping(tll.remove_button_, tll.id);
}

void CancelAction::sendTopic()
{
  std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
  auto msg = std::make_shared<actionlib_msgs::msg::GoalID>();
  while (it != topic_list_layouts_.end()) {
    it->publisher_->publish(*msg);
    it++;
  }
}

void CancelAction::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  rviz_common::Config topic_list_config = config.mapMakeChild("topics");

  std::vector<topicListLayout>::const_iterator it = topic_list_layouts_.begin();
  while (it != topic_list_layouts_.end()) {
    topic_list_config.listAppendNew().setValue(it->topic_name_->text());
    it++;
  }
  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void CancelAction::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  rviz_common::Config topic_list_config = config.mapGetChild("topics");
  int num_topics = topic_list_config.listLength();

  for (int i = 0; i < num_topics; i++) {
    addTopicList(topic_list_config.listChildAt(i).getValue().toString().toStdString());
  }
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::CancelAction, rviz_common::Panel)
