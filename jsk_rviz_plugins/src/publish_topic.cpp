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
#include "publish_topic.hpp"

#include <stdio.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace jsk_rviz_plugins
{
PublishTopic::PublishTopic(QWidget * parent) : rviz_common::Panel(parent) {}

void PublishTopic::onInitialize()
{
  QHBoxLayout * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Topic:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  // Lay out the topic field above the control widget.
  QVBoxLayout * layout = new QVBoxLayout;
  layout->addLayout(topic_layout);

  QPushButton * send_topic_button_ = new QPushButton("Send Topic");
  layout->addWidget(send_topic_button_);
  setLayout(layout);

  connect(send_topic_button_, SIGNAL(clicked()), this, SLOT(sendTopic()));
  connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void PublishTopic::updateTopic() { setTopic(output_topic_editor_->text()); }

// Set the topic name we are publishing to.
void PublishTopic::setTopic(const QString & new_topic)
{
  // Only take action if the name has changed.
  if (new_topic != output_topic_) {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if (output_topic_ == "") {
      // pub_.shutdown();
      pub_.reset();
    } else {
      //pub_ = nh_.advertise<std_msgs::Empty>(output_topic_.toStdString(), 1);
      pub_ =
        nh_->create_publisher<std_msgs::msg::Empty>(output_topic_.toStdString(), rclcpp::QoS(10));
    }

    Q_EMIT configChanged();
  }
}

void PublishTopic::sendTopic()
{
  // std_msgs::Empty msg;
  // pub_.publish(msg);
  auto msg = std::make_shared<std_msgs::msg::Empty>();
  pub_->publish(*msg);
}

void PublishTopic::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PublishTopic::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::PublishTopic, rviz_common::Panel)
