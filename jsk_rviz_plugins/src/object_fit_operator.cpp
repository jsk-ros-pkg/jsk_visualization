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
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTabWidget>
#include <QToolButton>
#include <QVBoxLayout>
// #include <ros/package.h>
#include <rviz_common/display_context.hpp>

#include "object_fit_operator.hpp"

using namespace rviz_common;
namespace jsk_rviz_plugins
{
ObjectFitOperatorAction::ObjectFitOperatorAction(QWidget * parent) : rviz_common::Panel(parent) {}

void ObjectFitOperatorAction::onInitialize()
{
  layout = new QVBoxLayout;

  horizontal_layout1_ = new QHBoxLayout();
  horizontal_layout2_ = new QHBoxLayout();

  //Button to send cancel topic
  std::string fit_button_name, reverse_fit_button_name, near_button_name, other_button_name;
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Todo: get icon file path
  std::string package_path = "";
  fit_button_name = package_path + std::string("/icons/fit.jpg");
  near_button_name = package_path + std::string("/icons/near.jpg");
  other_button_name = package_path + std::string("/icons/other.jpg");

  // nh_.param<std::string>("/object_fit_icon", fit_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/fit.jpg"));
  // nh_.param<std::string>("/object_near_icon", near_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/near.jpg"));
  // nh_.param<std::string>("/object_other_icon", other_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/other.jpg"));

  QSize iconSize(150, 150);
  fit_button_ = new QToolButton();
  fit_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  fit_button_->setIcon(QIcon(QPixmap(QString(fit_button_name.c_str()))));
  fit_button_->setText("Onto");
  fit_button_->setIconSize(iconSize);
  horizontal_layout1_->addWidget(fit_button_);

  check_box_ = new QCheckBox(QString("Reverse"));
  horizontal_layout1_->addWidget(check_box_);

  near_button_ = new QToolButton();
  near_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  near_button_->setIcon(QIcon(QPixmap(QString(near_button_name.c_str()))));
  near_button_->setIconSize(iconSize);
  near_button_->setText("Parallel");
  horizontal_layout2_->addWidget(near_button_);

  other_button_ = new QToolButton();
  other_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  other_button_->setIcon(QIcon(QPixmap(QString(other_button_name.c_str()))));
  other_button_->setText("Perpendicular");
  other_button_->setIconSize(iconSize);
  horizontal_layout2_->addWidget(other_button_);

  layout->addLayout(horizontal_layout1_);
  layout->addLayout(horizontal_layout2_);
  setLayout(layout);

  connect(fit_button_, SIGNAL(clicked()), this, SLOT(commandFit()));
  connect(check_box_, SIGNAL(clicked(bool)), this, SLOT(checkBoxChanged(bool)));
  connect(near_button_, SIGNAL(clicked()), this, SLOT(commandNear()));
  connect(other_button_, SIGNAL(clicked()), this, SLOT(commandOther()));

  // pub_ = nh_.advertise<jsk_rviz_plugin_msgs::msg::ObjectFitCommand>( "/object_fit_command", 1 );
  pub_ = nh_->create_publisher<jsk_rviz_plugin_msgs::msg::ObjectFitCommand>(
    "/object_fit_command", rclcpp::QoS(10));
}

void ObjectFitOperatorAction::checkBoxChanged(bool state) { reverse_ = state; }

void ObjectFitOperatorAction::commandFit()
{
  if (reverse_)
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::REVERSE_FIT);
  else
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::FIT);
}

void ObjectFitOperatorAction::commandNear()
{
  if (reverse_)
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::REVERSE_NEAR);
  else
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::NEAR);
}

void ObjectFitOperatorAction::commandOther()
{
  if (reverse_)
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::REVERSE_OTHER);
  else
    publishObjectFitOder(jsk_rviz_plugin_msgs::msg::ObjectFitCommand::OTHER);
}

void ObjectFitOperatorAction::publishObjectFitOder(int type)
{
  auto msg = std::make_shared<jsk_rviz_plugin_msgs::msg::ObjectFitCommand>();
  msg->command = type;
  // pub_.publish(msg);
  pub_->publish(*msg);
}

void ObjectFitOperatorAction::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void ObjectFitOperatorAction::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}
}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::ObjectFitOperatorAction, rviz_common::Panel)
