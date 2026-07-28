// -*- mode: c++ -*-
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


#include "tablet_controller_panel.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <OgreVector.h>

namespace jsk_rviz_plugins
{
  static rclcpp::Logger logger()
  {
    return rclcpp::get_logger("TabletControllerPanel");
  }

  TabletCmdVelArea::TabletCmdVelArea(QWidget* parent, TwistPublisher pub_cmd_vel):
    QWidget(parent), mouse_x_(-1), mouse_y_(-1), pub_cmd_vel_(pub_cmd_vel)
  {
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
  }

  QSize TabletCmdVelArea::minimumSizeHint() const
  {
    return QSize(300, 300);
  }

  QSize TabletCmdVelArea::sizeHint() const
  {
    return QSize(300, 300);
  }

  void TabletCmdVelArea::mouseMoveEvent(QMouseEvent* event){
    mouse_x_ = event->x();
    mouse_y_ = event->y();
    repaint();
  }
  void TabletCmdVelArea::mousePressEvent(QMouseEvent* event){
    mouse_x_ = event->x();
    mouse_y_ = event->y();
    repaint();
  }
  void TabletCmdVelArea::mouseReleaseEvent(QMouseEvent* /*event*/){
    mouse_x_ = -1;
    mouse_y_ = -1;
    repaint();
    publishCmdVel(0, 0, 0);
  }

  void TabletCmdVelArea::paintEvent(QPaintEvent* /*event*/)
  {
    QSize widget_size = size();
    int line_width = 20;
    int width = widget_size.width() - line_width * 2;
    int height = widget_size.height() - line_width * 2;
    // out circle
    int radius = std::min(width, height) / 2;
    int center_x = width / 2 + line_width;
    int center_y = height / 2 + line_width;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setColor(QColor(130, 177, 255));
    pen.setWidth(10);
    painter.setPen(pen);
    painter.drawArc(center_x - radius, center_y - radius,
                    radius * 2, radius * 2, 0, (360 + 1) * 16);
    // position of mouse
    QPen inner_pen;
    inner_pen.setColor(QColor(33, 150, 243));
    int inner_size = 40;
    inner_pen.setWidth(inner_size);
    painter.setPen(inner_pen);
    if (mouse_x_ == -1 && mouse_y_ == -1) {
      mouse_x_ = center_x;
      mouse_y_ = center_y;
    }
    else {
      publishVelocity(mouse_x_, mouse_y_, center_x, center_y);
    }
    painter.drawArc(mouse_x_ - inner_size / 2,
		    mouse_y_ - inner_size / 2,
		    inner_size, inner_size, 0, (360 + 1) * 16);
  }

  void TabletCmdVelArea::publishVelocity(
    int mouse_x, int mouse_y, int cx, int cy)
  {
    double diff_x = mouse_x - cx;
    double diff_y = mouse_y - cy;
    // Ogre::Vector3 is used instead of Eigen so that no extra dependency is needed
    Ogre::Vector3 ex(0, -1, 0);
    Ogre::Vector3 vel(diff_x / cx, diff_y / cy, 0);

    int sign = 1;
    if (ex.crossProduct(vel).dotProduct(Ogre::Vector3(0, 0, -1)) < 0) {
      sign = -1;
    }
    double dot = ex.dotProduct(vel) / ex.length() / vel.length();
    if (dot < -1) {
      dot = -1.0;
    }
    else if (dot > 1) {
      dot = 1.0;
    }
    double theta = sign * acos(dot);
    if (!std::isnan(theta)) {
      Ogre::Vector3 vel_refined(-vel[1], -vel[0], 0);

      publishCmdVel(vel_refined[0] * 0.2 , vel_refined[1] * 0.2, theta * 0.2);
    }
  }

  void TabletCmdVelArea::publishCmdVel(double x, double y, double theta)
  {
    RCLCPP_INFO(logger(), "(%f, %f)", x, y);
    RCLCPP_INFO(logger(), "theta: %f", theta);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = theta;
    if (pub_cmd_vel_) {
      pub_cmd_vel_->publish(twist);
    }
  }

  QString TabletControllerPanel::defaultButtonStyleSheet()
  {
    return "QPushButton {background-color: #FF5252; color: white; font-size: 30pt;}";
  }

  QString TabletControllerPanel::executeButtonStyleSheet()
  {
    return "QPushButton {background-color: white; color: #424242; font-size: 30pt;}";
  }

  QString TabletControllerPanel::radioButtonStyleSheet()
  {
    return "QRadioButton {font-size: 20pt; color: #424242;}";
  }

  QString TabletControllerPanel::listStyleSheet()
  {
    return "QListWidget {font-size: 20pt; color: #424242;}";
  }

  TabletControllerPanel::TabletControllerPanel(QWidget* parent):
    rviz_common::Panel(parent), cmd_vel_area_(NULL), task_dialog_(NULL), spot_dialog_(NULL)
  {
    layout_ = new QVBoxLayout();
    setLayout(layout_);
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
  }

  TabletControllerPanel::~TabletControllerPanel()
  {

  }

  void TabletControllerPanel::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    pub_start_demo_ = node_->create_publisher<jsk_rviz_plugins::msg::StringStamped>(
      "/Tablet/StartDemo", 1);
    pub_spot_ = node_->create_publisher<jsk_rviz_plugins::msg::StringStamped>(
      "/Tablet/MoveToSpot", 1);
    sub_spots_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/spots_marker_array", 1,
      std::bind(&TabletControllerPanel::spotCallback, this, std::placeholders::_1));
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/navigation/unsafe_vel", 1);

    layout_->addStretch();
    task_button_ = new QPushButton("Task", this);
    task_button_->setMinimumHeight(100);
    task_button_->setStyleSheet(defaultButtonStyleSheet());

    connect(task_button_, SIGNAL(released()), this, SLOT(taskButtonClicked()));
    layout_->addWidget(task_button_);
    layout_->addSpacing(10);
    spot_button_ = new QPushButton("Move to spot", this);
    spot_button_->setMinimumHeight(100);
    spot_button_->setStyleSheet(defaultButtonStyleSheet());
    connect(spot_button_, SIGNAL(released()), this, SLOT(spotButtonClicked()));
    layout_->addWidget(spot_button_);
    layout_->addSpacing(10);
    cmd_vel_area_ = new TabletCmdVelArea(this, pub_cmd_vel_);
    layout_->addWidget(cmd_vel_area_);
  }

  void TabletControllerPanel::load(const rviz_common::Config& config)
  {
    rviz_common::Panel::load(config);
  }

  void TabletControllerPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  ////////////////////////////////////////////////////////
  // callbacks
  ////////////////////////////////////////////////////////
  void TabletControllerPanel::spotCallback(
    const visualization_msgs::msg::MarkerArray::ConstSharedPtr marker)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    spots_.clear();
    for (size_t i = 0; i < marker->markers.size(); i++) {
      std::string text = marker->markers[i].text;
      if (!text.empty()) {
        spots_.push_back(text);
      }
    }
  }



  void TabletControllerPanel::taskButtonClicked()
  {
    task_dialog_ = new QDialog();
    task_dialog_->setBackgroundRole(QPalette::Base);
    task_dialog_->setAutoFillBackground(true);
    task_dialog_layout_ = new QVBoxLayout();
    task_radio_buttons_.clear();
    std::vector<std::string> tasks;
    tasks.push_back("/Tablet/other/GetGeorgia");
    tasks.push_back("/Tablet/chen/GoToElevator");
    tasks.push_back("/Tablet/chen/Greeting1");
    tasks.push_back("/Tablet/chen/Greeting2");
    tasks.push_back("/Tablet/chen/Greeting3");
    tasks.push_back("/Tablet/chen/Greeting4");
    tasks.push_back("/Tablet/chen/Greeting5");
    tasks.push_back("/Tablet/chen/HandOver");
    for (size_t i = 0; i < tasks.size(); i++) {
      QRadioButton* task = new QRadioButton(QString::fromStdString(tasks[i]), this);
      task->setMinimumHeight(50);
      if (i == 0) {
        task->setChecked(true);
      }
      task->setStyleSheet(radioButtonStyleSheet());
      task_radio_buttons_.push_back(task);
    }

    for (size_t i = 0; i < task_radio_buttons_.size(); i++) {
      task_dialog_layout_->addWidget(task_radio_buttons_[i]);
    }
    task_dialog_button_layout_ = new QHBoxLayout();
    task_execute_button_ = new QPushButton("Execute", this);
    task_execute_button_->setStyleSheet(executeButtonStyleSheet());
    task_execute_button_->setMinimumHeight(100);
    task_execute_button_->setMinimumWidth(300);
    task_dialog_button_layout_->addWidget(task_execute_button_);
    connect(task_execute_button_, SIGNAL(released()), this, SLOT(taskExecuteClicked()));
    task_cancel_button_ = new QPushButton("Cancel", this);
    task_cancel_button_->setStyleSheet(defaultButtonStyleSheet());
    task_cancel_button_->setMinimumHeight(100);
    task_cancel_button_->setMinimumWidth(300);
    connect(task_cancel_button_, SIGNAL(released()), this, SLOT(taskCancelClicked()));
    task_dialog_button_layout_->addWidget(task_cancel_button_);
    task_dialog_layout_->addLayout(task_dialog_button_layout_);
    task_dialog_->setLayout(task_dialog_layout_);
    task_dialog_->show();

  }

  void TabletControllerPanel::taskCancelClicked()
  {
    task_dialog_->reject();
  }

  void TabletControllerPanel::taskExecuteClicked()
  {
    for (size_t i = 0; i < task_radio_buttons_.size(); i++) {
      QRadioButton* radio = task_radio_buttons_[i];
      if (radio->isChecked()) {
        std::string task = radio->text().toStdString();
        RCLCPP_INFO(logger(), "task: %s", task.c_str());
        task_dialog_->reject();
        jsk_rviz_plugins::msg::StringStamped command;
        command.data = task;
        command.header.stamp = node_->now();
        pub_start_demo_->publish(command);
        return;
      }
    }
  }

  void TabletControllerPanel::spotButtonClicked()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    spot_dialog_ = new QDialog();
    spot_dialog_->setBackgroundRole(QPalette::Base);
    spot_dialog_->setAutoFillBackground(true);
    spot_dialog_layout_ = new QVBoxLayout();

    spot_list_ = new QListWidget();
    spot_list_->setSortingEnabled(true);
    spot_list_->setStyleSheet(listStyleSheet());
    for (size_t i = 0; i < spots_.size(); i++) {
      QListWidgetItem* item = new QListWidgetItem(
        QString::fromStdString(spots_[i]));
      item->setSizeHint(QSize(item->sizeHint().width(), 30));
      spot_list_->addItem(item);
    }
    spot_dialog_layout_->addWidget(spot_list_);
    spot_dialog_button_layout_ = new QHBoxLayout();
    spot_go_button_ = new QPushButton("Go", this);
    spot_go_button_->setStyleSheet(executeButtonStyleSheet());
    spot_go_button_->setMinimumHeight(50);
    spot_go_button_->setMinimumWidth(300);
    connect(spot_go_button_, SIGNAL(released()),
            this, SLOT(spotGoClicked()));
    spot_dialog_button_layout_->addWidget(spot_go_button_);

    spot_cancel_button_ = new QPushButton("Cancel", this);
    spot_cancel_button_->setMinimumHeight(50);
    spot_cancel_button_->setMinimumWidth(300);
    spot_cancel_button_->setStyleSheet(defaultButtonStyleSheet());
    connect(spot_cancel_button_, SIGNAL(released()),
            this, SLOT(spotCancelClicked()));
    spot_dialog_button_layout_->addWidget(spot_cancel_button_);
    spot_dialog_layout_->addLayout(spot_dialog_button_layout_);
    spot_dialog_->setLayout(spot_dialog_layout_);
    spot_dialog_->show();
  }

  void TabletControllerPanel::spotCancelClicked()
  {
    spot_dialog_->reject();
  }

  void TabletControllerPanel::spotGoClicked()
  {
    QListWidgetItem* item = spot_list_->currentItem();
    if (item) {
      std::string spot = item->text().toStdString();
      jsk_rviz_plugins::msg::StringStamped spot_command;
      spot_command.data = spot;
      spot_command.header.stamp = node_->now();
      pub_spot_->publish(spot_command);
    }
    spot_dialog_->reject();
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugins::TabletControllerPanel, rviz_common::Panel);
