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

#ifndef JSK_RVIZ_PLUGINS_TABLET_CONTROLLER_PANEL_H_
#define JSK_RVIZ_PLUGINS_TABLET_CONTROLLER_PANEL_H_

#ifndef Q_MOC_RUN
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialog>
#include <QListWidget>
#include <QListWidgetItem>
#include <QLabel>
#include <QTimer>
#include <QRadioButton>
#include <QPaintEvent>
#include <QMouseEvent>
#include <geometry_msgs/msg/twist.hpp>
#include <jsk_rviz_plugins/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace jsk_rviz_plugins
{
  class TabletCmdVelArea: public QWidget
  {
    Q_OBJECT
  public:
    typedef rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr TwistPublisher;
    TabletCmdVelArea(QWidget* parent, TwistPublisher pub_cmd_vel);
    virtual QSize minimumSizeHint() const;
    virtual QSize sizeHint() const;
  protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void publishVelocity(int mouse_x, int mouse_y, int cx, int cy);
    virtual void publishCmdVel(double x, double y, double theta);
    int mouse_x_;
    int mouse_y_;
    TwistPublisher pub_cmd_vel_;
  };

  class TabletControllerPanel: public rviz_common::Panel
  {
    Q_OBJECT
  public:
    TabletControllerPanel(QWidget* parent = 0);
    virtual ~TabletControllerPanel();
    virtual void onInitialize() override;
    virtual void load(const rviz_common::Config& config);
    virtual void save(rviz_common::Config config) const;

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void spotCallback(
      const visualization_msgs::msg::MarkerArray::ConstSharedPtr marker);
    virtual QString defaultButtonStyleSheet();
    virtual QString executeButtonStyleSheet();
    virtual QString radioButtonStyleSheet();
    virtual QString listStyleSheet();
    ////////////////////////////////////////////////////////
    // GUI variables
    ////////////////////////////////////////////////////////

    QVBoxLayout* layout_;
    QPushButton* task_button_;
    QPushButton* spot_button_;
    TabletCmdVelArea* cmd_vel_area_;

    QDialog* task_dialog_;
    QVBoxLayout* task_dialog_layout_;
    QHBoxLayout* task_dialog_button_layout_;
    QPushButton* task_execute_button_;
    QPushButton* task_cancel_button_;
    std::vector<QRadioButton*> task_radio_buttons_;

    std::vector<std::string> spots_;
    QDialog* spot_dialog_;
    QVBoxLayout* spot_dialog_layout_;
    QHBoxLayout* spot_dialog_button_layout_;
    QPushButton* spot_go_button_;
    QPushButton* spot_cancel_button_;
    QListWidget* spot_list_;
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<jsk_rviz_plugins::msg::StringStamped>::SharedPtr pub_spot_;
    rclcpp::Publisher<jsk_rviz_plugins::msg::StringStamped>::SharedPtr pub_start_demo_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_spots_;
    std::mutex mutex_;


  protected Q_SLOTS:
    ////////////////////////////////////////////////////////
    // callbacks
    ////////////////////////////////////////////////////////
    void taskButtonClicked();
    void taskCancelClicked();
    void taskExecuteClicked();
    void spotButtonClicked();
    void spotGoClicked();
    void spotCancelClicked();
  private:

  };
}

#endif
