#ifndef ROBOT_COMMAND_INTERFACE_H
#define ROBOT_COMMAND_INTERFACE_H

#ifndef Q_MOC_RUN
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/panel.hpp>
#else
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#if ROS_VERSION_MAJOR != 1
#include <jsk_rviz_plugins/srv/eus_command.hpp>
#include <resource_retriever/retriever.hpp>
#else
#include <jsk_rviz_plugins/EusCommand.h>
#include <resource_retriever/retriever.h>
#endif
#endif

namespace jsk_rviz_plugins
{
  class RobotCommandInterfaceAction: public rviz::Panel
  {
    Q_OBJECT
    public:
    RobotCommandInterfaceAction( QWidget* parent = 0 );
#if ROS_VERSION_MAJOR != 1
    // the node of rviz2 is only available here, not in the constructor
    virtual void onInitialize();
#endif

  protected Q_SLOTS:
    bool callRequestEusCommand(const std::string& command);
    void buttonCallback(int i);
  protected:
    void popupDialog(const std::string& text);
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node_;
#else
    // The ROS node handle.
    ros::NodeHandle nh_;
#endif
    QSignalMapper* signal_mapper_;
    std::map<int, std::string> euscommand_mapping_;
    std::map<int, std::string> emptyservice_mapping_;
    //std::vector<QToolButton*> buttons_;
  };

}

#endif // TELEOP_PANEL_H
