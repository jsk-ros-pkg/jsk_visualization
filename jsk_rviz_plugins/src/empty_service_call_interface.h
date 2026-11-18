#ifndef EMPTY_SERCICE_CALL_INTERFACE_H
#define EMPTY_SERCICE_CALL_INTERFACE_H

#ifndef Q_MOC_RUN
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/empty.hpp>
#include <map>
#else
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>
#endif
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

namespace jsk_rviz_plugins
{
  struct ServiceCallButtonInfo
  {
    std::string icon_file_path;
    std::string service_name;
    std::string text;
  };

  class EmptyServiceCallInterfaceAction: public rviz::Panel
  {
    Q_OBJECT
    public:
    EmptyServiceCallInterfaceAction( QWidget* parent = 0 );

#if ROS_VERSION_MAJOR != 1
    virtual void onInitialize();
#endif
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  protected Q_SLOTS:
    void callRequestEmptyCommand(int button_id);
    void parseROSParameters();
  protected:
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node_;
    std::map<std::string, rclcpp::Client<std_srvs::srv::Empty>::SharedPtr> clients_;
#else
    ros::NodeHandle nh_;
#endif
    std::vector<ServiceCallButtonInfo> service_call_button_infos_;
    QVBoxLayout* layout;
    QHBoxLayout* h_layout;
    QSignalMapper* signal_mapper;
  };
}

#endif
