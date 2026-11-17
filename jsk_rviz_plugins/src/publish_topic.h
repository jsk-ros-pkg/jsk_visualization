#ifndef PUBLISH_TOPIC_H
#define PUBLISH_TOPIC_H

#ifndef Q_MOC_RUN
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>
#else
#include <ros/ros.h>

#include <rviz/panel.h>
#endif
#endif

class QLineEdit;
class QPushButton;

namespace jsk_rviz_plugins
{
  class PublishTopic: public rviz::Panel
    {
Q_OBJECT
  public:
      PublishTopic( QWidget* parent = 0 );

#if ROS_VERSION_MAJOR != 1
      virtual void onInitialize();
#endif
      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic );

      protected Q_SLOTS:

      void sendVel() {}

      void updateTopic();

      void sendTopic();

    protected:
      QLineEdit* output_topic_editor_;

      QString output_topic_;

      QPushButton* send_topic_button_;

#if ROS_VERSION_MAJOR != 1
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr velocity_publisher_;

      rclcpp::Node::SharedPtr node_;
#else
      ros::Publisher velocity_publisher_;

      ros::NodeHandle nh_;
#endif
    };

}

#endif
