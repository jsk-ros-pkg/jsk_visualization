#ifndef SELECT_POINT_CLOUD_PUBLISH_ACTION_H
#define SELECT_POINT_CLOUD_PUBLISH_ACTION_H

#ifndef Q_MOC_RUN
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, PointCloud2)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, PointField)
#else
#include <ros/ros.h>

#include <rviz/panel.h>
#endif
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;
class PropertyTreeWidget;


namespace jsk_rviz_plugins
{
  class SelectPointCloudPublishAction: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      SelectPointCloudPublishAction( QWidget* parent = 0 );
#if ROS_VERSION_MAJOR != 1
      // the node of rviz2 is only available here, not in the constructor
      virtual void onInitialize();
#endif

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      protected Q_SLOTS:

      void publishPointCloud();
    protected:
      QPushButton* publish_pointcloud_button_;

      QVBoxLayout* layout;

#if ROS_VERSION_MAJOR != 1
      rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr select_pointcloud_publisher_;

      rclcpp::Node::SharedPtr node_;
#else
      // The ROS publisher for the command velocity.
      ros::Publisher select_pointcloud_publisher_;

      // The ROS node handle.
      ros::NodeHandle nh_;
#endif

    };

}

#endif // TELEOP_PANEL_H
