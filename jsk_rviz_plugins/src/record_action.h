#ifndef RECORD_ACTION_H
#define RECORD_ACTION_H

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
#include <jsk_rviz_plugins/msg/record_command.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(jsk_rviz_plugins, RecordCommand)
#else
#include <jsk_rviz_plugins/RecordCommand.h>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;

namespace jsk_rviz_plugins
{
  class RecordAction: public rviz::Panel
  {
    enum RecordState{
      IDLE = 0,
      RECORD = 1
    };
    Q_OBJECT
    public:
    RecordAction( QWidget* parent = 0 );

#if ROS_VERSION_MAJOR != 1
    virtual void onInitialize();
#endif
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  public Q_SLOTS:

    void setTopic( const QString& topic ) {}

  protected Q_SLOTS:

    void updateTopic() {};

    void commandPlay() {};
    void recordClick();

    void addTopicList(std::string topic_name);

    void OnClickPlayButton(int id);
    void OnClickDeleteButton(int id);

  protected:
    QLineEdit* record_motion_name_editor_;

    QString output_topic_;

    QPushButton* record_interface_button_;

    QComboBox* add_topic_box_;

    QSignalMapper *m_delete_sigmap_;
    QSignalMapper *m_play_sigmap_;

    QVBoxLayout* layout;

    struct motionListLayout{
      int id;
      QHBoxLayout* layout_;
      QPushButton* play_button_;
      QPushButton* remove_button_;
      QLabel* target_name_;
    };

    std::vector<motionListLayout> motion_list_layouts_;

#if ROS_VERSION_MAJOR != 1
    rclcpp::Publisher<jsk_rviz_plugins::RecordCommand>::SharedPtr pub_;
    rclcpp::Node::SharedPtr node_;
#else
    ros::Publisher pub_;
    ros::NodeHandle nh_;
#endif
    RecordState rstate_;
  };

}

#endif
