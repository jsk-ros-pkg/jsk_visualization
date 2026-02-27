#ifndef CANCEL_ACTION_H
#define CANCEL_ACTION_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <actionlib_msgs/msg/goal_id.hpp>

#include <rviz_common/panel.hpp>
#include <QtWidgets>
#endif

class QLineEdit;
class QLabel;
class QPushButton;

namespace jsk_rviz_plugins
{
  class CancelAction: public rviz_common::Panel
    {
Q_OBJECT
  public:
      CancelAction( QWidget* parent = 0 );

      virtual void onInitialize();
      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic ) {};

      protected Q_SLOTS:

      void updateTopic() {};

      void sendTopic();
      void addTopic();
      void initComboBox();

      void addTopicList(std::string topic_name);

      void OnClickDeleteButton(int id);

    protected:
      QString output_topic_;

      QPushButton* add_topic_button_;

      QComboBox* add_topic_box_;

      QPushButton* send_topic_button_;

      QSignalMapper *m_sigmap;

      QVBoxLayout* layout;

      struct topicListLayout{
	int id;
	QHBoxLayout* layout_;
	QPushButton* remove_button_;
	QLabel* topic_name_;
	rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr publisher_;
      };

      std::vector<topicListLayout> topic_list_layouts_;

      rclcpp::Node::SharedPtr node_;
    };

}

#endif // CANCEL_ACTION_H
