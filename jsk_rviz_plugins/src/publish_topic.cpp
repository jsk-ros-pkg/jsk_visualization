#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/empty.hpp>
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(std_msgs, Empty)
#else
#include <std_msgs/Empty.h>
#endif

#include "publish_topic.h"

namespace jsk_rviz_plugins
{
  PublishTopic::PublishTopic( QWidget* parent )
    : rviz::Panel( parent )
  {
    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Topic:" ));
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_ );


    // Lay out the topic field above the control widget.
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );

    QPushButton* send_topic_button_ = new QPushButton("Send Topic");
    layout->addWidget( send_topic_button_ );
    setLayout( layout );


    connect( send_topic_button_, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
    connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  }

#if ROS_VERSION_MAJOR != 1
  void PublishTopic::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }
#endif

  void PublishTopic::updateTopic()
  {
    setTopic( output_topic_editor_->text() );
  }

  // Set the topic name we are publishing to.
  void PublishTopic::setTopic( const QString& new_topic )
  {
    // Only take action if the name has changed.
    if( new_topic != output_topic_ )
      {
	output_topic_ = new_topic;
	// If the topic is the empty string, don't publish anything.
	if( output_topic_ == "" )
	  {
#if ROS_VERSION_MAJOR != 1
	    velocity_publisher_.reset();
#else
	    velocity_publisher_.shutdown();
#endif
	  }
	else
	  {
#if ROS_VERSION_MAJOR != 1
	    velocity_publisher_ = node_->create_publisher<std_msgs::Empty>( output_topic_.toStdString(), 1 );
#else
	    velocity_publisher_ = nh_.advertise<std_msgs::Empty>( output_topic_.toStdString(), 1 );
#endif
	  }

	Q_EMIT configChanged();
      }
  }
  
  void PublishTopic::sendTopic(){
    std_msgs::Empty msg;
#if ROS_VERSION_MAJOR != 1
    // a ros::Publisher without a topic ignores publish(); so does this
    if (velocity_publisher_) {
      velocity_publisher_->publish(msg);
    }
#else
    velocity_publisher_.publish(msg);
#endif
  }


  void PublishTopic::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void PublishTopic::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "Topic", &topic ))
      {
	output_topic_editor_->setText( topic );
	updateTopic();
      }
  }

}

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::PublishTopic, rviz::Panel )

