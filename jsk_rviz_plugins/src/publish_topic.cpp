#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <rviz_common/display_context.hpp>
#include "publish_topic.hpp"

namespace jsk_rviz_plugins
{
  PublishTopic::PublishTopic( QWidget* parent )
    : rviz_common::Panel( parent ) { }

  void PublishTopic::onInitialize()
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

    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  void PublishTopic::updateTopic()
  {
    setTopic( output_topic_editor_->text() );
  }

  // Set the topic name we are publishing to.
  void PublishTopic::setTopic( const QString& new_topic )
  {
    // Only take action if the name has changed.
    if (new_topic != output_topic_)
    {
      output_topic_ = new_topic;
      // If the topic is the empty string, don't publish anything.
      if (output_topic_ == "")
      {
        // pub_.shutdown();
        pub_.reset();
      }
      else
      {
        //pub_ = nh_.advertise<std_msgs::Empty>(output_topic_.toStdString(), 1);
        pub_ = nh_->create_publisher<std_msgs::msg::Empty>(output_topic_.toStdString(), rclcpp::QoS(10));
      }

      Q_EMIT configChanged();
    }
  }
  
  void PublishTopic::sendTopic(){
    // std_msgs::Empty msg;
    // pub_.publish(msg);
    auto msg = std::make_shared<std_msgs::msg::Empty>();
    pub_->publish(*msg);
  }

  void PublishTopic::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void PublishTopic::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
    QString topic;
    if( config.mapGetString( "Topic", &topic ))
      {
        output_topic_editor_->setText( topic );
        updateTopic();
      }
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::PublishTopic, rviz_common::Panel )

