#ifndef PUBLISH_TOPIC_H
#define PUBLISH_TOPIC_H

#include <ros/ros.h>

#include <rviz/panel.h>

class QLineEdit;
class QPushButton;

namespace jsk_rviz_plugin
{
  class PublishTopic: public rviz::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      // QWidget subclass constructors usually take a parent widget
      // parameter (which usually defaults to 0).  At the same time,
      // pluginlib::ClassLoader creates instances by calling the default
      // constructor (with no arguments).  Taking the parameter and giving
      // a default of 0 lets the default constructor work and also lets
      // someone using the class for something else to pass in a parent
      // widget as they normally would with Qt.
      PublishTopic( QWidget* parent = 0 );

      // Now we declare overrides of rviz::Panel functions for saving and
      // loading data from the config file.  Here the data is the
      // topic name.
      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      // Next come a couple of public Qt slots.
      public Q_SLOTS:
      // In this example setTopic() does not get connected to any signal
      // (it is called directly), but it is easy to define it as a public
      // slot instead of a private function in case it would be useful to
      // some other user.
      void setTopic( const QString& topic );

      // Here we declare some internal slots.
      protected Q_SLOTS:
      // sendvel() publishes the current velocity values to a ROS
      // topic.  Internally this is connected to a timer which calls it 10
      // times per second.
      void sendVel();

      // updateTopic() reads the topic name from the QLineEdit and calls
      // setTopic() with the result.
      void updateTopic();

      void sendTopic();

      // Then we finish up with protected member variables.
    protected:
      // The control-area widget which turns mouse events into command
      // velocities.

      // One-line text editor for entering the outgoing ROS topic name.
      QLineEdit* output_topic_editor_;

      // The current name of the output topic.
      QString output_topic_;

      QPushButton* send_topic_button_;

      // The ROS publisher for the command velocity.
      ros::Publisher velocity_publisher_;

      // The ROS node handle.
      ros::NodeHandle nh_;

    };

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
