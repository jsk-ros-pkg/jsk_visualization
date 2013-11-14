#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/Empty.h>

#include "publish_topic.h"

namespace jsk_rviz_plugin
{

  // BEGIN_TUTORIAL
  // Here is the implementation of the TeleopPanel class.  TeleopPanel
  // has these responsibilities:
  //
  // - Act as a container for GUI elements DriveWidget and QLineEdit.
  // - Publish command velocities 10 times per second (whether 0 or not).
  // - Saving and restoring internal state from a config file.
  //
  // We start with the constructor, doing the standard Qt thing of
  // passing the optional *parent* argument on to the superclass
  // constructor, and also zero-ing the velocities we will be
  // publishing.
  PublishTopic::PublishTopic( QWidget* parent )
    : rviz::Panel( parent )
  {
    //return;
    // Next we lay out the "output topic" text entry field using a
    // QLabel and a QLineEdit in a QHBoxLayout.
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

    // Create a timer for sending the output.  Motor controllers want to
    // be reassured frequently that they are doing the right thing, so
    // we keep re-sending velocities even when they aren't changing.
    // 
    // Here we take advantage of QObject's memory management behavior:
    // since "this" is passed to the new QTimer as its parent, the
    // QTimer is deleted by the QObject destructor when this TeleopPanel
    // object is destroyed.  Therefore we don't need to keep a pointer
    // to the timer.

    // Next we make signal/slot connections.
    connect( send_topic_button_, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
    connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  }

  // Read the topic name from the QLineEdit and call setTopic() with the
  // results.  This is connected to QLineEdit::editingFinished() which
  // fires when the user presses Enter or Tab or otherwise moves focus
  // away.
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
	    velocity_publisher_.shutdown();
	  }
	else
	  {
	    // The old ``velocity_publisher_`` is destroyed by this assignment,
	    // and thus the old topic advertisement is removed.  The call to
	    // nh_advertise() says we want to publish data on the new topic
	    // name.
	    velocity_publisher_ = nh_.advertise<std_msgs::Empty>( output_topic_.toStdString(), 1 );
	  }
	// rviz::Panel defines the configChanged() signal.  Emitting it
	// tells RViz that something in this panel has changed that will
	// affect a saved config file.  Ultimately this signal can cause
	// QWidget::setWindowModified(true) to be called on the top-level
	// rviz::VisualizationFrame, which causes a little asterisk ("*")
	// to show in the window's title bar indicating unsaved changes.
	Q_EMIT configChanged();
      }
  }
  
  void PublishTopic::sendTopic(){
    std_msgs::Empty msg;
    velocity_publisher_.publish(msg);
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
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

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::PublishTopic, rviz::Panel )
// END_TUTORIAL
