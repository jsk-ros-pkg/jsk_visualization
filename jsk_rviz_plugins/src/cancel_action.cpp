#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>

#include "cancel_action.h"

namespace jsk_rviz_plugin
{

  CancelAction::CancelAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Text Box and Add Button to add new topic
    QHBoxLayout* topic_layout = new QHBoxLayout;
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_ );

    QPushButton* add_topic_button_ = new QPushButton("Add Action");
    topic_layout->addWidget( add_topic_button_ );

    layout->addLayout( topic_layout );
    //End of Text Box and Add Button

    m_sigmap = new QSignalMapper(this);

    connect(m_sigmap, SIGNAL(mapped(int)),this, SLOT(OnClickDeleteButton(int)));

    //Button to send cancel topic
    QPushButton* send_topic_button_ = new QPushButton("Cancel Action");
    layout->addWidget( send_topic_button_ );

    setLayout( layout );

    connect( send_topic_button_, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
    connect( add_topic_button_, SIGNAL( clicked() ), this, SLOT( addTopic() ));
  }


  void CancelAction::OnClickDeleteButton(int id){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      if(it->id == id){
	it->topic_name_->hide();
	delete it->topic_name_;

	it->remove_button_->hide();
	delete it->remove_button_;

	delete it->layout_;
	it->publisher_.shutdown();
	it = topic_list_layouts_.erase( it );
	Q_EMIT configChanged();
      }else{
	++it;
      }
    }
  }

  void CancelAction::addTopic()
  {
    output_topic_ = output_topic_editor_->text();
    if( output_topic_ != "" ){
      addTopicList(output_topic_.toStdString());
      output_topic_editor_->setText("");
    }
    Q_EMIT configChanged();
  }

  void CancelAction::addTopicList(std::string topic_name){
    topicListLayout tll;

    if(!topic_list_layouts_.empty()){
      topicListLayout lastTll = topic_list_layouts_.back();
      tll.id = lastTll.id + 1;
    }else{
      tll.id = 0;
    }

    tll.layout_ = new QHBoxLayout;

    tll.topic_name_ = new QLabel( topic_name.c_str() );
    tll.layout_->addWidget( tll.topic_name_ );

    tll.remove_button_ = new QPushButton("Delete");
    tll.layout_->addWidget( tll.remove_button_ );

    layout->addLayout(tll.layout_);

    tll.publisher_ = nh_.advertise<actionlib_msgs::GoalID>( topic_name + "/cancel", 1 );
    
    topic_list_layouts_.push_back(tll);

    connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
    m_sigmap->setMapping(tll.remove_button_, tll.id);

  }

  void CancelAction::sendTopic(){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      actionlib_msgs::GoalID msg;
      it->publisher_.publish(msg);
      it++;
    }
  }

  void CancelAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );

    rviz::Config topic_list_config = config.mapMakeChild( "topics" );

    std::vector<topicListLayout>::const_iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      topic_list_config.listAppendNew().setValue( it->topic_name_->text() );
      it ++;
    }
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void CancelAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    rviz::Config topic_list_config = config.mapGetChild( "topics" );
    int num_topics = topic_list_config.listLength();

    for( int i = 0; i < num_topics; i++ ) {
      addTopicList(topic_list_config.listChildAt( i ).getValue().toString().toStdString());
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::CancelAction, rviz::Panel )
