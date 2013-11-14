#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/Empty.h>

#include "cancel_action.h"

namespace jsk_rviz_plugin
{

  CancelAction::CancelAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    //return;
    // Next we lay out the "output topic" text entry field using a
    // QLabel and a QLineEdit in a QHBoxLayout.
    QHBoxLayout* topic_layout = new QHBoxLayout;
    //topic_layout->addWidget( new QLabel( "Topic:" ));
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_ );

    QPushButton* add_topic_button_ = new QPushButton("Add Action");
    topic_layout->addWidget( add_topic_button_ );


    // Lay out the topic field above the control widget.
    layout = new QVBoxLayout;
    layout->addLayout( topic_layout );



    //QLabel* tmp_topic_list_ = new QLabel();

    //test

    m_sigmap = new QSignalMapper(this);

    /*
    std::vector<std::string> topic_list_;
    topic_list_.push_back("abc");
    topic_list_.push_back("def");
    topic_list_.push_back("aaa");
    topic_list_.push_back("bbb");
    topic_list_.push_back("ccc");
    int i=0;
    std::vector<std::string>::iterator it = topic_list_.begin();
    for( it = topic_list_.begin(); it != topic_list_.end(); ++it )
      {
	topicListLayout tll;
	tll.id = i;
	i++;
	tll.layout_ = new QHBoxLayout;
	
	tll.topic_name_ = new QLabel( (*it).c_str() );
	tll.layout_->addWidget( tll.topic_name_ );
	
	tll.remove_button_ = new QPushButton("Delete");
	tll.layout_->addWidget( tll.remove_button_ );

	layout->addLayout(tll.layout_);
	
	tll.publisher_ = nh_.advertise<std_msgs::Empty>( (*it).c_str(), 1 );
	topic_list_layouts_.push_back(tll);
	//cout << *it << endl;
	//	connect( topic_remove_button, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
	connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
	//connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
	m_sigmap->setMapping(tll.remove_button_, tll.id);
	//m_sigmap->setMapping(tll.remove_button_, tll.layout_);
	//m_sigmap->setMapping(tll.remove_button_, tll.topic_name_);
      }
    //    connect(m_sigmap, SIGNAL(mapped(QWidget *)),this, SLOT(OnClickButton(QWidget *)));
    */
    connect(m_sigmap, SIGNAL(mapped(int)),this, SLOT(OnClickDeleteButton(int)));
    //connect(m_sigmap, SIGNAL(mapped(QWidget *)),this, SLOT(OnClickButton(QObject *)));

    /*
    m_sigmap = new QSignalMapper(this);
    std::vector<std::string> topic_list_;
    topic_list_.push_back("abc");
    topic_list_.push_back("def");
    std::vector<std::string>::iterator it = topic_list_.begin();
    for( it = topic_list_.begin(); it != topic_list_.end(); ++it )
      {
	QHBoxLayout* topic_list_layout = new QHBoxLayout;
	topic_list_layout->addWidget( new QLabel( (*it).c_str() ));

	QPushButton* topic_remove_button = new QPushButton("Send Topic");
	topic_list_layout->addWidget( topic_remove_button );

	layout->addLayout(topic_list_layout);
	//cout << *it << endl;
	//	connect( topic_remove_button, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
	connect(topic_remove_button, SIGNAL(clicked()), m_sigmap, SLOT(map()));
	m_sigmap->setMapping(topic_remove_button, topic_list_layout);
      }
    connect(m_sigmap, SIGNAL(mapped(QWidget *)),this, SLOT(OnClickButton(QWidget *)));
    */
    //test
    //tmp_topic_list_->setText("aaa<br>aaa");
    //layout->addWidget( tmp_topic_list_ );

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
    //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    connect( add_topic_button_, SIGNAL( clicked() ), this, SLOT( addTopic() ));
    std::cout<<"aa"<<std::endl;
  }
  //void CancelAction::OnClickButton(QObject *topic_list_layout){
  //void CancelAction::OnClickButton(QWidget *topic_list_layout){
  void CancelAction::OnClickDeleteButton(int id){
    std::cout<<"vv"<<std::endl;
    std::cout<<id<<std::endl;
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    //for( it = topic_list_layouts_.begin(); it != topic_list_layouts_.end(); ++it ){
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
    //layout->removeWidget(topic_list_layout);
    //layout->removeWidget(topic_list_layouts_[0].layout_);
    //layout->removeWidget(topic_list_layout);
    //setLayout( layout );
  }

  // Read the topic name from the QLineEdit and call setTopic() with the
  // results.  This is connected to QLineEdit::editingFinished() which
  // fires when the user presses Enter or Tab or otherwise moves focus
  // away.
  void CancelAction::updateTopic()
  {
    setTopic( output_topic_editor_->text() );
  }

  // Set the topic name we are publishing to.
  void CancelAction::setTopic( const QString& new_topic )
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
    //std::cout<<tll.id<<endl;

    tll.layout_ = new QHBoxLayout;

    tll.topic_name_ = new QLabel( topic_name.c_str() );
    tll.layout_->addWidget( tll.topic_name_ );

    tll.remove_button_ = new QPushButton("Delete");
    tll.layout_->addWidget( tll.remove_button_ );

    layout->addLayout(tll.layout_);

    tll.publisher_ = nh_.advertise<std_msgs::Empty>( topic_name, 1 );
    topic_list_layouts_.push_back(tll);

    connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
    m_sigmap->setMapping(tll.remove_button_, tll.id);

  }
  
  void CancelAction::sendTopic(){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      std_msgs::Empty msg;
      it->publisher_.publish(msg);
      it++;
    }

    //std_msgs::Empty msg;
    //velocity_publisher_.publish(msg);
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void CancelAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    //config.
    //rviz::Config config; 
    //config.mapSetValue( "Height", height() ); config.mapSetValue( "Width", width() );    
    std::cout<<"save"<<std::endl; 

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
    std::cout<<"load"<<num_topics<<std::endl;
    for( int i = 0; i < num_topics; i++ ) {
      addTopicList(topic_list_config.listChildAt( i ).getValue().toString().toStdString());
      std::cout<<"load"<<std::endl;
      //filenames_.push_back( file_list_config.listChildAt( i ).getValue().toString() );
    }

    /*
    QString topic;
    if( config.mapGetString( "Topic", &topic ))
      {
	output_topic_editor_->setText( topic );
	updateTopic();
      }
    */
  }

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::CancelAction, rviz::Panel )
// END_TUTORIAL

