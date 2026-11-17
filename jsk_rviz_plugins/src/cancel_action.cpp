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
#include <action_msgs/srv/cancel_goal.hpp>
#include <rviz_common/display_context.hpp>
#else
#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#endif

#include "cancel_action.h"

namespace jsk_rviz_plugins
{
#if ROS_VERSION_MAJOR != 1
  // ROS 2 actions have no cancel topic but this service, see
  // rcl_action/types.h; a zeroed goal_info cancels all the goals.
  static const char* CANCEL_GOAL_SUFFIX = "/_action/cancel_goal";
#endif

  CancelAction::CancelAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Text Box and Add Button to add new topic
    QHBoxLayout* topic_layout = new QHBoxLayout;

    add_topic_box_ = new QComboBox;
#if ROS_VERSION_MAJOR == 1
    // on ROS 2, in onInitialize(), with the node of rviz
    initComboBox();
#endif
    topic_layout->addWidget( add_topic_box_ );

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

#if ROS_VERSION_MAJOR != 1
  void CancelAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    initComboBox();
  }
#endif

  void CancelAction::initComboBox(){
    add_topic_box_->addItem("");
#if ROS_VERSION_MAJOR != 1
    // actions are found by their cancel service
    const std::string suffix(CANCEL_GOAL_SUFFIX);
    for (const auto& service : node_->get_service_names_and_types()) {
      for (const auto& type : service.second) {
        const std::string& service_name = service.first;
        if (type == "action_msgs/srv/CancelGoal" &&
            service_name.length() > suffix.length() &&
            service_name.compare(service_name.length() - suffix.length(),
                                 suffix.length(), suffix) == 0) {
          std::string action_name =
            service_name.substr(0, service_name.length() - suffix.length());
          add_topic_box_->addItem(action_name.c_str());
        }
      }
    }
#else
    ros::master::V_TopicInfo topics;
    ros::master::getTopics (topics);
    ros::master::V_TopicInfo::iterator it = topics.begin();
    while( it != topics.end()){
      if(it->datatype == "actionlib_msgs/GoalID"){
	std::string action_name = it->name;
	std::string delete_string = "/cancel";
	std::string::size_type index = action_name.find_last_of(delete_string);
	if(index != std::string::npos){
	  action_name.erase(index - delete_string.length() + 1);
	  add_topic_box_->addItem(action_name.c_str());
	}
      }
      it ++;
    }
#endif
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
#if ROS_VERSION_MAJOR != 1
	it->client_.reset();
#else
	it->publisher_.shutdown();
#endif
	it = topic_list_layouts_.erase( it );
	Q_EMIT configChanged();
      }else{
	++it;
      }
    }
  }

  void CancelAction::addTopic()
  {
    output_topic_ = add_topic_box_->currentText();
    if( output_topic_ != "" ){
      add_topic_box_->setCurrentIndex( 0 );
      addTopicList(output_topic_.toStdString());
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

#if ROS_VERSION_MAJOR != 1
    tll.client_ = node_->create_client<action_msgs::srv::CancelGoal>( topic_name + CANCEL_GOAL_SUFFIX );
#else
    tll.publisher_ = nh_.advertise<actionlib_msgs::GoalID>( topic_name + "/cancel", 1 );
#endif
    
    topic_list_layouts_.push_back(tll);

    connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
    m_sigmap->setMapping(tll.remove_button_, tll.id);

  }

  void CancelAction::sendTopic(){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
#if ROS_VERSION_MAJOR != 1
      const std::string action_name = it->topic_name_->text().toStdString();
      if (!it->client_->service_is_ready()) {
        ROS_WARN("%s%s is not available", action_name.c_str(), CANCEL_GOAL_SUFFIX);
        it++;
        continue;
      }
      // rviz spins its node in the GUI thread, so do not block on the
      // response; it is logged once rviz processes it
      it->client_->async_send_request(
        std::make_shared<action_msgs::srv::CancelGoal::Request>(),
        [action_name](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
          const auto response = future.get();
          if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            ROS_INFO("Cancelled %zu goal(s) of %s",
                     response->goals_canceling.size(), action_name.c_str());
          } else {
            ROS_WARN("Failed to cancel the goals of %s, return_code=%d",
                     action_name.c_str(), static_cast<int>(response->return_code));
          }
        });
#else
      actionlib_msgs::GoalID msg;
      it->publisher_.publish(msg);
#endif
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

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::CancelAction, rviz::Panel )
