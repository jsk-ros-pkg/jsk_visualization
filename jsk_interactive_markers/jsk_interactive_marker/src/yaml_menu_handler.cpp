#include <jsk_interactive_marker/yaml_menu_handler.h>
#include <fstream>

using namespace jsk_interactive_marker;
YamlMenuHandler::YamlMenuHandler(ros::NodeHandle* node_ptr, std::string file_name) {
  _node_ptr = node_ptr;
  initMenu(file_name);
}

bool YamlMenuHandler::initMenu(std::string file_name) {
  YAML::Node doc;
#ifdef USE_OLD_YAML
  std::ifstream fin(file_name.c_str());
  ROS_INFO("opening yaml file  %s", file_name.c_str());
  if (!fin.good()){
    ROS_INFO("Unable to open yaml file %s", file_name.c_str());
    return false;
  }
  YAML::Parser parser(fin);
  if (!parser) {
    ROS_INFO("Unable to create YAML parser for marker_set");
    return false;
  }
  parser.GetNextDocument(doc);
#else
  // yaml-cpp is greater than 0.5.0
  ROS_INFO("opening yaml file with new yaml-cpp %s", file_name.c_str());
  if ( !(access(file_name.c_str(), F_OK) != -1)) {
    ROS_INFO("file not exists :%s", file_name.c_str());
    return false;
  }
  doc = YAML::LoadFile(file_name);
#endif
  for (int i=0; i<doc.size() ;i++) {
    std::string text, topic_name;
    const YAML::Node& single_menu = doc[i];
#ifdef USE_OLD_YAML
    single_menu["text"] >> text;
    single_menu["topic"] >> topic_name;
#else
    text = single_menu["text"].as<std::string>();
    topic_name = single_menu["topic"].as<std::string>();
#endif
    ROS_INFO("Regist %s, %s", text.c_str(), topic_name.c_str());
    _publisher_map[topic_name] = _node_ptr->advertise<std_msgs::String>(topic_name, 1);
    _menu_handler.insert(text, boost::bind(&YamlMenuHandler::pubTopic, this, _1, topic_name));
  }
  return true;
}

void YamlMenuHandler::pubTopic(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string topic_name) {
  std_msgs::String text_msg;
  text_msg.data = feedback->marker_name;
  if (_publisher_map.find(topic_name)==_publisher_map.end()) {
    return;
  }
  else {
    _publisher_map[topic_name].publish(text_msg);
  }
}

void YamlMenuHandler::applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name) {
  _menu_handler.apply(*server_ptr, name);
}
