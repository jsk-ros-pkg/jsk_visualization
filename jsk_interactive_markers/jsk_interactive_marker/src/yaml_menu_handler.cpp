#include <jsk_interactive_marker/yaml_menu_handler.h>

using namespace jsk_interactive_marker;
YamlMenuHandler::YamlMenuHandler(ros::NodeHandle* node_ptr, std::string file_name) {
  ROS_INFO("hoge %s ", file_name.c_str());
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
    std::string text, service_name;
    const YAML::Node single_menu = doc[i];
#ifdef USE_OLD_YAML
    single_menu["text"] >> text;
    single_menu["service"] >> service_name;
#else
    text = single_menu["text"].as<std::string>();
    service_name = single_menu["service"].as<std::string>();
#endif
    ROS_INFO("Regist %s, %s", text.c_str(), service_name.c_str());
    _menu_handler.insert(text, boost::bind(&YamlMenuHandler::callService, this, _1, service_name));
  }
}

void YamlMenuHandler::callService(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string service_name) {
  ros::ServiceClient client = _node_ptr->serviceClient<std_srvs::Empty>(service_name);
  std_srvs::Empty srv;
  ROS_INFO("calling service: %s", service_name.c_str());
  if(!client.call(srv)){
    ROS_INFO("cannot call service: %s", service_name.c_str());
    return;
  }
  else{
    ROS_INFO("succeeded in calling service: %s", service_name.c_str());
  }
}

void YamlMenuHandler::applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name) {
  _menu_handler.apply(*server_ptr, name);
}
