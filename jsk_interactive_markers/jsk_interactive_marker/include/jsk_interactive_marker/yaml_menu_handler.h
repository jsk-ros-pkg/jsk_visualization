#ifndef __YAML_MENU_HANDLER_H__
#define __YAML_MENU_HANDLER_H__

#include <vector>
#include <yaml-cpp/yaml.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_msgs/String.h>
#include <map>

namespace jsk_interactive_marker {
  class YamlMenuHandler {
   public:
    ros::NodeHandle* _node_ptr;
    interactive_markers::MenuHandler _menu_handler;
    std::map<std::string, ros::Publisher> _publisher_map;
    YamlMenuHandler(ros::NodeHandle* node_ptr, std::string file_name);
    bool initMenu(std::string file);
    void pubTopic(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string topic_name);
    void applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name);
  };
};

#endif
