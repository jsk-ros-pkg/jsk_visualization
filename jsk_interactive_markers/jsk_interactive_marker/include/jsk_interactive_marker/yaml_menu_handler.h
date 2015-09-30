#ifndef __YAML_MENU_HANDLER_H__
#define __YAML_MENU_HANDLER_H__

#include <vector>
#include <yaml-cpp/yaml.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_srvs/Empty.h>
namespace jsk_interactive_marker {
  class YamlMenuHandler {
   public:
    ros::NodeHandle* _node_ptr;
    interactive_markers::MenuHandler _menu_handler;
    YamlMenuHandler(ros::NodeHandle* node_ptr, std::string file_name);
    bool initMenu(std::string file);
    void callService(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string service_name);
    void applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name);
  };
};

#endif
