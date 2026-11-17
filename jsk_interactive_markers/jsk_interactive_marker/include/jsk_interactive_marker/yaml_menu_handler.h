#ifndef __YAML_MENU_HANDLER_H__
#define __YAML_MENU_HANDLER_H__

#include "jsk_interactive_marker/ros1_ros2_compat.h"
#include <vector>
#include <yaml-cpp/yaml.h>
#if ROS_VERSION_MAJOR != 1
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <std_msgs/msg/string.hpp>
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(visualization_msgs, InteractiveMarkerFeedback)
#else
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_msgs/String.h>
#endif
#include <map>

namespace jsk_interactive_marker {
  class YamlMenuHandler {
   public:
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr _node_ptr;
    interactive_markers::MenuHandler _menu_handler;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> _publisher_map;
    YamlMenuHandler(rclcpp::Node::SharedPtr node_ptr, std::string file_name);
#else
    ros::NodeHandle* _node_ptr;
    interactive_markers::MenuHandler _menu_handler;
    std::map<std::string, ros::Publisher> _publisher_map;
    YamlMenuHandler(ros::NodeHandle* node_ptr, std::string file_name);
#endif
    bool initMenu(std::string file);
    void pubTopic(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string topic_name);
    void applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name);
  };
};

#endif
