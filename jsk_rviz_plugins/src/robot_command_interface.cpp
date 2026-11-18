#include <stdio.h>

#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include "robot_command_interface.h"
#include <chrono>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_path.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <std_srvs/srv/empty.hpp>
#include <exception>
#else
#include "rviz/config.h"
#include "robot_command_interface.h"
#include "ros/time.h"
#include <ros/package.h>
#include <boost/format.hpp>
#include <exception>
#include <std_srvs/Empty.h>
#endif

namespace jsk_rviz_plugins
{

  // Exception class
  class RobotCommandParseException: public std::runtime_error
  {
  public:
    RobotCommandParseException(const std::string& text): std::runtime_error(text) {}
  };

  RobotCommandInterfaceAction::RobotCommandInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
#if ROS_VERSION_MAJOR != 1
    setLayout(new QHBoxLayout());
  }

  // The buttons are made from the parameters, and the node of rviz2 is only
  // available from onInitialize(), so the ROS 2 constructor ends above and
  // onInitialize() builds the rest of the panel.
  void RobotCommandInterfaceAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    signal_mapper_ = new QSignalMapper(this);
    QHBoxLayout* layout = static_cast<QHBoxLayout*>(this->layout());
    // ROS 2 parameters cannot hold an array of structs, so the buttons are
    // described as a list of ids in `robot_command_buttons` plus one
    // parameter group per id:
    //   robot_command_buttons: ["reset_pose", ...]
    //   robot_command.reset_pose.name: "Reset Pose"
    //   robot_command.reset_pose.icon: "package://jsk_rviz_plugins/icons/reset-pose.jpg"
    //   robot_command.reset_pose.type: "euscommand"  # or "emptysrv"
    //   robot_command.reset_pose.command: "(send *ri* :angle-vector ...)"
    //   robot_command.reset_pose.srv: "/my_service"  # for type: emptysrv
    auto param = [this](const std::string& name, const auto& default_value) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter(name, default_value);
      }
      return node_->get_parameter(name);
    };
    const std::vector<std::string> button_ids =
      param("robot_command_buttons", std::vector<std::string>{}).as_string_array();
    if (!button_ids.empty()) {
      try {
        for (size_t i = 0; i < button_ids.size(); i++) {
          const std::string base = "robot_command." + button_ids[i] + ".";
          const std::string name = param(base + "name", std::string()).as_string();
          std::string icon = param(base + "icon", std::string()).as_string();
          const std::string type = param(base + "type", std::string()).as_string();
          const std::string command = param(base + "command", std::string()).as_string();
          const std::string srv = param(base + "srv", std::string()).as_string();
          QToolButton* button = new QToolButton();
          button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
          if (name.empty()) {
            throw RobotCommandParseException("element of ~robot_comamnd_buttons should have name field");
          }
          button->setText(QString(name.c_str()));
          if (!icon.empty()) {
            if (icon.find("package://") == 0) {
              icon.erase(0, strlen("package://"));
              size_t package_end = icon.find("/");
              std::string package = icon.substr(0, package_end);
              icon.erase(0, package_end);
              icon = ament_index_cpp::get_package_share_path(package).string() + icon;
            }
            button->setIcon(QIcon(QPixmap(QString(icon.c_str()))));
            button->setIconSize(QSize(80, 80));
            button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
          }
          if (type == "euscommand") {
            if (command.empty()) {
              throw RobotCommandParseException("type: euscommand requires command field");
            }
            euscommand_mapping_[i] = command;
            button->setToolTip(euscommand_mapping_[i].c_str());
          }
          else if (type == "emptysrv") {
            if (srv.empty()) {
              throw RobotCommandParseException("type: emptysrv requires srv field");
            }
            emptyservice_mapping_[i] = srv;
            button->setToolTip(emptyservice_mapping_[i].c_str());
          }
          else {
            throw RobotCommandParseException("type field is required");
          }
          // connect
          connect(button, SIGNAL(clicked()), signal_mapper_, SLOT(map()));
          signal_mapper_->setMapping(button, i);
          layout->addWidget(button);
        }
      }
      catch (RobotCommandParseException& e) {
        popupDialog(std::string("Malformed ~robot_command_buttons parameter.\n")
                    + e.what() + "\n"
                    "See package://jsk_rviz_plugins/config/default_robot_command.yaml");
      }
    }
    else {
      popupDialog("You need to specify ~robot_command_buttons parameter.\n"
                  "See package://jsk_rviz_plugins/launch/robot_command_interface_sample.launch.py");
    }
#else
    resource_retriever::Retriever r;
    signal_mapper_ = new QSignalMapper(this);
    ros::NodeHandle nh("~");
    QHBoxLayout* layout = new QHBoxLayout();
    // Parse yaml file from parameter
    if (nh.hasParam("robot_command_buttons")) {
      try {
        XmlRpc::XmlRpcValue robot_command_buttons_xmlrpc;
        nh.param("robot_command_buttons", robot_command_buttons_xmlrpc, robot_command_buttons_xmlrpc);
        if (robot_command_buttons_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          throw RobotCommandParseException("~robot_comamnd_buttons should be an array");
        }
        else {
          for (size_t i = 0; i < robot_command_buttons_xmlrpc.size(); i++) {
            XmlRpc::XmlRpcValue button_xmlrpc = robot_command_buttons_xmlrpc[i];
            if (button_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              throw RobotCommandParseException("element of ~robot_comamnd_buttons should be an struct");
            }
            else {
              std::string name;
              QToolButton* button = new QToolButton();
              //button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
              button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

              if (button_xmlrpc.hasMember("name")) {
                name = (std::string)button_xmlrpc["name"];
              }
              else {
                throw RobotCommandParseException("element of ~robot_comamnd_buttons should have name field");
              }
              button->setText(QString(name.c_str()));
              if (button_xmlrpc.hasMember("icon")) {
                // TODO: resolve path
                std::string icon;
                icon = (std::string)button_xmlrpc["icon"];
                if (icon.find("package://") == 0) {
                  icon.erase(0, strlen("package://"));
                  size_t package_end = icon.find("/");
                  std::string package = icon.substr(0, package_end);
                  icon.erase(0, package_end);
                  std::string package_path;
                  package_path = ros::package::getPath(package);
                  icon = package_path + icon;
                }
                button->setIcon(QIcon(QPixmap(QString(icon.c_str()))));
                button->setIconSize(QSize(80, 80));
                button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
              }
              std::string type;
              if (button_xmlrpc.hasMember("type")) {
                type = (std::string)button_xmlrpc["type"];
              }
              if (type == "euscommand") {
                if (button_xmlrpc.hasMember("command")) {
                  euscommand_mapping_[i] = (std::string)button_xmlrpc["command"];
                  button->setToolTip(euscommand_mapping_[i].c_str());
                }
                else {
                  throw RobotCommandParseException("type: euscommand requires command field");
                }
              }
              else if (type == "emptysrv") {
                if (button_xmlrpc.hasMember("srv")) {
                  emptyservice_mapping_[i] = (std::string)button_xmlrpc["srv"];
                  button->setToolTip(emptyservice_mapping_[i].c_str());
                }
                else {
                  throw RobotCommandParseException("type: emptysrv requires srv field");
                }
              }
              else {
                throw RobotCommandParseException("type field is required");
              }
              // connect
              connect(button, SIGNAL(clicked()), signal_mapper_, SLOT(map()));
              signal_mapper_->setMapping(button, i);
              layout->addWidget(button);
            }
          }
        }
      }
      catch (RobotCommandParseException& e) {
        popupDialog((boost::format("Malformed ~robot_command_buttons parameter.\n"
                                  "%s\n"
                                  "See package://jsk_rviz_plugins/config/default_robot_command.yaml")
                     % e.what()).str().c_str());
      }
    }
    else {
      popupDialog("You need to specify ~robot_command_buttons parameter.\n"
                  "See package://jsk_rviz_plugins/launch/robot_command_interface_sample.launch");
    }
#endif
    layout->addStretch();
    connect(signal_mapper_, SIGNAL(mapped(int)), this, SLOT(buttonCallback(int)));
    // QToolButton* button = new QToolButton();
    
    // // button->setPopupMode(QToolButton::MenuButtonPopup);
    // button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    // button->setIcon(QIcon(QPixmap(QString("/home/garaemon/ros/hydro/src/jsk-ros-pkg/jsk_visualization/jsk_rviz_plugins/icons/stop-imp.png"))));
    
    // button->setText("Hello worpld");
    // layout->addWidget(button);
#if ROS_VERSION_MAJOR == 1
    this->setLayout(layout);
#endif
  }

#if ROS_VERSION_MAJOR != 1
  // Call an service of rviz2's node and wait for its response, failing as
  // ros::service::call() does when the service is not there.
  template<class ServiceT>
  static bool callService(const rclcpp::Node::SharedPtr& node, const std::string& name,
                          const typename ServiceT::Request::SharedPtr& request)
  {
    auto client = node->create_client<ServiceT>(name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      return false;
    }
    auto result = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS;
  }
#endif

  bool RobotCommandInterfaceAction::callRequestEusCommand(const std::string& command){
#if ROS_VERSION_MAJOR != 1
    auto request = std::make_shared<jsk_rviz_plugins::srv::EusCommand::Request>();
    request->command = command;
    return callService<jsk_rviz_plugins::srv::EusCommand>(node_, "/eus_command", request);
#else
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::EusCommand>("/eus_command", true);
    jsk_rviz_plugins::EusCommand srv;
    srv.request.command = command;
    return client.call(srv);
#endif
  }

  void RobotCommandInterfaceAction::buttonCallback(int i)
  {
    ROS_INFO("buttonCallback(%d)", i);
    if (euscommand_mapping_.find(i) != euscommand_mapping_.end()) {
      if(!callRequestEusCommand(euscommand_mapping_[i])) {
#if ROS_VERSION_MAJOR != 1
        popupDialog("Failed to call " + euscommand_mapping_[i]);
#else
        popupDialog((boost::format("Failed to call %s") % euscommand_mapping_[i]).str().c_str());
#endif
      }
    }
    else if (emptyservice_mapping_.find(i) != emptyservice_mapping_.end()) {
#if ROS_VERSION_MAJOR != 1
      if (!callService<std_srvs::srv::Empty>(
            node_, emptyservice_mapping_[i], std::make_shared<std_srvs::srv::Empty::Request>())) {
        popupDialog("Failed to call " + emptyservice_mapping_[i]);
      }
#else
      std_srvs::Empty emp;
      if (!ros::service::call(emptyservice_mapping_[i], emp)) {
        popupDialog((boost::format("Failed to call %s") % emptyservice_mapping_[i]).str().c_str());
      }
#endif
    }
    else {
#if ROS_VERSION_MAJOR != 1
      popupDialog("Failed to find corresponding command for " + std::to_string(i));
#else
      popupDialog((boost::format("Failed to find corresponding command for %d") % i).str().c_str());
#endif
    }
  }

  void RobotCommandInterfaceAction::popupDialog(const std::string& text)
  {
    QMessageBox msg_box;
    msg_box.setText("Unexpected error");
    msg_box.setText(QString(text.c_str()));
    msg_box.exec();
  }
}

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RobotCommandInterfaceAction, rviz::Panel )
