#include <stdio.h>

#include <exception>
#include <string>
#include <vector>

#include <QSignalMapper>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/config.hpp>
#include <std_srvs/srv/empty.hpp>

#include "robot_command_interface.h"

namespace jsk_rviz_plugins
{

  // Exception class
  class RobotCommandParseException: public std::runtime_error
  {
  public:
    RobotCommandParseException(const std::string& text): std::runtime_error(text) {}
  };

  RobotCommandInterfaceAction::RobotCommandInterfaceAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout_ = new QHBoxLayout();
    setLayout( layout_ );
  }

  void RobotCommandInterfaceAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    signal_mapper_ = new QSignalMapper(this);
    parseROSParameters();
    layout_->addStretch();
    // QSignalMapper::mapped() was removed in Qt6, mappedInt() replaces it
    connect(signal_mapper_, &QSignalMapper::mappedInt,
            this, &RobotCommandInterfaceAction::buttonCallback);
  }

  // ROS 2 parameters cannot hold an array of structs, so the buttons are described
  // as a list of ids in `robot_command_buttons` plus one parameter group per id:
  //   robot_command_buttons: ["reset_pose", ...]
  //   robot_command.reset_pose.name: "Reset Pose"
  //   robot_command.reset_pose.icon: "package://jsk_rviz_plugins/icons/reset-pose.jpg"
  //   robot_command.reset_pose.type: "euscommand"  # or "emptysrv"
  //   robot_command.reset_pose.command: "(send *ri* :angle-vector ...)"
  //   robot_command.reset_pose.srv: "/my_service"  # for type: emptysrv
  void RobotCommandInterfaceAction::parseROSParameters()
  {
    std::vector<std::string> button_ids;
    node_->declare_parameter<std::vector<std::string>>(
      "robot_command_buttons", std::vector<std::string>{});
    node_->get_parameter("robot_command_buttons", button_ids);

    if (button_ids.empty()) {
      popupDialog("You need to specify ~robot_command_buttons parameter.\n"
                  "See package://jsk_rviz_plugins/config/default_robot_command.yaml");
      return;
    }

    try {
      for (size_t i = 0; i < button_ids.size(); i++) {
        const std::string base = std::string("robot_command.") + button_ids[i] + ".";
        node_->declare_parameter<std::string>(base + "name", "");
        node_->declare_parameter<std::string>(base + "icon", "");
        node_->declare_parameter<std::string>(base + "type", "");
        node_->declare_parameter<std::string>(base + "command", "");
        node_->declare_parameter<std::string>(base + "srv", "");

        std::string name = node_->get_parameter(base + "name").as_string();
        std::string icon = node_->get_parameter(base + "icon").as_string();
        std::string type = node_->get_parameter(base + "type").as_string();
        std::string command = node_->get_parameter(base + "command").as_string();
        std::string srv = node_->get_parameter(base + "srv").as_string();

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
            icon = ament_index_cpp::get_package_share_directory(package) + icon;
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
        layout_->addWidget(button);
      }
    }
    catch (RobotCommandParseException& e) {
      popupDialog(std::string("Malformed ~robot_command_buttons parameter.\n")
                  + e.what() + "\n"
                  "See package://jsk_rviz_plugins/config/default_robot_command.yaml");
    }
  }

  bool RobotCommandInterfaceAction::callRequestEusCommand(const std::string& command){
    auto client = node_->create_client<jsk_rviz_plugins::srv::EusCommand>("/eus_command");
    auto request = std::make_shared<jsk_rviz_plugins::srv::EusCommand::Request>();
    request->command = command;
    auto result = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS;
  }

  void RobotCommandInterfaceAction::buttonCallback(int i)
  {
    RCLCPP_INFO(node_->get_logger(), "buttonCallback(%d)", i);
    if (euscommand_mapping_.find(i) != euscommand_mapping_.end()) {
      if(!callRequestEusCommand(euscommand_mapping_[i])) {
        popupDialog("Failed to call " + euscommand_mapping_[i]);
      }
    }
    else if (emptyservice_mapping_.find(i) != emptyservice_mapping_.end()) {
      auto client = node_->create_client<std_srvs::srv::Empty>(emptyservice_mapping_[i]);
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        popupDialog("Failed to call " + emptyservice_mapping_[i]);
      }
    }
    else {
      popupDialog("Failed to find corresponding command for "
                  + std::to_string(i));
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

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RobotCommandInterfaceAction, rviz_common::Panel )
