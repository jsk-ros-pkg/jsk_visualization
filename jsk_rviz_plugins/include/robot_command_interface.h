#ifndef ROBOT_COMMAND_INTERFACE_H
#define ROBOT_COMMAND_INTERFACE_H

#ifndef Q_MOC_RUN
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#include <jsk_rviz_plugins/srv/eus_command.hpp>
#include <resource_retriever/retriever.hpp>
#endif

namespace jsk_rviz_plugins
{
  class RobotCommandInterfaceAction: public rviz_common::Panel
  {
    Q_OBJECT
    public:
    RobotCommandInterfaceAction( QWidget* parent = 0 );

    virtual void onInitialize() override;

  protected Q_SLOTS:
    bool callRequestEusCommand(const std::string& command);
    void buttonCallback(int i);
  protected:
    void popupDialog(const std::string& text);
    void parseROSParameters();
    rclcpp::Node::SharedPtr node_;
    QHBoxLayout* layout_;
    QSignalMapper* signal_mapper_;
    std::map<int, std::string> euscommand_mapping_;
    std::map<int, std::string> emptyservice_mapping_;
  };

}

#endif // ROBOT_COMMAND_INTERFACE_H
