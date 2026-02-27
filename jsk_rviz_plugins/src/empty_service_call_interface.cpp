#include "empty_service_call_interface.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

namespace jsk_rviz_plugins
{
  EmptyServiceCallInterfaceAction::EmptyServiceCallInterfaceAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout();
    setLayout( layout );
  }

  void EmptyServiceCallInterfaceAction::onInitialize()
  {
    plugin_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    parseROSParameters();

    QHBoxLayout* h_layout = new QHBoxLayout;
    h_layout->setAlignment(Qt::AlignLeft);
    signal_mapper = new QSignalMapper(this);

    for(size_t i = 0; i < service_call_button_infos_.size();i++){
      ServiceCallButtonInfo target_button = service_call_button_infos_[i];
      QToolButton* tbutton = new QToolButton(this);
      tbutton->setText(target_button.text.c_str());
      tbutton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      tbutton->setIconSize(QSize(100, 100));
      tbutton->setIcon(QIcon(QPixmap(QString(target_button.icon_file_path.c_str()))));
      connect(tbutton, SIGNAL(clicked()), signal_mapper, SLOT(map()));
      signal_mapper->setMapping(tbutton, i);
      h_layout->addWidget(tbutton);
    };
    connect(signal_mapper, SIGNAL(mapped(int)),
            this, SLOT(callRequestEmptyCommand(int)));
    layout->addLayout(h_layout);
  }

  void EmptyServiceCallInterfaceAction::parseROSParameters(){    
    //icon file package file_name
    std::string icon_package_name;
    plugin_node_->declare_parameter("icon_include_package", "jsk_rviz_plugins");
    plugin_node_->get_parameter("icon_include_package", icon_package_name);
    RCLCPP_INFO(plugin_node_->get_logger(), "Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    if(!icon_package_name.empty())
      icon_path_prefix = ament_index_cpp::get_package_share_directory(icon_package_name) + std::string("/icons/");

     std::vector<std::string> button_ids;

     plugin_node_->declare_parameter<std::vector<std::string>>(
         "rviz_service_call.buttons", std::vector<std::string>{});

     plugin_node_->get_parameter("rviz_service_call.buttons", button_ids);

     if (button_ids.empty()) {
       RCLCPP_WARN(plugin_node_->get_logger(),
                   "Parameter 'rviz_service_call.buttons' is empty. No buttons "
                   "will be created.");
       return;
     }

     // --- read each button fields ---
     service_call_button_infos_.clear();
     service_call_button_infos_.reserve(button_ids.size());

     for (const auto &id : button_ids) {
       const std::string base = std::string("rviz_service_call.") + id + ".";

       // declare with empty defaults so missing values don't throw
       plugin_node_->declare_parameter<std::string>(base + "icon", "");
       plugin_node_->declare_parameter<std::string>(base + "service_name", "");
       plugin_node_->declare_parameter<std::string>(base + "text", "");

       std::string icon, service_name, text;
       plugin_node_->get_parameter(base + "icon", icon);
       plugin_node_->get_parameter(base + "service_name", service_name);
       plugin_node_->get_parameter(base + "text", text);

       if (service_name.empty()) {
         RCLCPP_WARN(
             plugin_node_->get_logger(),
             "Button '%s' has empty service_name (%sservice_name). Skip.",
             id.c_str(), base.c_str());
         continue;
       }

       ServiceCallButtonInfo new_button;
       new_button.icon_file_path =
           icon.empty() ? "" : (icon_path_prefix + icon);
       new_button.service_name = service_name;
       new_button.text = text.empty() ? id : text;

       service_call_button_infos_.push_back(new_button);

       RCLCPP_INFO(plugin_node_->get_logger(),
                   "Loaded button id='%s' icon='%s' service='%s' text='%s'",
                   id.c_str(), icon.c_str(), service_name.c_str(),
                   new_button.text.c_str());
     }
  };

  void EmptyServiceCallInterfaceAction::callRequestEmptyCommand(const int button_id){
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
      plugin_node_->create_client<std_srvs::srv::Empty>(service_call_button_infos_[button_id].service_name);
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(plugin_node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(plugin_node_->get_logger(), "Call Success");
    } else{
      RCLCPP_ERROR(plugin_node_->get_logger(), "Service call FAIL");
    }
  }

  void EmptyServiceCallInterfaceAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
  }

  void EmptyServiceCallInterfaceAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::EmptyServiceCallInterfaceAction, rviz_common::Panel )
