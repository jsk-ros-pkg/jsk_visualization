#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#else
#include "rviz/config.h"
#endif
#include "empty_service_call_interface.h"
#if ROS_VERSION_MAJOR == 1
#include <ros/package.h>
#endif
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

using namespace rviz;
namespace jsk_rviz_plugins
{
  EmptyServiceCallInterfaceAction::EmptyServiceCallInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
#if ROS_VERSION_MAJOR != 1
    // the parameters are read through the node of rviz, which the panel
    // only gets in onInitialize()
    layout = new QVBoxLayout();
    setLayout( layout );
  }

  void EmptyServiceCallInterfaceAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
#endif
    parseROSParameters();

    QHBoxLayout* h_layout = new QHBoxLayout;
    h_layout->setAlignment(Qt::AlignLeft);
#if ROS_VERSION_MAJOR == 1
    layout = new QVBoxLayout();
#endif
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
#if ROS_VERSION_MAJOR == 1
    setLayout( layout );
#endif
  }

  void EmptyServiceCallInterfaceAction::parseROSParameters(){
#if ROS_VERSION_MAJOR != 1
    // the parameters of the rviz2 node:
    //   icon_include_package: jsk_rviz_plugins
    //   rviz_service_call:
    //     buttons: [<id>, ...]
    //     <id>: {icon: ..., service_name: ..., text: ...}
    auto param = [this](const std::string& name, const auto& default_value) {
      using T = std::decay_t<decltype(default_value)>;
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<T>(name, default_value);
      }
      return node_->get_parameter(name).get_value<T>();
    };
    // a parameter of another type than expected (declare_parameter() and
    // get_value() throw) or a missing icon package leaves the panel empty
    // instead of taking rviz down
    try {
    std::string icon_package_name = param("icon_include_package", std::string("jsk_rviz_plugins"));
    ROS_INFO("Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    if(!icon_package_name.empty())
      icon_path_prefix = ament_index_cpp::get_package_share_directory(icon_package_name) + std::string("/icons/");

    std::vector<std::string> button_ids = param("rviz_service_call.buttons", std::vector<std::string>());
    if (button_ids.empty()) {
      ROS_WARN("rviz_service_call.buttons is not set, no buttons are shown");
    }
    for (const std::string& id : button_ids)
      {
        const std::string base = "rviz_service_call." + id + ".";
        ServiceCallButtonInfo new_button;
        new_button.icon_file_path = icon_path_prefix + param(base + "icon", std::string());
        new_button.service_name = param(base + "service_name", std::string());
        new_button.text = param(base + "text", std::string());
        service_call_button_infos_.push_back(new_button);
      }
    } catch (const std::exception& e) {
      ROS_WARN("Malformed rviz_service_call parameters, no buttons are shown: %s", e.what());
      service_call_button_infos_.clear();
    }
#else
    ros::NodeHandle nh("~");

    //icon file package file_name
    std::string icon_package_name;
    nh.param<std::string>("icon_include_package", icon_package_name, std::string("jsk_rviz_plugins"));
    ROS_INFO("Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    if(!icon_package_name.empty())
      icon_path_prefix = ros::package::getPath(icon_package_name) + std::string("/icons/");

    XmlRpc::XmlRpcValue buttons_list;
    nh.getParam("rviz_service_call/buttons", buttons_list);
    // XmlRpcValue throws on size() of a missing (invalid) value and on
    // operator[] or the casts of a value of another type, which aborted rviz
    if (buttons_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("~rviz_service_call/buttons is not set or is not a list, no buttons are shown");
      return;
    }
    for (int32_t i = 0; i < buttons_list.size(); ++i) {
      XmlRpc::XmlRpcValue& button = buttons_list[i];
      const char* keys[] = {"icon", "service_name", "text"};
      for (size_t k = 0; k < 3; k++) {
        if (button.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !button.hasMember(keys[k]) ||
            button[keys[k]].getType() != XmlRpc::XmlRpcValue::TypeString) {
          ROS_WARN("~rviz_service_call/buttons[%d] has no string %s, no buttons are shown", i, keys[k]);
          return;
        }
      }
    }
    for (int32_t i = 0; i < buttons_list.size(); ++i)
      {
        ServiceCallButtonInfo new_button;
        new_button.icon_file_path = icon_path_prefix + static_cast<std::string>(buttons_list[i]["icon"]);
        new_button.service_name = static_cast<std::string>(buttons_list[i]["service_name"]);
        new_button.text = static_cast<std::string>(buttons_list[i]["text"]);
        service_call_button_infos_.push_back(new_button);
      }
#endif
  };

  void EmptyServiceCallInterfaceAction::callRequestEmptyCommand(const int button_id){
#if ROS_VERSION_MAJOR != 1
    // rviz spins its node in the GUI thread, so do not block on the
    // response; it is logged once rviz processes it
    const std::string& service_name = service_call_button_infos_[button_id].service_name;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr& client = clients_[service_name];
    if (!client) {
      client = node_->create_client<std_srvs::srv::Empty>(service_name);
    }
    if (!client->service_is_ready()) {
      ROS_ERROR("Service call FAIL");
      return;
    }
    client->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        ROS_INFO("Call Success");
      });
#else
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(service_call_button_infos_[button_id].service_name, true);
    std_srvs::Empty srv;
    if(client.call(srv))
      ROS_INFO("Call Success");
    else{
      ROS_ERROR("Service call FAIL");
    };
#endif
  }

  void EmptyServiceCallInterfaceAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void EmptyServiceCallInterfaceAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::EmptyServiceCallInterfaceAction, rviz::Panel )
