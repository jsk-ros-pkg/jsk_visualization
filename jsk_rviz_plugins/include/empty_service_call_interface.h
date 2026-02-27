#ifndef EMPTY_SERCICE_CALL_INTERFACE_H
#define EMPTY_SERCICE_CALL_INTERFACE_H

#ifndef Q_MOC_RUN
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#endif

namespace jsk_rviz_plugins
{
  struct ServiceCallButtonInfo
  {
    std::string icon_file_path;
    std::string service_name;
    std::string text;
  };

  class EmptyServiceCallInterfaceAction: public rviz_common::Panel
  {
    Q_OBJECT
    public:
    EmptyServiceCallInterfaceAction( QWidget* parent = 0 );

    virtual void onInitialize() override;
    virtual void load( const rviz_common::Config& config );
    virtual void save( rviz_common::Config config ) const;

  protected Q_SLOTS:
    void callRequestEmptyCommand(int button_id);
    void parseROSParameters();
  protected:
    rclcpp::Node::SharedPtr plugin_node_;
    std::vector<ServiceCallButtonInfo> service_call_button_infos_;
    QVBoxLayout* layout;
    QHBoxLayout* h_layout;
    QSignalMapper* signal_mapper;
  };
}

#endif
