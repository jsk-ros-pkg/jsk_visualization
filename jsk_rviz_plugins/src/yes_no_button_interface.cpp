#include "yes_no_button_interface.h"
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#else
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <ros/package.h>
#endif
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

#if ROS_VERSION_MAJOR == 1
#include <jsk_gui_msgs/YesNo.h>
#endif


namespace jsk_rviz_plugins
{

  YesNoButtonInterface::YesNoButtonInterface(QWidget* parent)
    : rviz::Panel(parent)
  {
    layout_ = new QHBoxLayout;

    yes_button_ = new QPushButton("Yes");
    layout_->addWidget(yes_button_);
    yes_button_->setEnabled(false);

    no_button_ = new QPushButton("No");
    layout_->addWidget(no_button_);
    no_button_->setEnabled(false);

    connect(yes_button_, SIGNAL(clicked()), this, SLOT(respondYes()));
    connect(no_button_, SIGNAL(clicked()), this, SLOT(respondNo()));

    setLayout(layout_);
  }

  void YesNoButtonInterface::onInitialize()
  {
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node =
      getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    if (!node->get_service_names_and_types().count("/rviz/yes_no_button")) {
      yes_no_button_service_ = node->create_service<jsk_gui_msgs::YesNo>(
        "/rviz/yes_no_button",
        std::bind(&YesNoButtonInterface::requested, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));
    }
  }

  void YesNoButtonInterface::requested(
      std::shared_ptr<rclcpp::Service<jsk_gui_msgs::YesNo>> /*service*/,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<jsk_gui_msgs::YesNo::Request> /*req*/)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (request_header_) {
      // a request is still waiting for the user: answer it with "no"
      yes_ = false;
      respond();
    }
    request_header_ = request_header;
    need_user_input_ = true;
    yes_button_->setEnabled(true);
    no_button_->setEnabled(true);
  }

  // called with mutex_ locked
  void YesNoButtonInterface::respond()
  {
    std::shared_ptr<rmw_request_id_t> request_header;
    request_header.swap(request_header_);
    yes_button_->setEnabled(false);
    no_button_->setEnabled(false);
    if (request_header && yes_no_button_service_) {
      jsk_gui_msgs::YesNo::Response res;
      res.yes = yes_;
      yes_no_button_service_->send_response(*request_header, res);
    }
  }
#else
    ros::NodeHandle nh;
    if (!ros::service::exists("/rviz/yes_no_button", /*print_failure_reason*/false)) {
      yes_no_button_service_ = nh.advertiseService(
        "/rviz/yes_no_button",
        &YesNoButtonInterface::requested,
        this);
    }
  }

  bool YesNoButtonInterface::requested(
      jsk_gui_msgs::YesNo::Request& req,
      jsk_gui_msgs::YesNo::Response& res)
  {
    need_user_input_ = true;
    yes_button_->setEnabled(true);
    no_button_->setEnabled(true);
    while (need_user_input_) {
      QApplication::processEvents(QEventLoop::AllEvents, 100);
    }
    yes_button_->setEnabled(false);
    no_button_->setEnabled(false);
    res.yes = yes_;
    return true;
  }
#endif

  void YesNoButtonInterface::respondYes()
  {
    boost::mutex::scoped_lock lock(mutex_);
    yes_ = true;
    need_user_input_ = false;
#if ROS_VERSION_MAJOR != 1
    respond();
#endif
  }

  void YesNoButtonInterface::respondNo()
  {
    boost::mutex::scoped_lock lock(mutex_);
    yes_ = false;
    need_user_input_ = false;
#if ROS_VERSION_MAJOR != 1
    respond();
#endif
  }

  void YesNoButtonInterface::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  void YesNoButtonInterface::load(const rviz::Config& config)
  {
    rviz::Panel::load(config);
  }

}  // namespace jsk_rviz_plugins


#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::YesNoButtonInterface, rviz::Panel)
