#include "yes_no_button_interface.hpp"
// #include <boost/thread.hpp>
// #include <thread>

#include <rviz_common/config.hpp>
//#include <ros/package.h>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QVBoxLayout>
//#include <jsk_gui_msgs/msg/yes_no.hpp>
#include <rviz_common/display_context.hpp>

namespace jsk_rviz_plugins
{

YesNoButtonInterface::YesNoButtonInterface(QWidget * parent) : rviz_common::Panel(parent)
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
  // ros::NodeHandle nh;
  // if (!ros::service::exists("/rviz/yes_no_button", /*print_failure_reason*/false)) {
  //   yes_no_button_service_ = nh.advertiseService(
  //     "/rviz/yes_no_button",
  //     &YesNoButtonInterface::requested,
  //     this);
  // }
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  if (true)  // if (!ros::service::exists("/rviz/yes_no_button", /*print_failure_reason*/false))
  {
    using namespace std::placeholders;
    yes_no_button_service_ = nh_->create_service<jsk_gui_msgs::srv::YesNo>(
      "/rviz/yes_no_button", std::bind(&YesNoButtonInterface::requested, this, _1, _2, _3));
  }
}

bool YesNoButtonInterface::requested(
  //     jsk_gui_msgs::srv::YesNo::Request& req,
  //     jsk_gui_msgs::srv::YesNo::Response& res)
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<jsk_gui_msgs::srv::YesNo::Request> req,
  const std::shared_ptr<jsk_gui_msgs::srv::YesNo::Response> res)
{
  (void)request_header;
  (void)req;

  need_user_input_ = true;
  yes_button_->setEnabled(true);
  no_button_->setEnabled(true);
  while (need_user_input_) {
    QApplication::processEvents(QEventLoop::AllEvents, 100);
  }

  yes_button_->setEnabled(false);
  no_button_->setEnabled(false);
  res->yes = yes_;
  return true;
}

void YesNoButtonInterface::respondYes()
{
  std::lock_guard<std::mutex> lock(mutex_);
  yes_ = true;
  need_user_input_ = false;
}

void YesNoButtonInterface::respondNo()
{
  std::lock_guard<std::mutex> lock(mutex_);
  yes_ = false;
  need_user_input_ = false;
}

void YesNoButtonInterface::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void YesNoButtonInterface::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::YesNoButtonInterface, rviz_common::Panel)
