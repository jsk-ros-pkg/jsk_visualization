#ifndef JSK_YES_NO_BUTTON_INTERFACE_HPP
#define JSK_YES_NO_BUTTON_INTERFACE_HPP

#ifndef Q_MOC_RUN
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

// #include <rviz/panel.h>
#include <rviz_common/panel.hpp>

// #include <boost/thread.hpp>
// #include <thread>
#include <mutex>

// #if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
// #else
// #  include <QtGui>
// #endif
#endif

#include <jsk_gui_msgs/srv/yes_no.hpp>

namespace jsk_rviz_plugins
{

  class YesNoButtonInterface: public rviz_common::Panel
  {
  Q_OBJECT
  public:
    YesNoButtonInterface(QWidget* parent = 0);

    virtual void onInitialize();
    virtual void load(const rviz_common::Config& config);
    virtual void save(rviz_common::Config config) const;

  protected Q_SLOTS:
    void respondYes();
    void respondNo();
  protected:
    virtual bool requested(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<jsk_gui_msgs::srv::YesNo::Request> req,
      const std::shared_ptr<jsk_gui_msgs::srv::YesNo::Response> res);
      // jsk_gui_msgs::srv::YesNo::Request& req,
      // jsk_gui_msgs::srv::YesNo::Response& res);
    
    QHBoxLayout* layout_;
    QPushButton* yes_button_;
    QPushButton* no_button_;
    bool yes_;
    bool need_user_input_;
    std::mutex mutex_;
    
    //ros::ServiceServer yes_no_button_service_;
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Service<jsk_gui_msgs::srv::YesNo>::SharedPtr yes_no_button_service_;
  };

}  // namespace jsk_rviz_plugins


#endif  // YES_NO_BUTTON_INTERFACE_H
