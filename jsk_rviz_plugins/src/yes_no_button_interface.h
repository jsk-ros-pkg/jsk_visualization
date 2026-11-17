#ifndef YES_NO_BUTTON_INTERFACE_H
#define YES_NO_BUTTON_INTERFACE_H

#include "rviz_compat.h"
#ifndef Q_MOC_RUN
#if ROS_VERSION_MAJOR != 1
#include <rviz_common/panel.hpp>
#include <boost/thread/mutex.hpp>
#else
#include <ros/ros.h>
#include <rviz/panel.h>
#include <boost/thread.hpp>
#endif
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

#if ROS_VERSION_MAJOR != 1
#include <jsk_gui_msgs/srv/yes_no.hpp>
JSK_ROS1_ROS2_COMPAT_SRV_ALIAS(jsk_gui_msgs, YesNo)
#else
#include <jsk_gui_msgs/YesNo.h>
#endif


namespace jsk_rviz_plugins
{

  class YesNoButtonInterface: public rviz::Panel
  {
  Q_OBJECT
  public:
    YesNoButtonInterface(QWidget* parent = 0);

    virtual void onInitialize();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected Q_SLOTS:
    void respondYes();
    void respondNo();
  protected:
#if ROS_VERSION_MAJOR != 1
    // rviz_common spins its node in the GUI thread, which can not wait for
    // the buttons in the service callback as on ROS 1, so the response is
    // deferred until a button is pushed
    void requested(
      std::shared_ptr<rclcpp::Service<jsk_gui_msgs::YesNo>> service,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<jsk_gui_msgs::YesNo::Request> req);
    void respond();
    std::shared_ptr<rmw_request_id_t> request_header_;
#else
    virtual bool requested(
      jsk_gui_msgs::YesNo::Request& req,
      jsk_gui_msgs::YesNo::Response& res);
#endif
    QHBoxLayout* layout_;
    QPushButton* yes_button_;
    QPushButton* no_button_;
    bool yes_;
    bool need_user_input_;
    boost::mutex mutex_;
#if ROS_VERSION_MAJOR != 1
    rclcpp::Service<jsk_gui_msgs::YesNo>::SharedPtr yes_no_button_service_;
#else
    ros::ServiceServer yes_no_button_service_;
#endif
  };

}  // namespace jsk_rviz_plugins


#endif  // YES_NO_BUTTON_INTERFACE_H
