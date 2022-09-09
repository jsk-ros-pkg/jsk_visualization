#ifndef OBJECT_FIT_OPERATOR_H
#define OBJECT_FIT_OPERATOR_H

#ifndef Q_MOC_RUN
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
// #include <rviz/panel.h>
#include <QCheckBox>
#include <QToolButton>
#include <QtGlobal>
#include <QtWidgets>
#include <jsk_rviz_plugin_msgs/msg/object_fit_command.hpp>
#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QToolButton;

namespace jsk_rviz_plugins
{
class ObjectFitOperatorAction : public rviz_common::Panel
{
  Q_OBJECT
public:
  ObjectFitOperatorAction(QWidget * parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:
  void commandFit();
  void commandNear();
  void commandOther();
  void checkBoxChanged(bool state);
  void publishObjectFitOder(int type);

protected:
  QToolButton * fit_button_;
  QToolButton * near_button_;
  QToolButton * other_button_;
  QCheckBox * check_box_;

  QHBoxLayout * horizontal_layout1_;
  QHBoxLayout * horizontal_layout2_;
  QVBoxLayout * layout;
  bool reverse_;

  // ros::NodeHandle nh_;
  // ros::Publisher pub_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<jsk_rviz_plugin_msgs::msg::ObjectFitCommand>::SharedPtr pub_;
};
}  // namespace jsk_rviz_plugins

#endif
