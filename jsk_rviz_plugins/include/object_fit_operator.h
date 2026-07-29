#ifndef OBJECT_FIT_OPERATOR_H
#define OBJECT_FIT_OPERATOR_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#include <QToolButton>
#include <QCheckBox>
#include <jsk_rviz_plugins/msg/object_fit_command.hpp>
#endif

class QLineEdit;
class QToolButton;

namespace jsk_rviz_plugins
{
  class ObjectFitOperatorAction: public rviz_common::Panel
    {
      Q_OBJECT
      public:
      ObjectFitOperatorAction( QWidget* parent = 0 );

      virtual void onInitialize() override;
      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

    protected Q_SLOTS:
      void commandFit();
      void commandNear();
      void commandOther();
      void checkBoxChanged(bool state);
      void publishObjectFitOder(int type);

    protected:
      QToolButton* fit_button_;
      QToolButton* near_button_;
      QToolButton* other_button_;
      QCheckBox* check_box_;

      QHBoxLayout* horizontal_layout1_;
      QHBoxLayout* horizontal_layout2_;
      QVBoxLayout* layout;
      bool reverse_;

      rclcpp::Node::SharedPtr node_;
      rclcpp::Publisher<jsk_rviz_plugins::msg::ObjectFitCommand>::SharedPtr pub_;
    };
}

#endif
