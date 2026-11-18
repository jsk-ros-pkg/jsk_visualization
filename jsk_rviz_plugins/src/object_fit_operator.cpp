#include <QLineEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QCheckBox>
#include <QLabel>
#include "object_fit_operator.h"
#if ROS_VERSION_MAJOR != 1
#include <ament_index_cpp/get_package_share_path.hpp>
#include <rviz_common/display_context.hpp>
#else
#include <ros/package.h>
#endif

using namespace rviz;
namespace jsk_rviz_plugins
{
  ObjectFitOperatorAction::ObjectFitOperatorAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    layout = new QVBoxLayout;
#if ROS_VERSION_MAJOR != 1
    setLayout( layout );
  }

  // The buttons read their icons from the parameters, and the node of rviz2
  // is only available from onInitialize(), so the ROS 2 constructor ends
  // above and onInitialize() builds the rest of the panel.
  void ObjectFitOperatorAction::onInitialize()
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
#endif

    horizontal_layout1_ = new QHBoxLayout();
    horizontal_layout2_ = new QHBoxLayout();

    //Button to send cancel topic
    std::string fit_button_name, reverse_fit_button_name, near_button_name, other_button_name;
#if ROS_VERSION_MAJOR != 1
    const std::string icon_dir =
      (ament_index_cpp::get_package_share_path("jsk_rviz_plugins") / "icons").string();
    auto icon_param = [this](const std::string& name, const std::string& default_value) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter(name, default_value);
      }
      return node_->get_parameter(name).as_string();
    };
    fit_button_name = icon_param("object_fit_icon", icon_dir + "/fit.jpg");
    near_button_name = icon_param("object_near_icon", icon_dir + "/near.jpg");
    other_button_name = icon_param("object_other_icon", icon_dir + "/other.jpg");
#else
    nh_.param<std::string>("/object_fit_icon", fit_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/fit.jpg"));
    nh_.param<std::string>("/object_near_icon", near_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/near.jpg"));
    nh_.param<std::string>("/object_other_icon", other_button_name, ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/other.jpg"));
#endif

    QSize iconSize(150, 150);
    fit_button_ = new QToolButton();
    fit_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    fit_button_->setIcon(QIcon(QPixmap(QString(fit_button_name.c_str()))));
    fit_button_->setText("Onto");
    fit_button_->setIconSize(iconSize);
    horizontal_layout1_->addWidget( fit_button_ );

    check_box_ = new QCheckBox(QString("Reverse"));
    horizontal_layout1_->addWidget(check_box_);

    near_button_ = new QToolButton();
    near_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    near_button_->setIcon(QIcon(QPixmap(QString(near_button_name.c_str()))));
    near_button_->setIconSize(iconSize);
    near_button_->setText("Parallel");
    horizontal_layout2_->addWidget( near_button_ );

    other_button_ = new QToolButton();
    other_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    other_button_->setIcon(QIcon(QPixmap(QString(other_button_name.c_str()))));
    other_button_->setText("Perpendicular");
    other_button_->setIconSize(iconSize);
    horizontal_layout2_->addWidget( other_button_ );

    layout->addLayout(horizontal_layout1_);
    layout->addLayout(horizontal_layout2_);
#if ROS_VERSION_MAJOR == 1
    setLayout( layout );
#endif

    connect( fit_button_, SIGNAL( clicked() ), this, SLOT( commandFit()));
    connect( check_box_, SIGNAL(clicked(bool)), this, SLOT(checkBoxChanged(bool)));
    connect( near_button_, SIGNAL( clicked() ), this, SLOT( commandNear()));
    connect( other_button_, SIGNAL( clicked() ), this, SLOT( commandOther()));

#if ROS_VERSION_MAJOR != 1
    pub_ = node_->create_publisher<jsk_rviz_plugins::ObjectFitCommand>("/object_fit_command", 1);
#else
    pub_ = nh_.advertise<jsk_rviz_plugins::ObjectFitCommand>( "/object_fit_command", 1 );
#endif
  }

  void ObjectFitOperatorAction::checkBoxChanged(bool state){
    reverse_ = state;
  }

  void ObjectFitOperatorAction::commandFit(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_FIT);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::FIT);
  }

  void ObjectFitOperatorAction::commandNear(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_NEAR);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::NEAR);
  }

  void ObjectFitOperatorAction::commandOther(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::REVERSE_OTHER);
    else
      publishObjectFitOder(jsk_rviz_plugins::ObjectFitCommand::OTHER);
  }

  void ObjectFitOperatorAction::publishObjectFitOder(int type){
    jsk_rviz_plugins::ObjectFitCommand msg;
    msg.command = type;
#if ROS_VERSION_MAJOR != 1
    pub_->publish(msg);
#else
    pub_.publish(msg);
#endif
  }

  void ObjectFitOperatorAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void ObjectFitOperatorAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#if ROS_VERSION_MAJOR != 1
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::ObjectFitOperatorAction, rviz::Panel )
