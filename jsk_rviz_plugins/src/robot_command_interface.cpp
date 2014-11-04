#include <stdio.h>

#include "rviz/config.h"
#include "robot_command_interface.h"
#include "ros/time.h"
#include <ros/package.h>

#include "ui_robot_command_interface.h"

using namespace rviz;

namespace jsk_rviz_plugin
{

  RobotCommandInterfaceAction::RobotCommandInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    ui_ = new Ui::RobotCommandInterface();
    ui_->setupUi(this);
    ui_->reset_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->reset_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/reset-pose.jpg")).c_str()))));
    ui_->reset_manip_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->reset_manip_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/reset-manip-pose.jpg")).c_str()))));
    ui_->init_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->init_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/init-pose.jpg")).c_str()))));
    ui_->hand_reset_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_reset_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/hand-reset-pose.jpg")).c_str()))));
    ui_->hand_hook_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_hook_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/hand-hook-pose.jpg")).c_str()))));
    ui_->hand_grasp_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_grasp_pose_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/hand-grasp-pose.jpg")).c_str()))));
    ui_->hrpsys_start_abc_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_abc_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/start-abc.png")).c_str()))));
    ui_->hrpsys_start_st_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_st_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/start-st.png")).c_str()))));
    ui_->hrpsys_start_imp_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_imp_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/start-imp.png")).c_str()))));
    ui_->hrpsys_stop_abc_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_abc_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/stop-abc.png")).c_str()))));
    ui_->hrpsys_stop_st_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_st_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/stop-st.png")).c_str()))));
    ui_->hrpsys_stop_imp_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_imp_button->setIcon(QIcon(QPixmap(QString((ros::package::getPath("jsk_rviz_plugins")+std::string("/icons/stop-imp.png")).c_str()))));

    connect( ui_->reset_pose_button, SIGNAL( clicked() ), this, SLOT( callRequestResetPose()));
    connect( ui_->reset_manip_pose_button, SIGNAL( clicked() ), this, SLOT( callRequestManipPose()));
    connect( ui_->init_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestInitPose()));

    connect( ui_->hand_reset_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestResetGripperPose()));
    connect( ui_->hand_hook_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestHookGrippePose()));
    connect( ui_->hand_grasp_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestGraspGrippePose()));

    connect( ui_->hrpsys_start_abc_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartABC()));
    connect( ui_->hrpsys_start_st_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartST()));
    connect( ui_->hrpsys_start_imp_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartIMP()));

    connect( ui_->hrpsys_stop_abc_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopABC ()));
    connect( ui_->hrpsys_stop_st_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopST()));
    connect( ui_->hrpsys_stop_imp_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopIMP()));
  }

  void RobotCommandInterfaceAction::callRequestResetPose(){
    std::string command("(send *ri* :angle-vector (send *robot* :reset-pose) 5000))");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestManipPose(){
    std::string command("(send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestInitPose(){
    std::string command("(send *ri* :angle-vector (send *robot* :init-pose) 5000)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestResetGripperPose(){
    std::string command("(progn (send *robot* :hand :arms :reset-pose) (send *ri* :hand-angle-vector (apply #\"concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestHookGrippePose(){
    std::string command("(progn (send *robot* :hand :arms :hook-pose) (send *ri* :hand-angle-vector (apply #\"concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestGraspGrippePose(){
    std::string command("(progn (send *robot* :hand :arms :grasp-pose) (send *ri* :hand-angle-vector (apply #\"concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStartABC(){
    std::string command("(send *ri* :start-auto-balancer)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStartST(){
    std::string command("(send *ri* :start-st)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStartIMP(){
    std::string command("(send *ri* :start-impedance :arms)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStopABC(){
    std::string command("(send *ri* :stop-auto-balancer)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStopST(){
    std::string command("(send *ri* :stop-st)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestStopIMP(){
    std::string command("(send *ri* :stop-impedance :arms)");
    callRequestEusCommand(command);
  };

  void RobotCommandInterfaceAction::callRequestEusCommand(std::string command){
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::EusCommand>("/eus_command", true);
    jsk_rviz_plugins::EusCommand srv;
    srv.request.command = command;
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  }

  void RobotCommandInterfaceAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void RobotCommandInterfaceAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugin::RobotCommandInterfaceAction, rviz::Panel )
