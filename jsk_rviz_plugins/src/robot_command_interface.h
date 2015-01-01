#ifndef ROBOT_COMMAND_INTERFACE_H
#define ROBOT_COMMAND_INTERFACE_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtGui>
#include <jsk_rviz_plugins/EusCommand.h>

namespace Ui
{
  class RobotCommandInterface;
}

namespace jsk_rviz_plugins
{
  class RobotCommandInterfaceAction: public rviz::Panel
  {
    Q_OBJECT
    public:
    RobotCommandInterfaceAction( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  protected Q_SLOTS:

    void callRequestResetPose();
    void callRequestManipPose();
    void callRequestInitPose();

    void callRequestResetGripperPose();
    void callRequestHookGrippePose();
    void callRequestGraspGrippePose();

    void callRequestStartABC();
    void callRequestStartST();
    void callRequestStartIMP();
    void callRequestStartIMPforDrill();

    void callRequestStopABC();
    void callRequestStopST();
    void callRequestStopIMP();

    void callRequestEusCommand(std::string command);

  protected:
    // The ROS node handle.
    ros::NodeHandle nh_;
    Ui::RobotCommandInterface *ui_;
  };

}

#endif // TELEOP_PANEL_H
