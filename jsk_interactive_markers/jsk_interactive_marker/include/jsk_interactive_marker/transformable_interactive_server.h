#ifndef __TRANSFORMABLE_INTERACTIVE_SERVER_H__
#define __TRANSFORMABLE_INTERACTIVE_SERVER_H__

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/transformable_object.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <jsk_rviz_plugins/OverlayText.h>
#include <iostream>
#include <sstream>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_interactive_marker/InteractiveSettingConfig.h>
#include <jsk_interactive_marker/GetTransformableMarkerPose.h>
#include <jsk_interactive_marker/SetTransformableMarkerPose.h>
#include <jsk_interactive_marker/GetTransformableMarkerColor.h>
#include <jsk_interactive_marker/SetTransformableMarkerColor.h>
#include <jsk_interactive_marker/GetTransformableMarkerFocus.h>
#include <jsk_interactive_marker/SetTransformableMarkerFocus.h>
#include <jsk_interactive_marker/GetMarkerDimensions.h>
#include <jsk_interactive_marker/SetMarkerDimensions.h>
#include <jsk_interactive_marker/GetType.h>
#include <jsk_interactive_marker/GetTransformableMarkerExistence.h>
#include <jsk_interactive_marker/MarkerDimensions.h>

using namespace std;

namespace jsk_interactive_marker
{
  class TransformableInteractiveServer{
  public:
    TransformableInteractiveServer();
    ~TransformableInteractiveServer();

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void setRadius(std_msgs::Float32 msg);
    void setSmallRadius(std_msgs::Float32 msg);
    void setX(std_msgs::Float32 msg);
    void setY(std_msgs::Float32 msg);
    void setZ(std_msgs::Float32 msg);

    void setPose(geometry_msgs::PoseStamped msg);
    void addPose(geometry_msgs::Pose msg);
    void addPoseRelative(geometry_msgs::Pose msg);

    void setColor(std_msgs::ColorRGBA msg);

    void insertNewBox( std::string frame_id, std::string name, std::string description );
    void insertNewCylinder( std::string frame_id, std::string name, std::string description );
    void insertNewTorus( std::string frame_id, std::string name, std::string description );

    void insertNewObject(TransformableObject* tobject, std::string name);
    void eraseObject(std::string name);
    void eraseAllObject();
    void eraseFocusObject();

    void run();
    void focusTextPublish();
    void focusPosePublish();

    void updateTransformableObject(TransformableObject* tobject);

    bool getPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req,jsk_interactive_marker::GetTransformableMarkerPose::Response &res);
    bool setPoseService(jsk_interactive_marker::SetTransformableMarkerPose::Request &req,jsk_interactive_marker::SetTransformableMarkerPose::Response &res);
    bool getColorService(jsk_interactive_marker::GetTransformableMarkerColor::Request &req,jsk_interactive_marker::GetTransformableMarkerColor::Response &res);
    bool setColorService(jsk_interactive_marker::SetTransformableMarkerColor::Request &req,jsk_interactive_marker::SetTransformableMarkerColor::Response &res);
    bool getFocusService(jsk_interactive_marker::GetTransformableMarkerFocus::Request &req,jsk_interactive_marker::GetTransformableMarkerFocus::Response &res);
    bool setFocusService(jsk_interactive_marker::SetTransformableMarkerFocus::Request &req,jsk_interactive_marker::SetTransformableMarkerFocus::Response &res);
    bool getTypeService(jsk_interactive_marker::GetType::Request &req,jsk_interactive_marker::GetType::Response &res);
    bool getExistenceService(jsk_interactive_marker::GetTransformableMarkerExistence::Request &req,jsk_interactive_marker::GetTransformableMarkerExistence::Response &res);
    bool setDimensionsService(jsk_interactive_marker::SetMarkerDimensions::Request &req,jsk_interactive_marker::SetMarkerDimensions::Response &res);
    bool getDimensionsService(jsk_interactive_marker::GetMarkerDimensions::Request &req,jsk_interactive_marker::GetMarkerDimensions::Response &res);
    void publishMarkerDimensions();

    bool requestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate::Request &req,jsk_rviz_plugins::RequestMarkerOperate::Response &res);
    virtual void configCallback(InteractiveSettingConfig &config, uint32_t level);
    void SetInitialInteractiveMarkerConfig( TransformableObject* tobject );

    void tfTimerCallback(const ros::TimerEvent&);
    bool setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::PoseStamped pose_stamped);

    std::string focus_object_marker_name_;
    ros::NodeHandle* n_;

    boost::mutex mutex_;

    ros::Subscriber setcolor_sub_;
    ros::Subscriber setpose_sub_;
    ros::Subscriber addpose_sub_;
    ros::Subscriber addpose_relative_sub_;

    ros::Subscriber set_r_sub_;
    ros::Subscriber set_sm_r_sub_;
    ros::Subscriber set_h_sub_;
    ros::Subscriber set_x_sub_;
    ros::Subscriber set_y_sub_;
    ros::Subscriber set_z_sub_;

    ros::ServiceServer get_pose_srv_;
    ros::ServiceServer set_pose_srv_;
    ros::ServiceServer get_color_srv_;
    ros::ServiceServer set_color_srv_;
    ros::ServiceServer get_focus_srv_;
    ros::ServiceServer set_focus_srv_;
    ros::ServiceServer get_type_srv_;
    ros::ServiceServer get_exist_srv_;
    ros::ServiceServer set_dimensions_srv;
    ros::ServiceServer get_dimensions_srv;
    ros::Publisher marker_dimensions_pub_;
    ros::ServiceServer request_marker_operate_srv_;

    boost::shared_ptr <dynamic_reconfigure::Server<InteractiveSettingConfig> > config_srv_;

    ros::Subscriber setrad_sub_;
    ros::Publisher focus_text_pub_;
    ros::Publisher focus_pose_pub_;
    interactive_markers::InteractiveMarkerServer* server_;
    map<string, TransformableObject*> transformable_objects_map_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    int torus_udiv_;
    int torus_vdiv_;
    bool display_interactive_manipulator_;

    ros::Timer tf_timer;
  };
}

#endif
