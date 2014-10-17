#ifndef __TRANSFORMABLE_INTERACTIVE_SERVER_H__
#define __TRANSFORMABLE_INTERACTIVE_SERVER_H__

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/transformable_object.h>
#include <jsk_interactive_marker/RequestMarkerOperate.h>
#include <jsk_interactive_marker/GetType.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <jsk_rviz_plugins/OverlayText.h>
#include <iostream>
#include <sstream>
#include <tf/transform_listener.h>

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

    bool getPoseService(jsk_interactive_marker::GetPose::Request &req,jsk_interactive_marker::GetPose::Response &res);
    bool getTypeService(jsk_interactive_marker::GetType::Request &req,jsk_interactive_marker::GetType::Response &res);

    bool requestMarkerOperateService(jsk_interactive_marker::RequestMarkerOperate::Request &req,jsk_interactive_marker::RequestMarkerOperate::Response &res);

    std::string focus_object_marker_name_;
    ros::NodeHandle* n_;

    ros::Subscriber setcolor_sub_;
    ros::Subscriber setpose_sub_;
    ros::Subscriber addpose_sub_;

    ros::Subscriber set_r_sub_;
    ros::Subscriber set_sm_r_sub_;
    ros::Subscriber set_h_sub_;
    ros::Subscriber set_x_sub_;
    ros::Subscriber set_y_sub_;
    ros::Subscriber set_z_sub_;

    ros::ServiceServer get_pose_srv_;
    ros::ServiceServer get_type_srv_;
    ros::ServiceServer request_marker_operate_srv_;

    ros::Subscriber setrad_sub_;
    ros::Publisher focus_text_pub_;
    ros::Publisher focus_pose_pub_;
    interactive_markers::InteractiveMarkerServer* server_;
    map<string, TransformableObject*> transformable_objects_map_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    int torus_udiv_;
    int torus_vdiv_;
  };
}

#endif
