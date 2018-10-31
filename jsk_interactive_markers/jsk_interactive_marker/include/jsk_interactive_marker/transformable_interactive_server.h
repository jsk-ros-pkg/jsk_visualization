#ifndef __TRANSFORMABLE_INTERACTIVE_SERVER_H__
#define __TRANSFORMABLE_INTERACTIVE_SERVER_H__

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/transformable_object.h>
#include <jsk_interactive_marker/yaml_menu_handler.h>
#include <jsk_interactive_marker/parent_and_child_interactive_marker_server.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
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
#include <jsk_interactive_marker/PoseStampedWithName.h>

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

    void setPose( const geometry_msgs::PoseStampedConstPtr &msg_ptr , bool for_interactive_control=false);
    void addPose(geometry_msgs::Pose msg);
    void addPoseRelative(geometry_msgs::Pose msg);

    void setControlRelativePose(geometry_msgs::Pose msg);

    void setColor(std_msgs::ColorRGBA msg);

    void insertNewBox( std::string frame_id, std::string name, std::string description );
    void insertNewCylinder( std::string frame_id, std::string name, std::string description );
    void insertNewTorus( std::string frame_id, std::string name, std::string description );
    void insertNewMesh( std::string frame_id, std::string name, std::string description , std::string mesh_resource, bool mesh_use_embedded_materials);

    void insertNewObject(TransformableObject* tobject, std::string name);
    void eraseObject(std::string name);
    void eraseAllObject();
    void eraseFocusObject();

    void run();
    void focusTextPublish();
    void focusPosePublish();
    void focusObjectMarkerNamePublish();
    void focusInteractiveManipulatorDisplay();

    void enableInteractiveManipulatorDisplay(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                             const bool enable);

    void updateTransformableObject(TransformableObject* tobject);

    bool getPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req,jsk_interactive_marker::GetTransformableMarkerPose::Response &res, bool for_interactive_control);
    bool setPoseService(jsk_interactive_marker::SetTransformableMarkerPose::Request &req,jsk_interactive_marker::SetTransformableMarkerPose::Response &res, bool for_interactive_control);
    bool getColorService(jsk_interactive_marker::GetTransformableMarkerColor::Request &req,jsk_interactive_marker::GetTransformableMarkerColor::Response &res);
    bool setColorService(jsk_interactive_marker::SetTransformableMarkerColor::Request &req,jsk_interactive_marker::SetTransformableMarkerColor::Response &res);
    bool getFocusService(jsk_interactive_marker::GetTransformableMarkerFocus::Request &req,jsk_interactive_marker::GetTransformableMarkerFocus::Response &res);
    bool setFocusService(jsk_interactive_marker::SetTransformableMarkerFocus::Request &req,jsk_interactive_marker::SetTransformableMarkerFocus::Response &res);
    bool getTypeService(jsk_interactive_marker::GetType::Request &req,jsk_interactive_marker::GetType::Response &res);
    bool getExistenceService(jsk_interactive_marker::GetTransformableMarkerExistence::Request &req,jsk_interactive_marker::GetTransformableMarkerExistence::Response &res);
    bool setDimensionsService(jsk_interactive_marker::SetMarkerDimensions::Request &req,jsk_interactive_marker::SetMarkerDimensions::Response &res);
    bool getDimensionsService(jsk_interactive_marker::GetMarkerDimensions::Request &req,jsk_interactive_marker::GetMarkerDimensions::Response &res);
    bool hideService(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);
    bool showService(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& res);
    void publishMarkerDimensions();

    bool requestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate::Request &req,jsk_rviz_plugins::RequestMarkerOperate::Response &res);
    virtual void configCallback(InteractiveSettingConfig &config, uint32_t level);
    void SetInitialInteractiveMarkerConfig( TransformableObject* tobject );

    void tfTimerCallback(const ros::TimerEvent&);
    bool setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::PoseStamped pose_stamped, bool for_interactive_control=false);
    
    std::string focus_object_marker_name_;
    ros::NodeHandle* n_;

    boost::mutex mutex_;

    ros::Subscriber setcolor_sub_;
    ros::Subscriber setpose_sub_;
    ros::Subscriber setcontrolpose_sub_;
    ros::Subscriber addpose_sub_;
    ros::Subscriber addpose_relative_sub_;
    
    ros::Subscriber setcontrol_relative_sub_;
    
    ros::Subscriber set_r_sub_;
    ros::Subscriber set_sm_r_sub_;
    ros::Subscriber set_h_sub_;
    ros::Subscriber set_x_sub_;
    ros::Subscriber set_y_sub_;
    ros::Subscriber set_z_sub_;
    
    ros::ServiceServer hide_srv_;
    ros::ServiceServer show_srv_;
    ros::ServiceServer get_pose_srv_;
    ros::ServiceServer get_control_pose_srv_;
    ros::ServiceServer set_pose_srv_;
    ros::ServiceServer set_control_pose_srv_;
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

    std::shared_ptr <dynamic_reconfigure::Server<InteractiveSettingConfig> > config_srv_;

    ros::Subscriber setrad_sub_;
    ros::Publisher focus_name_text_pub_;
    ros::Publisher focus_pose_text_pub_;
    ros::Publisher focus_object_marker_name_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_with_name_pub_;
    interactive_markers::InteractiveMarkerServer* server_;
    map<string, TransformableObject*> transformable_objects_map_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    int torus_udiv_;
    int torus_vdiv_;
    jsk_interactive_marker::InteractiveSettingConfig config_;
    bool strict_tf_;
    int interactive_manipulator_orientation_;
    ros::Timer tf_timer;
    std::shared_ptr <YamlMenuHandler> yaml_menu_handler_ptr_;
  };
}

#endif
