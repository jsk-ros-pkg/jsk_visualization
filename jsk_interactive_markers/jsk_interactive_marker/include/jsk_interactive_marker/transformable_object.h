#ifndef __TRANSFORMABLE_OBJECT_H__
#define __TRANSFORMABLE_OBJECT_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <jsk_interactive_marker/GetPose.h>
#include <jsk_rviz_plugins/RequestMarkerOperate.h>
#include <vector>
#include <algorithm>
#include <jsk_interactive_marker/InteractiveSettingConfig.h>

namespace jsk_interactive_marker {
  class TransformableObject{
  public:
    TransformableObject();

    std::vector<visualization_msgs::InteractiveMarkerControl> makeRotateTransFixControl();

    tf::TransformBroadcaster br;
    visualization_msgs::InteractiveMarker getInteractiveMarker();
    virtual visualization_msgs::Marker getVisualizationMsgMarker(){};
    void addMarker(visualization_msgs::InteractiveMarker &int_marker, bool always_visible = true, unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D);
    void addControl(visualization_msgs::InteractiveMarker &int_marker);

    visualization_msgs::Marker marker_;
    geometry_msgs::Pose pose_;
    std::string name_;
    std::string frame_id_;
    std::string description_;
    int type_;
    bool display_interactive_manipulator_;

    void setPose(geometry_msgs::Pose pose);
    void addPose(geometry_msgs::Pose msg, bool relative=false);
    void publishTF();
    std::string getFrameId() { return frame_id_; }
    geometry_msgs::Pose getPose(){return pose_;};
    void setInteractiveMarkerSetting(InteractiveSettingConfig config);
    virtual bool setRadius(std_msgs::Float32 recieve_val){return false;};
    virtual bool setSmallRadius(std_msgs::Float32 recieve_val){return false;};
    virtual bool setHeight(std_msgs::Float32 recieve_val){return false;};
    virtual bool setX(std_msgs::Float32 recieve_val){return false;};
    virtual bool setY(std_msgs::Float32 recieve_val){return false;};
    virtual bool setZ(std_msgs::Float32 recieve_val){return false;};
    virtual void setRGBA(float r , float g, float b, float a){};
    virtual void setXYZ(float x , float y, float z){};
    virtual void getXYZ(float &x, float &y, float&z){};
    virtual void setRSR(float r , float sr){};
    virtual void getRSR(float &r , float &sr){};
    virtual void setRZ(float r , float z){};
    virtual void getRZ(float &r , float &z){};

    int getType() { return type_; };
    void setType(int type) { type_ = type; return; };

    virtual float getInteractiveMarkerScale(){};
  };
};

namespace jsk_interactive_marker
{
  class TransformableBox: public TransformableObject
  {
  public:
    TransformableBox( float length , float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    TransformableBox( float x, float y, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setXYZ( float x , float y, float z){box_x_=x;box_y_=y;box_z_=z;};
    void setRGBA( float r , float g, float b, float a){box_r_=r;box_g_=g;box_b_=b;box_a_=a;};
    void getXYZ(float &x, float &y, float&z){x=box_x_;y=box_y_;z=box_z_;};

    bool setX(std_msgs::Float32 x){box_x_=x.data;return true;};
    bool setY(std_msgs::Float32 y){box_y_=y.data;return true;};
    bool setZ(std_msgs::Float32 z){box_z_=z.data;return true;};

    float getInteractiveMarkerScale(){return 1+std::max(std::max(box_x_, box_y_),box_z_);};

    float box_x_;
    float box_y_;
    float box_z_;
    float box_r_;
    float box_g_;
    float box_b_;
    float box_a_;
  };
};

namespace jsk_interactive_marker
{
  class TransformableTorus: public TransformableObject
  {
  public:
    TransformableTorus( float radius, float small_radius, int u_div, int v_div, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setRGBA( float r , float g, float b, float a){torus_r_=r;torus_g_=g;torus_b_=b;torus_a_=a;};
    void setRSR( float r , float sr){torus_radius_=r; torus_small_radius_=sr;};
    void getRSR(float &r , float &sr){r=torus_radius_; sr=torus_small_radius_;};

    bool setRadius(std_msgs::Float32 r){torus_radius_=r.data;return true;};
    bool setSmallRadius(std_msgs::Float32 sr){torus_small_radius_=sr.data;return true;};

    std::vector<geometry_msgs::Point > calcurateTriangleMesh();

    float getInteractiveMarkerScale(){return (torus_radius_+torus_small_radius_+1);};

    float torus_radius_;
    float torus_small_radius_;
    float torus_r_;
    float torus_g_;
    float torus_b_;
    float torus_a_;

    int u_division_num_;
    int v_division_num_;
  };
};

namespace jsk_interactive_marker
{
  class TransformableCylinder: public TransformableObject
  {
  public:
    TransformableCylinder( float radius, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setRGBA( float r , float g, float b, float a){cylinder_r_=r;cylinder_g_=g;cylinder_b_=b;cylinder_a_=a;};
    void setRZ( float r , float z){cylinder_radius_=r; cylinder_z_=z;};
    void getRZ(float &r , float &z){r=cylinder_radius_; z=cylinder_z_;};

    bool setRadius(std_msgs::Float32 r){cylinder_radius_=r.data;return true;};
    bool setZ(std_msgs::Float32 z){cylinder_z_=z.data;return true;};

    float getInteractiveMarkerScale(){return 1+std::max(cylinder_radius_, cylinder_z_);};

    float cylinder_radius_;
    float cylinder_z_;
    float cylinder_r_;
    float cylinder_g_;
    float cylinder_b_;
    float cylinder_a_;
  };
};

#endif
