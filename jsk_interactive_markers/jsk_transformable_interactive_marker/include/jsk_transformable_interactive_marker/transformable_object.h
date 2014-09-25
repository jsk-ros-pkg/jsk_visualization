#ifndef __TRANSFORMABLE_OBJECT_H__
#define __TRANSFORMABLE_OBJECT_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <jsk_transformable_interactive_marker/GetPose.h>
#include <vector>
#include <algorithm>

namespace jsk_transformable_interactive_marker {
  class TransformableObject{
  public:
    TransformableObject();

    std::vector<visualization_msgs::InteractiveMarkerControl> makeRotateTransFixControl();

    visualization_msgs::InteractiveMarker getInteractiveMarker();
    virtual visualization_msgs::Marker getVisualizationMsgMarker(){};
    void addMarker(visualization_msgs::InteractiveMarker &int_marker, bool always_visible = true, unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D);
    void addControl(visualization_msgs::InteractiveMarker &int_marker, bool fixed = true);

    visualization_msgs::Marker marker_;
    geometry_msgs::Pose pose_;
    std::string name_;
    std::string frame_id_;
    std::string description_;

    void setPose(geometry_msgs::Pose pose){pose_=pose;};
    void addPose(geometry_msgs::Pose msg);
    geometry_msgs::Pose getPose(){return pose_;};
    virtual bool setRadius(std_msgs::Float32 recieve_val){return false;};
    virtual bool setSmallRadius(std_msgs::Float32 recieve_val){return false;};
    virtual bool setHeight(std_msgs::Float32 recieve_val){return false;};
    virtual bool setX(std_msgs::Float32 recieve_val){return false;};
    virtual bool setY(std_msgs::Float32 recieve_val){return false;};
    virtual bool setZ(std_msgs::Float32 recieve_val){return false;};
    virtual void setRGBA(float r , float g, float b, float a){};

    virtual float getInteractiveMarkerScale(){};
  };
};

#endif
