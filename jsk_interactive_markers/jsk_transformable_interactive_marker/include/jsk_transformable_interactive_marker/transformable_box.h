#ifndef __TRANSFORMABLE_BOX_H__
#define __TRANSFORMABLE_BOX_H__


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <jsk_transformable_interactive_marker/transformable_object.h>

namespace jsk_transformable_interactive_marker
{
  class TransformableBox: public TransformableObject
  {
  public:
    TransformableBox( float length , float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    TransformableBox( float x, float y, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setXYZ( float x , float y, float z){box_x_=x;box_y_=y;box_z_=z;};
    void setRGBA( float r , float g, float b, float a){box_r_=r;box_g_=g;box_b_=b;box_a_=a;};

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

#endif
