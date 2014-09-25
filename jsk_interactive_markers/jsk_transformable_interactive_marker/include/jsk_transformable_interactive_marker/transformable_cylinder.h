#ifndef __TRANSFORMABLE_CYLINDER_H__
#define __TRANSFORMABLE_CYLINDER_H__


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <jsk_transformable_interactive_marker/transformable_object.h>

namespace jsk_transformable_interactive_marker
{
  class TransformableCylinder: public TransformableObject
  {
  public:
    TransformableCylinder( float radius, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setRGBA( float r , float g, float b, float a){cylinder_r_=r;cylinder_g_=g;cylinder_b_=b;cylinder_a_=a;};

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
