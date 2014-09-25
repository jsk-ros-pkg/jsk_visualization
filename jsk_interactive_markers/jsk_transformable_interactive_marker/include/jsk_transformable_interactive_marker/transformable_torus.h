#ifndef __TRANSFORMABLE_TORUS_H__
#define __TRANSFORMABLE_TORUS_H__


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <jsk_transformable_interactive_marker/transformable_object.h>

namespace jsk_transformable_interactive_marker
{
  class TransformableTorus: public TransformableObject
  {
  public:
    TransformableTorus( float radius, float small_radius, int u_div, int v_div, float r, float g, float b, float a, std::string frame, std::string name, std::string description);

    visualization_msgs::Marker getVisualizationMsgMarker();
    void setRGBA( float r , float g, float b, float a){torus_r_=r;torus_g_=g;torus_b_=b;torus_a_=a;};

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

#endif
