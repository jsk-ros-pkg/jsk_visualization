#include <jsk_transformable_interactive_marker/transformable_cylinder.h>

namespace jsk_transformable_interactive_marker{
  TransformableCylinder::TransformableCylinder( float radius, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    cylinder_radius_ = radius;
    cylinder_z_ = z;
    cylinder_r_ = r;
    cylinder_g_ = g;
    cylinder_b_ = b;
    cylinder_a_ = a;
    marker_.type = visualization_msgs::Marker::CYLINDER;

    frame_id_ = frame;
    name_ = name;
    description_ = description;
  }

  visualization_msgs::Marker TransformableCylinder::getVisualizationMsgMarker(){
    marker_.scale.x = cylinder_radius_;
    marker_.scale.y = cylinder_radius_;
    marker_.scale.z = cylinder_z_;
    marker_.color.r = cylinder_r_;
    marker_.color.g = cylinder_g_;
    marker_.color.b = cylinder_b_;
    marker_.color.a = cylinder_a_;
    return marker_;
  }
}
