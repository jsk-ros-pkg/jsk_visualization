#include <jsk_transformable_interactive_marker/transformable_box.h>

namespace jsk_transformable_interactive_marker{
  TransformableBox::TransformableBox( float length , float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    box_x_ = box_y_ = box_z_ = length;

    box_r_ = r;
    box_g_ = g;
    box_b_ = b;
    box_a_ = a;
    marker_.type = visualization_msgs::Marker::CUBE;

    frame_id_ = frame;
    name_ = name;
    description_ = description;
  }

  TransformableBox::TransformableBox( float x, float y, float z , float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    box_x_ = x;
    box_y_ = y;
    box_z_ = z;
    box_r_ = r;
    box_g_ = g;
    box_b_ = b;
    box_a_ = a;
    marker_.type = visualization_msgs::Marker::CUBE;

    frame_id_ = frame;
    name_ = name;
    description_ = description;
  }

  visualization_msgs::Marker TransformableBox::getVisualizationMsgMarker(){
    marker_.scale.x = box_x_;
    marker_.scale.y = box_y_;
    marker_.scale.z = box_z_;
    marker_.color.r = box_r_;
    marker_.color.g = box_g_;
    marker_.color.b = box_b_;
    marker_.color.a = box_a_;
    return marker_;
  }
}
