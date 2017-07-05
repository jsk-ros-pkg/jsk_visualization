#include <jsk_interactive_marker/transformable_object.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>

#define PI 3.14159265

using namespace jsk_interactive_marker;

inline geometry_msgs::Pose inverse(geometry_msgs::Pose pose){
  Eigen::Affine3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  tf::poseEigenToMsg(pose_eigen.inverse(), pose);
  return pose;
}


TransformableObject::TransformableObject(){
  ROS_INFO("Init TransformableObject");
  control_offset_pose_.orientation.x = 0;
  control_offset_pose_.orientation.y = 0;
  control_offset_pose_.orientation.z = 0;
  control_offset_pose_.orientation.w = 1;
  pose_ = control_offset_pose_;
  display_interactive_manipulator_ = true;
  display_description_ = true;
}

void TransformableObject::setDisplayInteractiveManipulator(bool v)
{
  display_interactive_manipulator_ = v;
}

void TransformableObject::setDisplayDescription(bool v)
{
  display_description_ = v;
}

void TransformableObject::setInteractiveMarkerSetting(const InteractiveSettingConfig& config){
  display_interactive_manipulator_ = config.display_interactive_manipulator;
  display_description_ = config.display_description_only_selected ? false : true;
  interactive_manipulator_orientation_ = config.interactive_manipulator_orientation;
  interaction_mode_ = static_cast<unsigned int>(config.interaction_mode);
}

std::vector<visualization_msgs::InteractiveMarkerControl> TransformableObject::makeRotateTransFixControl(unsigned int orientation_mode){
  visualization_msgs::InteractiveMarkerControl control;

  std::vector<visualization_msgs::InteractiveMarkerControl> controls;
  control.orientation_mode = orientation_mode;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  controls.push_back(control);

  return controls;
};

void TransformableObject::addMarker(visualization_msgs::InteractiveMarker &int_marker, bool always_visible, unsigned int interaction_mode)
{
  visualization_msgs::Marker marker = getVisualizationMsgMarker();
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = always_visible;
  marker_control.markers.push_back(marker);
  marker_control.interaction_mode = interaction_mode;
  int_marker.controls.push_back(marker_control);
};

void TransformableObject::addControl(visualization_msgs::InteractiveMarker &int_marker)
{
  if(display_interactive_manipulator_){
    std::vector<visualization_msgs::InteractiveMarkerControl> rotate_controls = makeRotateTransFixControl(interactive_manipulator_orientation_);
    int_marker.controls.insert(int_marker.controls.end(), rotate_controls.begin(), rotate_controls.end());
  }
};

visualization_msgs::InteractiveMarker TransformableObject::getInteractiveMarker(){
  visualization_msgs::InteractiveMarker int_marker;

  addMarker(int_marker, true, interaction_mode_);
  addControl(int_marker);
  int_marker.header.frame_id = frame_id_;
  int_marker.name = name_;
  if (display_description_) {
    int_marker.description = description_;
  } else {
    int_marker.description = "";
  }
  int_marker.pose = pose_;
  int_marker.scale = getInteractiveMarkerScale();
  return int_marker;
};

void TransformableObject::setPose(geometry_msgs::Pose pose, bool for_interactive_control){
  if(for_interactive_control) {
    pose_ = pose;
  }
  else {
    Eigen::Affine3d control_offset_eigen;
    tf::poseMsgToEigen(control_offset_pose_, control_offset_eigen);
    Eigen::Affine3d pose_eigen;
    tf::poseMsgToEigen(pose, pose_eigen);
    tf::poseEigenToMsg(pose_eigen * control_offset_eigen, pose_);
  }
}

geometry_msgs::Pose TransformableObject::getPose(bool for_interactive_control){
  if(for_interactive_control) {
    return pose_;
  }
  else{
    geometry_msgs::Pose pose;
    Eigen::Affine3d control_offset_eigen;
    tf::poseMsgToEigen(control_offset_pose_, control_offset_eigen);
    Eigen::Affine3d pose_eigen;
    tf::poseMsgToEigen(pose_, pose_eigen);
    tf::poseEigenToMsg(pose_eigen * control_offset_eigen.inverse(), pose);
    //return pose;
    return pose;
  }
}


void TransformableObject::addPose(geometry_msgs::Pose msg, bool relative){
  Eigen::Vector3d original_p(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond original_q;
  tf::quaternionMsgToEigen(pose_.orientation, original_q);
  Eigen::Quaterniond diff_q;
  tf::quaternionMsgToEigen(msg.orientation, diff_q);
  Eigen::Quaterniond updated_q;
  if(relative) {
     original_p = Eigen::Affine3d(original_q) * original_p;
  }
  if(relative) {
    updated_q = original_q * diff_q;
  } else {
    updated_q = diff_q * original_q;
  }
  pose_.position.x += original_p[0];
  pose_.position.y += original_p[1];
  pose_.position.z += original_p[2];
  tf::quaternionEigenToMsg(updated_q, pose_.orientation);
}

void TransformableObject::publishTF(){
  tf::Transform transform;
  tf::poseMsgToTF(getPose(), transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id_, name_));
}

namespace jsk_interactive_marker{
  TransformableCylinder::TransformableCylinder( float radius, float z, float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    cylinder_radius_ = radius;
    cylinder_z_ = z;
    cylinder_r_ = r;
    cylinder_g_ = g;
    cylinder_b_ = b;
    cylinder_a_ = a;
    marker_.type = visualization_msgs::Marker::CYLINDER;
    type_ = jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER;

    frame_id_ = frame;
    name_ = name;
    description_ = description;
  }

  visualization_msgs::Marker TransformableCylinder::getVisualizationMsgMarker(){
    marker_.scale.x = cylinder_radius_ * 2.0;
    marker_.scale.y = cylinder_radius_ * 2.0;
    marker_.scale.z = cylinder_z_;
    marker_.color.r = cylinder_r_;
    marker_.color.g = cylinder_g_;
    marker_.color.b = cylinder_b_;
    marker_.color.a = cylinder_a_;
    marker_.pose = control_offset_pose_;
    return marker_;
  }
}

namespace jsk_interactive_marker{

  TransformableTorus::TransformableTorus( float radius, float small_radius, int u_div, int v_div, float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    torus_radius_ = radius;
    torus_small_radius_ = small_radius;
    torus_r_ = r;
    torus_g_ = g;
    torus_b_ = b;
    torus_a_ = a;
    marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    type_ = jsk_rviz_plugins::TransformableMarkerOperate::TORUS;

    frame_id_ = frame;
    name_ = name;
    description_ = description;

    u_division_num_ = u_div;
    v_division_num_ = v_div;
  }

  std::vector<geometry_msgs::Point > TransformableTorus::calcurateTriangleMesh(){
    std::vector<geometry_msgs::Point> triangle_mesh;
    float center_x = 0;
    float center_y = 0;
    float u_division_num = u_division_num_;
    float v_division_num = v_division_num_;
    std::vector<std::vector<geometry_msgs::Point> > points_array;
    for (int i = 0; i < u_division_num; i ++){
      std::vector<geometry_msgs::Point> points;
      float target_circle_x = torus_radius_ * cos( ( i / u_division_num) * 2 * PI) ;
      float target_circle_y = torus_radius_ * sin( ( i / u_division_num) * 2 * PI) ;
      for (int j = 0; j < v_division_num; j++){
        geometry_msgs::Point new_point;
        new_point.x = target_circle_x + torus_small_radius_ * cos ( (j / v_division_num) * 2 * PI) * cos( ( i / u_division_num) * 2 * PI);
        new_point.y = target_circle_y + torus_small_radius_ * cos ( (j / v_division_num) * 2 * PI) * sin( ( i / u_division_num) * 2 * PI);
        new_point.z = torus_small_radius_ * sin ( (j / v_division_num) * 2 * PI);
        points.push_back(new_point);
      }
      points_array.push_back(points);
    }

    //create mesh list;
    for(int i = 0; i < u_division_num; i++){
      std::vector<geometry_msgs::Point> target_points = points_array[i];
      float prev_index = i - 1, next_index = i + 1;
      if(prev_index < 0)
        prev_index = u_division_num - 1;
      if(next_index > u_division_num - 1)
        next_index = 0;
      std::vector<geometry_msgs::Point> prev_points = points_array[prev_index];
      std::vector<geometry_msgs::Point> next_points = points_array[next_index];
      for(int j = 0; j < v_division_num; j++){
        float next_point_index = j + 1;
        if( next_point_index > v_division_num - 1)
          next_point_index = 0;
        //first pushes
        triangle_mesh.push_back(target_points[j]);
        triangle_mesh.push_back(next_points[j]);
        triangle_mesh.push_back(target_points[next_point_index]);
        //second pushes
        triangle_mesh.push_back(target_points[j]);
        triangle_mesh.push_back(target_points[next_point_index]);
        triangle_mesh.push_back(prev_points[next_point_index]);
      }
    }

    return triangle_mesh;
  }

  visualization_msgs::Marker TransformableTorus::getVisualizationMsgMarker(){
    marker_.points = calcurateTriangleMesh();
    marker_.color.r = torus_r_;
    marker_.color.g = torus_g_;
    marker_.color.b = torus_b_;
    marker_.color.a = torus_a_;
    return marker_;
  }
}

namespace jsk_interactive_marker{

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
    type_ = jsk_rviz_plugins::TransformableMarkerOperate::BOX;

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

namespace jsk_interactive_marker{

  TransformableMesh::TransformableMesh( std::string frame, std::string name, std::string description, std::string mesh_resource, bool mesh_use_embedded_materials){
    marker_scale_ = 0.5;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    type_ = jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE;
    mesh_resource_ = mesh_resource;
    mesh_use_embedded_materials_ = mesh_use_embedded_materials;
    frame_id_ = frame;
    name_ = name;
    description_ = description;
  }

  visualization_msgs::Marker TransformableMesh::getVisualizationMsgMarker(){
    marker_.mesh_resource = mesh_resource_;
    marker_.mesh_use_embedded_materials = mesh_use_embedded_materials_;
    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;
    marker_.color.r = mesh_r_;
    marker_.color.g = mesh_g_;
    marker_.color.b = mesh_b_;
    marker_.color.a = mesh_a_;
    marker_.pose = inverse(control_offset_pose_);
    return marker_;
  }
}
