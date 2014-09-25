#include <jsk_transformable_interactive_marker/transformable_object.h>

using namespace jsk_transformable_interactive_marker;

TransformableObject::TransformableObject(){
  ROS_INFO("Init TransformableObject");
  pose_.orientation.x = 0;
  pose_.orientation.y = 0;
  pose_.orientation.z = 0;
  pose_.orientation.w = 1;
}

std::vector<visualization_msgs::InteractiveMarkerControl> TransformableObject::makeRotateTransFixControl(){
  visualization_msgs::InteractiveMarkerControl control;

  std::vector<visualization_msgs::InteractiveMarkerControl> controls;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
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

void TransformableObject::addControl(visualization_msgs::InteractiveMarker &int_marker, bool fixed)
{
  if(fixed){
    std::vector<visualization_msgs::InteractiveMarkerControl> rotate_controls = makeRotateTransFixControl();
    int_marker.controls.insert(int_marker.controls.end(), rotate_controls.begin(), rotate_controls.end());
  }
};

visualization_msgs::InteractiveMarker TransformableObject::getInteractiveMarker(){
  visualization_msgs::InteractiveMarker int_marker;

  addMarker(int_marker);
  addControl(int_marker);
  int_marker.header.frame_id = frame_id_;
  int_marker.name = name_;
  int_marker.description = description_;
  int_marker.pose = pose_;
  int_marker.scale = getInteractiveMarkerScale();
  return int_marker;
};

void TransformableObject::addPose(geometry_msgs::Pose msg){
  pose_.position.x += msg.position.x;
  pose_.position.y += msg.position.y;
  pose_.position.z += msg.position.z;
  float tmp_x = pose_.orientation.x, tmp_y = pose_.orientation.y,
    tmp_z = pose_.orientation.z, tmp_w = pose_.orientation.w;
  pose_.orientation.w =
    tmp_w * msg.orientation.w -
    tmp_x * msg.orientation.x -
    tmp_y * msg.orientation.y -
    tmp_z * msg.orientation.z;
  pose_.orientation.x =
    tmp_y * msg.orientation.z -
    tmp_z * msg.orientation.y +
    tmp_y * msg.orientation.x +
    tmp_x * msg.orientation.w;
  pose_.orientation.y =
    tmp_z * msg.orientation.x -
    tmp_x * msg.orientation.z +
    tmp_w * msg.orientation.y +
    tmp_y * msg.orientation.w ;
  pose_.orientation.z =
    tmp_x * msg.orientation.y -
    tmp_y * msg.orientation.x +
    tmp_w * msg.orientation.z +
    tmp_z * msg.orientation.w;
}
