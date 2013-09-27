#include <jsk_interactive_marker/interactive_marker_utils.h>


visualization_msgs::InteractiveMarker makeFingerControlMarker(const char *name, geometry_msgs::PoseStamped ps){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;
  int_marker.scale = 0.5;

  visualization_msgs::InteractiveMarkerControl control;

  //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;

  int_marker.controls.push_back(control);

  return int_marker;

}


visualization_msgs::InteractiveMarker makeSandiaHandMarker(geometry_msgs::PoseStamped ps){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;

  visualization_msgs::InteractiveMarkerControl control;

  control.markers.push_back(makeSandiaFinger0Marker("/right_f0_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f0_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f0_2"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger0Marker("/right_f1_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f1_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f1_2"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger0Marker("/right_f2_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f2_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f2_2"));
  int_marker.controls.push_back(control);

  control.markers.push_back(makeSandiaFinger0Marker("/right_f3_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f3_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f3_2"));
  int_marker.controls.push_back(control);

  return int_marker;

}




visualization_msgs::InteractiveMarker makeSandiaHandInteractiveMarker(geometry_msgs::PoseStamped ps, std::string hand, int finger, int link){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;
  

  visualization_msgs::InteractiveMarkerControl control;
  std::stringstream ss;
  //std::string frame = "/" + hand + "_" + finger + "_" + link;
  ss << hand << "_f" << finger << "_" << link;
  
  int_marker.name = ss.str() + "Marker";
  int_marker.header.frame_id = ss.str();
  int_marker.header.frame_id = "odom";
  std::cout << ss.str() << std::endl;
  std::string frame_id = "odom";
  //frame_id = ss.str();
  switch(link){
  case 0:
    control.markers.push_back(makeSandiaFinger0Marker(frame_id));
    break;
  case 1:
    control.markers.push_back(makeSandiaFinger1Marker(frame_id));
    break;
  case 2:
    control.markers.push_back(makeSandiaFinger2Marker(frame_id));
    break;
  default:
    break;
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back(control);

  return int_marker;

}



visualization_msgs::Marker makeSandiaFinger0Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  //marker.header.frame_id = "odom";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.003;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.023;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

visualization_msgs::Marker makeSandiaFinger1Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.024;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.067;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

visualization_msgs::Marker makeSandiaFinger2Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.024;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.018;
  marker.scale.y = 0.018;
  marker.scale.z = 0.057;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}
