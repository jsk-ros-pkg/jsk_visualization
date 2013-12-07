#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/door_foot.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;


visualization_msgs::Marker DoorFoot::makeRWallMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.7;
  marker.scale.z = 2.0;
  
  marker.pose.position.y = - marker.scale.y / 2;
  marker.pose.position.z = marker.scale.z / 2;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.9;

  return marker;
}

visualization_msgs::Marker DoorFoot::makeLWallMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.7;
  marker.scale.z = 2.0;
  
  marker.pose.position.y = 0.914 + marker.scale.y / 2;
  marker.pose.position.z = marker.scale.z / 2;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.9;

  return marker;
}


visualization_msgs::Marker DoorFoot::makeDoorMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.914;
  marker.scale.z = 2.0;
  
  marker.pose.position.y = marker.scale.y / 2;
  marker.pose.position.z = marker.scale.z / 2;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.3;

  return marker;
}

visualization_msgs::Marker DoorFoot::makeKnobMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.03;
  marker.scale.y = 0.1;
  marker.scale.z = 0.03;
  
  marker.pose.position.x = -0.05;
  marker.pose.position.y = 0.914 - 0.1 - marker.scale.y/2 ;
  marker.pose.position.z = 0.9398;//37 in
  marker.pose.orientation.w = 1.0;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker DoorFoot::makeKnobMarker(int position){
  double size = 0.02;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.02;
  marker.scale.y = size;
  marker.scale.z = 0.03;
  
  marker.pose.position.x = -0.07;
  marker.pose.position.y = 0.914 - marker.scale.y/2 - position * size;
  marker.pose.position.z = 0.9398;//37 in
  marker.pose.orientation.w = 1.0;
  
  switch(position % 5){
  case 0:
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    break;
  case 1:
    marker.color.r = 0.5;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    break;
  case 2:
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    break;
  case 3:
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.7;
    break;
  case 4:
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    break;
  }
  marker.color.a = 0.7;

  return marker;
}


visualization_msgs::Marker DoorFoot::makeRFootMarker(){
  geometry_msgs::Pose pose;
  if(push){
    //Right Foot Position
    pose.position.x = -0.523;
    pose.position.y = 0.270;
    pose.position.z = 0;
    pose.orientation.w = 0.766;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0.642788;
  }else{
    pose.position.x = -0.8;
    pose.position.y = 0.0;
    pose.position.z = -0.910;
    pose.orientation.w = 1.0;
  }

  return makeFootMarker(pose, true);
}
visualization_msgs::Marker DoorFoot::makeLFootMarker(){
  geometry_msgs::Pose pose;
  if(push){
    pose.position.x = -0.747;
    pose.position.y = 0.310;
    pose.position.z = 0;
    pose.orientation.w = 0.766;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0.642788;
  }else{
    pose.position.x = -0.8;
    pose.position.y = -0.3;
    pose.position.z = -0.910;
    pose.orientation.w = 1.0;
  }
  return makeFootMarker(pose, false);

}


visualization_msgs::Marker DoorFoot::makeFootMarker(geometry_msgs::Pose pose, bool right){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  double PADDING_PARAM = 0.01;
  marker.scale.x = 0.27 + PADDING_PARAM;
  marker.scale.y = 0.14 + PADDING_PARAM;
  marker.scale.z = 0.05;

  marker.pose = pose;

  if(right){
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }else{
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  marker.color.a = 1.0;

  return marker;
}


visualization_msgs::InteractiveMarker DoorFoot::makeInteractiveMarker(){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = "/map";
  mk.header.stamp = ros::Time(0);
  mk.name = marker_name;
  mk.scale = 0.3;
  mk.pose = door_pose;

  visualization_msgs::InteractiveMarkerControl triangleMarker;
  triangleMarker.always_visible = true;
  triangleMarker.markers.push_back( makeRWallMarker());
  triangleMarker.markers.push_back( makeLWallMarker());
  triangleMarker.markers.push_back( makeDoorMarker());

  if(use_color_knob){
    for(int i=0;i<12;i++){
      triangleMarker.markers.push_back( makeKnobMarker(i));
    }
  }else{
    triangleMarker.markers.push_back( makeKnobMarker());
  }

  if (footstep_show_initial_p_) {
    for(size_t i=0; i<2; i++){
      if(foot_list[i].header.frame_id == "right"){
        triangleMarker.markers.push_back( makeFootMarker( foot_list[i].pose, true ));
      }else{
        triangleMarker.markers.push_back( makeFootMarker( foot_list[i].pose, false ));
      }
    }
  }
  else {
    int first_index = footstep_index_ * 2;
    int second_index = footstep_index_ * 2 + 1;
    if (foot_list.size() > first_index) {
      triangleMarker.markers.push_back( makeFootMarker( foot_list[first_index].pose,
                                                        foot_list[first_index].header.frame_id == "right"));
    }
    if (foot_list.size() > second_index) {
      triangleMarker.markers.push_back( makeFootMarker( foot_list[second_index].pose,
                                                        foot_list[second_index].header.frame_id == "right"));
    }
  }
  
  

  mk.controls.push_back( triangleMarker );
  
  im_helpers::add6DofControl(mk, true);
  return mk;
}

void DoorFoot::moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  door_pose = feedback->pose;
  //  std::cout << "moved" << std::endl;
}

void DoorFoot::showStandLocationCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  footstep_show_initial_p_ = true;
  updateBoxInteractiveMarker();
}
void DoorFoot::showNextStepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if (foot_list.size() > 2) {
    if (footstep_show_initial_p_) {
      footstep_index_ = 1;
      footstep_show_initial_p_ = false;
    }
    else {
      ++footstep_index_;
      if (foot_list.size() / 2 == footstep_index_) {
	footstep_index_ = 1;
      }
    }
  }
  updateBoxInteractiveMarker();
}
void DoorFoot::showPreviousStepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if (foot_list.size() > 2) {
    if (footstep_show_initial_p_) {
      footstep_index_ = 1;
      footstep_show_initial_p_ = false;
    }
    else {
      --footstep_index_;
      if (footstep_index_ == 0) {
	footstep_index_ = (foot_list.size() - 1)/ 2;
      }
    }
  }
  updateBoxInteractiveMarker();
}

void DoorFoot::pushDoorCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  push = true;
  updateBoxInteractiveMarker();
}

void DoorFoot::pullDoorCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  push = false;
  updateBoxInteractiveMarker();
}


interactive_markers::MenuHandler DoorFoot::makeMenuHandler(){
  interactive_markers::MenuHandler mh;
  mh.insert("Push Door", boost::bind( &DoorFoot::pushDoorCb, this, _1));
  mh.insert("Pull Door", boost::bind( &DoorFoot::pullDoorCb, this, _1));
  mh.insert("Show Initial Stand Location", boost::bind( &DoorFoot::showStandLocationCb, this, _1));
  mh.insert("Show Next Step", boost::bind( &DoorFoot::showNextStepCb, this, _1));
  mh.insert("Show Previous Step", boost::bind( &DoorFoot::showPreviousStepCb, this, _1));
  return mh;
}


void DoorFoot::updateBoxInteractiveMarker(){
  visualization_msgs::InteractiveMarker boxIM = makeInteractiveMarker();

  server_->insert(boxIM,
		  boost::bind( &DoorFoot::moveBoxCb, this, _1 ));
  menu_handler.apply(*server_, marker_name);
  server_->applyChanges();
}

DoorFoot::DoorFoot () : nh_(), pnh_("~") {
  footstep_show_initial_p_ = true;
  footstep_index_ = -1;
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("door_marker") );
  pnh_.param("push", push, true);
  pnh_.param("use_color_knob", use_color_knob, true);

  XmlRpc::XmlRpcValue v;
  pnh_.param("foot_list", v, v);
  for(int i=0; i< v.size(); i++){
    XmlRpc::XmlRpcValue foot = v[i];
    geometry_msgs::Pose p = getPose(foot["pose"]);
    geometry_msgs::PoseStamped footPose;
    footPose.pose = p;
    footPose.header.frame_id.assign(foot["leg"]);
    foot_list.push_back(footPose);
  }

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name));

  menu_handler = makeMenuHandler();
  updateBoxInteractiveMarker();
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "door_foot_marker");
  DoorFoot triFoot;
  int counter = 0;
  ros::spin();
  return 0;
}


geometry_msgs::Pose getPose( XmlRpc::XmlRpcValue val){
  geometry_msgs::Pose p;
  XmlRpc::XmlRpcValue pos = val["position"];
  p.position.x = getXmlValue(pos["x"]);
  p.position.y = getXmlValue(pos["y"]);
  p.position.z = getXmlValue(pos["z"]);

  XmlRpc::XmlRpcValue ori = val["orientation"];
  p.orientation.x = getXmlValue(ori["x"]);
  p.orientation.y = getXmlValue(ori["y"]);
  p.orientation.z = getXmlValue(ori["z"]);
  p.orientation.w = getXmlValue(ori["w"]);

  return p;
}

double getXmlValue( XmlRpc::XmlRpcValue val ){
  switch(val.getType()){
  case XmlRpc::XmlRpcValue::TypeInt:
    return (double)((int)val);
  case XmlRpc::XmlRpcValue::TypeDouble:
    return (double)val;
  default:
    return 0;
  }
}
  
