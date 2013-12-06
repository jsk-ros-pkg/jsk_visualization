#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/door_foot.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;

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
  marker.color.a = 1.0;

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

  return makeFootMarker(pose);
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
  return makeFootMarker(pose);

}
visualization_msgs::Marker DoorFoot::makeFootMarker(geometry_msgs::Pose pose){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  double PADDING_PARAM = 0.01;
  marker.scale.x = 0.27 + PADDING_PARAM;
  marker.scale.y = 0.14 + PADDING_PARAM;
  marker.scale.z = 0.05;

  marker.pose = pose;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;

  return marker;

}

visualization_msgs::InteractiveMarker DoorFoot::makeInteractiveMarker(){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = "/map";
  mk.header.stamp = ros::Time(0);
  mk.name = marker_name;
  mk.scale = 0.4;

  visualization_msgs::InteractiveMarkerControl triangleMarker;
  triangleMarker.always_visible = true;
  triangleMarker.markers.push_back( makeDoorMarker());
  triangleMarker.markers.push_back( makeKnobMarker());
  triangleMarker.markers.push_back( makeRFootMarker());
  triangleMarker.markers.push_back( makeLFootMarker());
  mk.controls.push_back( triangleMarker );
  
  im_helpers::add6DofControl(mk, true);
  return mk;
}

void DoorFoot::moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //  std::cout << "moved" << std::endl;
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
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("door_marker") );
  pnh_.param("push", push, true);

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
  ros::spin();
  return 0;
}


