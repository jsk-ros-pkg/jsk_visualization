#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/triangle_foot.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;

visualization_msgs::Marker TriangleFoot::makeTriangleMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.03;//line width;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  
  geometry_msgs::Point point1;

  geometry_msgs::Point point2;
  point2.z = 0.3048; //12in

  geometry_msgs::Point point3;

  point3.y = -0.6096; //24in
  if(reverse){
    point3.y = - point3.y;
  }
  point3.z = 0.0;

  marker.points.push_back(point1);
  marker.points.push_back(point2);
  marker.points.push_back(point3);
  marker.points.push_back(point1);
  return marker;
}

visualization_msgs::Marker TriangleFoot::makeRFootMarker(){
  geometry_msgs::Pose pose;
  if(reverse){
    pose.position.x = -0.65;
    pose.position.y = 0.65;
    pose.position.z = -0.910;
  }else{
    pose.position.x = -0.8;
    pose.position.y = 0.0;
    pose.position.z = -0.910;
  }
  pose.orientation.w = 1.0;
  return makeFootMarker(pose);
}
visualization_msgs::Marker TriangleFoot::makeLFootMarker(){
  geometry_msgs::Pose pose;
  if(reverse){
    pose.position.x = -0.65;
    pose.position.y = 0.35;
    pose.position.z = -0.910;
  }else{
    pose.position.x = -0.8;
    pose.position.y = -0.3;
    pose.position.z = -0.910;
  }
  pose.orientation.w = 1.0;
  return makeFootMarker(pose);

}
visualization_msgs::Marker TriangleFoot::makeFootMarker(geometry_msgs::Pose pose){
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

visualization_msgs::InteractiveMarker TriangleFoot::makeInteractiveMarker(){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = "/map";
  mk.header.stamp = ros::Time(0);
  mk.name = marker_name;

  mk.scale = 0.4;

  mk.pose.position.z = 0.910;

  visualization_msgs::InteractiveMarkerControl triangleMarker;
  triangleMarker.always_visible = true;
  triangleMarker.markers.push_back( makeTriangleMarker());
  triangleMarker.markers.push_back( makeRFootMarker());
  triangleMarker.markers.push_back( makeLFootMarker());
  mk.controls.push_back( triangleMarker );
  
  im_helpers::add6DofControl(mk, true);
  return mk;
}

void TriangleFoot::moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //  std::cout << "moved" << std::endl;
}

void TriangleFoot::reverseTriangleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  reverse ^= true;
  updateBoxInteractiveMarker();
}


interactive_markers::MenuHandler TriangleFoot::makeMenuHandler(){
  interactive_markers::MenuHandler mh;
  mh.insert("Reverse Triangle", boost::bind( &TriangleFoot::reverseTriangleCb, this, _1));
  return mh;
}


void TriangleFoot::updateBoxInteractiveMarker(){
  visualization_msgs::InteractiveMarker boxIM = makeInteractiveMarker();

  server_->insert(boxIM,
		  boost::bind( &TriangleFoot::moveBoxCb, this, _1 ));
  menu_handler.apply(*server_, marker_name);
  server_->applyChanges();
}

TriangleFoot::TriangleFoot () : nh_(), pnh_("~") {
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("triangle_marker") );
  pnh_.param("reverse", reverse, false);

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
  ros::init(argc, argv, "triangle_foot_marker");
  TriangleFoot triFoot;
  ros::spin();
  return 0;
}


