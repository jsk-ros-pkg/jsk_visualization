#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/triangle_foot.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;

visualization_msgs::Marker TriangleFoot::makeTriangleMarker(){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.05;//line width;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.8;

  
  geometry_msgs::Point point1;
  point1.y = 1.0;
  point1.z = 1.0;

  geometry_msgs::Point point2;
  point2.y = 0.0;
  point2.z = 1.0;

  geometry_msgs::Point point3;
  point3.y = 1.0;
  point3.z = 0.0;

  marker.points.push_back(point1);
  marker.points.push_back(point2);
  marker.points.push_back(point3);
  marker.points.push_back(point1);
  return marker;
}

visualization_msgs::Marker TriangleFoot::makeRFootMarker(){
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.orientation.w = 1.0;
  return makeFootMarker(pose);
}
visualization_msgs::Marker TriangleFoot::makeLFootMarker(){
  geometry_msgs::Pose pose;
  pose.position.x = -1;
  pose.orientation.w = 1.0;
  return makeFootMarker(pose);

}
visualization_msgs::Marker TriangleFoot::makeFootMarker(geometry_msgs::Pose pose){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.pose = pose;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.8;

  return marker;

}

visualization_msgs::InteractiveMarker TriangleFoot::makeInteractiveMarker(){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = "/map";
  mk.header.stamp = ros::Time(0);
  mk.name = marker_name;

  mk.scale = 1.0;

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
  std::cout << "moved" << std::endl;
}

void TriangleFoot::updateBoxInteractiveMarker(){
  visualization_msgs::InteractiveMarker boxIM = makeInteractiveMarker();

  server_->insert(boxIM,
		  boost::bind( &TriangleFoot::moveBoxCb, this, _1 ));
  server_->applyChanges();
}

TriangleFoot::TriangleFoot () : nh_(), pnh_("~") {
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("triangle_marker") );

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name));

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


