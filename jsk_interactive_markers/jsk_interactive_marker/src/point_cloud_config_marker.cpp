#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/point_cloud_config_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;

visualization_msgs::Marker PointCloudConfigMarker::makeBoxMarker(double size){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.3;
  return marker;
}

visualization_msgs::InteractiveMarker PointCloudConfigMarker::makeBoxInteractiveMarker(double size, std::string name){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = "/map";
  mk.header.stamp = ros::Time(0);
  mk.name = name;
  mk.scale = size * 1.05;

  visualization_msgs::InteractiveMarkerControl controlBox;
  controlBox.always_visible = true;
  controlBox.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  controlBox.markers.push_back( makeBoxMarker(size) );
  mk.controls.push_back( controlBox );

  visualization_msgs::InteractiveMarkerControl controlArrow;
  controlArrow.always_visible = true;
  controlArrow.orientation.w = 1;
  controlArrow.orientation.x = 1;
  controlArrow.orientation.y = 0;
  controlArrow.orientation.z = 0;

  controlArrow.name = "move_x";
  controlArrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(controlArrow);

  controlArrow.orientation.w = 1;
  controlArrow.orientation.x = 0;
  controlArrow.orientation.y = 1;
  controlArrow.orientation.z = 0;
  controlArrow.name = "move_z";
  controlArrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(controlArrow);

  controlArrow.orientation.w = 1;
  controlArrow.orientation.x = 0;
  controlArrow.orientation.y = 0;
  controlArrow.orientation.z = 1;
  controlArrow.name = "move_y";
  controlArrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(controlArrow);

  return mk;
}

interactive_markers::MenuHandler PointCloudConfigMarker::makeMenuHandler(){
  interactive_markers::MenuHandler mh;
  mh.insert("Get Point Cloud", boost::bind( &PointCloudConfigMarker::publishMarkerMsg, this, _1));

  resolution_menu_ = mh.insert( "Resolution" );
  resolution_10cm_menu_ = mh.insert( resolution_menu_, "10 cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_10cm_menu_, interactive_markers::MenuHandler::CHECKED );

  resolution_ = 0.1;
  checked_resolution_menu_ = resolution_10cm_menu_;


 resolution_5cm_menu_ = mh.insert( resolution_menu_, "5 cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_5cm_menu_, interactive_markers::MenuHandler::UNCHECKED );


  mh.insert("Cancel", boost::bind( &PointCloudConfigMarker::cancelCb, this, _1));
  mh.insert("Clear All Point Cloud", boost::bind( &PointCloudConfigMarker::clearCb, this, _1));
  /*
  resolution_1cm_menu_ = mh.insert( resolution_menu_, "10 cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_10cm_menu_, interactive_markers::MenuHandler::CHECKED );
  */
  return mh;
}


void PointCloudConfigMarker::moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::cout << "moved" << std::endl;
  //publishMarkerMsg( feedback );
}

void PointCloudConfigMarker::changeResolutionCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  menu_handler.setCheckState( checked_resolution_menu_ , interactive_markers::MenuHandler::UNCHECKED );
  checked_resolution_menu_ = feedback->menu_entry_id;

  if(feedback->menu_entry_id == resolution_10cm_menu_){
    resolution_ = 0.1;

  }else if(feedback->menu_entry_id == resolution_5cm_menu_){
    resolution_ = 0.05;
  }
    std::cout <<"resolution:" << resolution_ << std::endl;

  menu_handler.setCheckState( feedback->menu_entry_id , interactive_markers::MenuHandler::CHECKED );

  menu_handler.reApply( *server_ );
  server_->applyChanges();


}



visualization_msgs::Marker PointCloudConfigMarker::makeMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.pose = feedback->pose;
  marker.scale.x = size_;
  marker.scale.y = size_;
  marker.scale.z = size_;
  marker.color.r = resolution_;
  marker.color.g = resolution_ * 10;
  if(marker.color.g > 1.0){marker.color.g = 1.0;}
  marker.color.b = resolution_ * 10;
  if(marker.color.b > 1.0){marker.color.b = 1.0;}
  marker.color.a = 0.1;
  marker.header = feedback->header;
  return marker;
}

void PointCloudConfigMarker::publishMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  visualization_msgs::Marker marker = makeMarkerMsg(feedback);
  marker.id = marker_id++;
  pub_.publish(marker);
}


void PointCloudConfigMarker::cancelCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if(marker_id == 0){
    return;
  }
  visualization_msgs::Marker marker = makeMarkerMsg(feedback);
  marker.id = --marker_id;
  marker.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker);

}

void PointCloudConfigMarker::clearCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  marker_id = 0;
  
  visualization_msgs::Marker marker = makeMarkerMsg(feedback);
  marker.id = -1;
  marker.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker);
}


PointCloudConfigMarker::PointCloudConfigMarker () : nh_(), pnh_("~"), marker_id(0) {
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("point_cloud_config_marker") );

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name));
  
  pub_ =  pnh_.advertise<visualization_msgs::Marker> ("get", 1);
  
  visualization_msgs::InteractiveMarker boxIM = makeBoxInteractiveMarker(size_, marker_name);
  
  menu_handler = makeMenuHandler();

  server_->insert(boxIM,
		  boost::bind( &PointCloudConfigMarker::moveBoxCb, this, _1 ));
  menu_handler.apply(*server_, marker_name);
  server_->applyChanges();

  /*
  interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
  sub_menu_move_ = model_menu_.insert( "Move" );
  model_menu_.insert( sub_menu_move_, "Yes", 
		      boost::bind( &PointCloudConfigMarker::jointMoveCB, this, _1) );
  //    model_menu_.insert( "Move" ,
  //boost::bind( &PointCloudConfigMarker::jointMoveCB, this, _1) );
  model_menu_.insert( "Reset Marker Pose",
		      boost::bind( &PointCloudConfigMarker::resetMarkerCB, this, _1) );
  */
  
  return;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_config_marker");
  PointCloudConfigMarker pccm;
  ros::spin();
  return 0;
}


