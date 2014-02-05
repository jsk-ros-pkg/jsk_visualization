#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/point_cloud_config_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

using namespace std;

visualization_msgs::Marker PointCloudConfigMarker::makeBoxMarker(geometry_msgs::Vector3 size){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale = size;

  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.3;
  return marker;
}

visualization_msgs::Marker PointCloudConfigMarker::makeTextMarker(geometry_msgs::Vector3 size){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = "hello world";
  marker.scale.z = 0.1;
  marker.scale.y = 0.1;
  marker.scale.x = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::InteractiveMarker PointCloudConfigMarker::makeBoxInteractiveMarker(MarkerControlConfig mconfig, std::string name){
  visualization_msgs::InteractiveMarker mk;
  mk.header.frame_id = base_frame;
  if (latest_feedback_) {
    mk.pose = latest_feedback_->pose;
  }
  //mk.pose = 
  std::string description;
  std::stringstream ss;
  ss << "size: (" << mconfig.size.x
     << ", " << mconfig.size.y
     << ", " << mconfig.size.z << ")" << std::endl;
  ss << "resolution: " << mconfig.resolution_ << std::endl;
  description = ss.str();
  
  mk.header.stamp = ros::Time(0);
  mk.name = name;
  //mk.scale = size * 1.05;

  mk.scale = 1.0;
  mk.scale = max(mconfig.size.x, max(mconfig.size.y, mconfig.size.z));

  visualization_msgs::InteractiveMarkerControl controlBox;
  controlBox.always_visible = true;
  controlBox.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  controlBox.markers.push_back( makeBoxMarker(mconfig.size) );
  mk.controls.push_back( controlBox );

  visualization_msgs::InteractiveMarkerControl textBox;
  textBox.always_visible = true;
  textBox.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  visualization_msgs::Marker text_marker = makeTextMarker(mconfig.size);
  text_marker.text = description;
  textBox.markers.push_back( text_marker );
  mk.controls.push_back( textBox );
  
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
  mh.insert("Add Point Cloud ROI", boost::bind( &PointCloudConfigMarker::publishMarkerMsg, this, _1));

  mh.insert("Start", boost::bind( &PointCloudConfigMarker::publishMarkerMsg, this, _1));

  mh.insert("Stop", boost::bind( &PointCloudConfigMarker::publishMarkerMsg, this, _1));



  resolution_menu_ = mh.insert( "Resolution" );
  resolution_20cm_menu_ = mh.insert( resolution_menu_, "20cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_20cm_menu_, interactive_markers::MenuHandler::UNCHECKED );


  resolution_10cm_menu_ = mh.insert( resolution_menu_, "10cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_10cm_menu_, interactive_markers::MenuHandler::UNCHECKED );


 resolution_5cm_menu_ = mh.insert( resolution_menu_, "5cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_5cm_menu_, interactive_markers::MenuHandler::CHECKED );
  marker_control_config.resolution_ = 0.05;
  checked_resolution_menu_ = resolution_5cm_menu_;
  //box size menu
  box_size_menu_ = mh.insert( "Box Size" );
  // 100x100x100
  box_size_100_menu_ = mh.insert( box_size_menu_, "100 x 100 x 100", boost::bind( &PointCloudConfigMarker::changeBoxSizeCb, this, _1));
  mh.setCheckState( box_size_100_menu_, interactive_markers::MenuHandler::UNCHECKED );
  
  // 50x50x50
  box_size_50_menu_ = mh.insert( box_size_menu_, "50 x 50 x 50", boost::bind( &PointCloudConfigMarker::changeBoxSizeCb, this, _1));
  mh.setCheckState( box_size_50_menu_, interactive_markers::MenuHandler::UNCHECKED );

  checked_box_size_menu_ = box_size_50_menu_;

  // 25x25x25
  box_size_25_menu_ = mh.insert( box_size_menu_, "25 x 25 x 25", boost::bind( &PointCloudConfigMarker::changeBoxSizeCb, this, _1));
  mh.setCheckState( box_size_25_menu_, interactive_markers::MenuHandler::UNCHECKED );

  mh.insert("Cancel", boost::bind( &PointCloudConfigMarker::cancelCb, this, _1));
  mh.insert("Clear All Point Cloud", boost::bind( &PointCloudConfigMarker::clearCb, this, _1));
  /*
  resolution_1cm_menu_ = mh.insert( resolution_menu_, "10 cm", boost::bind( &PointCloudConfigMarker::changeResolutionCb, this, _1));
  mh.setCheckState( resolution_10cm_menu_, interactive_markers::MenuHandler::CHECKED );
  */
  return mh;
}

void PointCloudConfigMarker::publishCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
  current_pose_pub_.publish(pose);
}

void PointCloudConfigMarker::publishCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = feedback->header;
  pose_stamped.pose = feedback->pose;
  current_pose_pub_.publish(pose_stamped);
}

void PointCloudConfigMarker::moveBoxCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //std::cout << "moved" << std::endl;
  publishCurrentPose(feedback);
  latest_feedback_ = feedback;
}

void PointCloudConfigMarker::changeResolutionCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  menu_handler.setCheckState( checked_resolution_menu_ , interactive_markers::MenuHandler::UNCHECKED );
  checked_resolution_menu_ = feedback->menu_entry_id;

  if(feedback->menu_entry_id == resolution_10cm_menu_){
      marker_control_config.resolution_ = 0.1;

  }else if(feedback->menu_entry_id == resolution_5cm_menu_){
      marker_control_config.resolution_ = 0.05;
  }
  else if(feedback->menu_entry_id == resolution_20cm_menu_){
      marker_control_config.resolution_ = 0.2;
  }
    std::cout <<"resolution:" <<   marker_control_config.resolution_ << std::endl;

  menu_handler.setCheckState( feedback->menu_entry_id , interactive_markers::MenuHandler::CHECKED );

  menu_handler.reApply( *server_ );
  server_->applyChanges();
  latest_feedback_ = feedback;

}


void PointCloudConfigMarker::changeBoxResolution(const std_msgs::Float32::ConstPtr &msg){
  marker_control_config.resolution_ = msg->data;
  updateBoxInteractiveMarker();
}

void PointCloudConfigMarker::changeBoxSize(geometry_msgs::Vector3 size){
  marker_control_config.size = size;
  updateBoxInteractiveMarker();
}


void PointCloudConfigMarker::changeBoxSizeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  menu_handler.setCheckState( checked_resolution_menu_ , interactive_markers::MenuHandler::UNCHECKED );
  checked_resolution_menu_ = feedback->menu_entry_id;

  menu_handler.setCheckState( feedback->menu_entry_id , interactive_markers::MenuHandler::CHECKED );

  geometry_msgs::Vector3 vec;
  if(feedback->menu_entry_id == box_size_100_menu_){
    vec.x = 1.0;
    vec.y = 1.0;
    vec.z = 1.0;
    changeBoxSize(vec);
  }else if(feedback->menu_entry_id == box_size_50_menu_){

  }
}




visualization_msgs::Marker PointCloudConfigMarker::makeMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.pose = feedback->pose;
  marker.scale.x = marker_control_config.size.x;
  marker.scale.y = marker_control_config.size.y;
  marker.scale.z = marker_control_config.size.z;
  marker.color.r =   marker_control_config.resolution_;
  marker.color.g =   marker_control_config.resolution_ * 10;
  if(marker.color.g > 1.0){marker.color.g = 1.0;}
  marker.color.b =   marker_control_config.resolution_ * 10;
  if(marker.color.b > 1.0){marker.color.b = 1.0;}
  marker.color.a = 0.1;
  marker.header = feedback->header;
  return marker;
}

void PointCloudConfigMarker::publishMarkerMsg( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  visualization_msgs::Marker marker = makeMarkerMsg(feedback);
  marker.id = marker_control_config.marker_id++;
  pub_.publish(marker);
  latest_feedback_ = feedback;
}


void PointCloudConfigMarker::cancelCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  if(marker_control_config.marker_id == 0){
    return;
  }
  visualization_msgs::Marker marker = makeMarkerMsg(feedback);
  marker.id = --marker_control_config.marker_id;
  marker.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker);
  latest_feedback_ = feedback;
}


void PointCloudConfigMarker::clearBox(){
  marker_control_config.marker_id = 0;
  
  visualization_msgs::Marker marker = makeMarkerMsg(latest_feedback_);
  marker.id = -1;
  marker.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker);
}

void PointCloudConfigMarker::clearBoxCB( const std_msgs::Empty::ConstPtr &msg) {
  clearBox();
}

void PointCloudConfigMarker::clearCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  latest_feedback_ = feedback;
  clearBox();
}

void PointCloudConfigMarker::updateBoxInteractiveMarker(){
  visualization_msgs::InteractiveMarker boxIM = makeBoxInteractiveMarker(marker_control_config, marker_name);
  
  server_->insert(boxIM,
      boost::bind( &PointCloudConfigMarker::moveBoxCb, this, _1 ));
  menu_handler.apply(*server_, marker_name);
  server_->applyChanges();
}

void PointCloudConfigMarker::updatePoseCB(const geometry_msgs::PoseStamped::ConstPtr &pose) {
  //ROS_INFO("get new pose");
  server_->setPose(marker_name, pose->pose);
  server_->applyChanges();
  publishCurrentPose(pose);
  // manually update the latest_feedback_
  visualization_msgs::InteractiveMarkerFeedback::Ptr feedback(new visualization_msgs::InteractiveMarkerFeedback);
  feedback->pose = pose->pose;
  feedback->header = pose->header;
  latest_feedback_ = feedback;
}

void PointCloudConfigMarker::addBoxCB(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("add!");
  visualization_msgs::Marker marker = makeMarkerMsg(latest_feedback_);
  marker.id = marker_control_config.marker_id++;
  pub_.publish(marker);
}

PointCloudConfigMarker::PointCloudConfigMarker () : nh_(), pnh_("~") {
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("size", size_, 1.0 );
  pnh_.param("marker_name", marker_name, std::string ("point_cloud_config_marker") );
  pnh_.param("base_frame", base_frame, std::string("/pelvis") );

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name));
  
  pub_ =  pnh_.advertise<visualization_msgs::Marker> ("get", 1);
  current_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped> ("current_pose", 1);
  pose_update_sub_ = pnh_.subscribe("update_pose", 1, &PointCloudConfigMarker::updatePoseCB,
                                    this);
  add_box_sub_ = pnh_.subscribe("add_box", 1, &PointCloudConfigMarker::addBoxCB, this);
  clear_box_sub_ = pnh_.subscribe("clear_box", 1, &PointCloudConfigMarker::clearBoxCB, this);
  change_box_size_sub_ = pnh_.subscribe("change_size", 1, &PointCloudConfigMarker::changeBoxSize, this);
  change_box_resolution_sub_ = pnh_.subscribe("change_resolution", 1, &PointCloudConfigMarker::changeBoxResolution, this);  
  menu_handler = makeMenuHandler();
  
  marker_control_config = MarkerControlConfig(size_);

  updateBoxInteractiveMarker();
  

  /*
  updateBoxInter_markers::MenuHandler::EntryHandle sub_menu_move_;
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


