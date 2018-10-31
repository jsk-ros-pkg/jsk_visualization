#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <math.h>
#include <jsk_interactive_marker/MarkerMenu.h>
#include <jsk_interactive_marker/MarkerPose.h>

#include <std_msgs/Int8.h>

#include <jsk_interactive_marker/interactive_marker_interface.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

#include <dynamic_tf_publisher/SetDynamicTF.h>

#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace im_utils;

visualization_msgs::InteractiveMarker InteractiveMarkerInterface::make6DofControlMarker( std::string name, geometry_msgs::PoseStamped &stamped, float scale, bool fixed_position, bool fixed_rotation){
  
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
    
  //x axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  if(fixed_position){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }
    
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
    

  //y axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  if(fixed_position){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
    
  //z axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  if(fixed_position){
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
    
  return int_marker;
}



void InteractiveMarkerInterface::proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
  if(feedback->control_name.find("center_sphere") != std::string::npos){
    proc_feedback(feedback, jsk_interactive_marker::MarkerPose::SPHERE_MARKER);
  }else{
    proc_feedback(feedback, jsk_interactive_marker::MarkerPose::GENERAL);
  }
  
}

void InteractiveMarkerInterface::proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int type ) {

  jsk_interactive_marker::MarkerPose mp;
  mp.pose.header = feedback->header;
  mp.pose.pose = feedback->pose;
  mp.marker_name = feedback->marker_name;
  mp.type = type;
  pub_.publish( mp );

  //update Marker Pose Status
  control_state_.marker_pose_.pose = feedback->pose;
  control_state_.marker_pose_.header = feedback->header;

  pub_marker_tf(feedback->header, feedback->pose);

}

void InteractiveMarkerInterface::pub_marker_tf ( std_msgs::Header header, geometry_msgs::Pose pose){
  geometry_msgs::Transform tf;
  tf.translation.x = pose.position.x;
  tf.translation.y = pose.position.y;
  tf.translation.z = pose.position.z;
  tf.rotation = pose.orientation;
  
  dynamic_tf_publisher::SetDynamicTF SetTf;
  SetTf.request.freq = 10;
  SetTf.request.cur_tf.header.stamp = ros::Time::now();
  SetTf.request.cur_tf.header.frame_id = header.frame_id;
  SetTf.request.cur_tf.child_frame_id = "/moving_marker";
  SetTf.request.cur_tf.transform = tf;
  dynamic_tf_publisher_client_.call(SetTf);
}

void InteractiveMarkerInterface::pub_marker_pose ( std_msgs::Header header, geometry_msgs::Pose pose, std::string name, int type ) {
  jsk_interactive_marker::MarkerPose mp;
  mp.pose.header = header;
  mp.pose.pose = pose;
  mp.marker_name = name;
  mp.type = type;
  pub_.publish( mp );
}




void InteractiveMarkerInterface::pub_marker_menuCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu){
  jsk_interactive_marker::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu=menu;
  pub_move_.publish(m);
}

void InteractiveMarkerInterface::pub_marker_menuCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu, int type){
  jsk_interactive_marker::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu = menu;
  m.type = type;
  pub_move_.publish(m);
}


void InteractiveMarkerInterface::pub_marker_menu(std::string marker, int menu, int type){
  jsk_interactive_marker::MarkerMenu m;
  m.marker_name = marker;
  m.menu=menu;
  m.type = type;
  pub_move_.publish(m);
}

void InteractiveMarkerInterface::pub_marker_menu(std::string marker, int menu){
  pub_marker_menu(marker, menu, 0);
}

void InteractiveMarkerInterface::IMSizeLargeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.5, pose);
}

void InteractiveMarkerInterface::IMSizeMiddleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.3, pose);
}

void InteractiveMarkerInterface::IMSizeSmallCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.1, pose);
}

void InteractiveMarkerInterface::changeMoveModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,0);
  pub_marker_menu(feedback->marker_name,13);

}
void InteractiveMarkerInterface::changeMoveModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,1);
  pub_marker_menu(feedback->marker_name,13);

}
void InteractiveMarkerInterface::changeMoveModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,2);
  pub_marker_menu(feedback->marker_name,13);
}

void InteractiveMarkerInterface::changeForceModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("%s changeForceMode",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,0);
  pub_marker_menu(feedback->marker_name,12);

}
void InteractiveMarkerInterface::changeForceModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("%s changeForceMode1",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,1);
  pub_marker_menu(feedback->marker_name,12);

}
void InteractiveMarkerInterface::changeForceModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("%s changeForceMode2",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,2);
  pub_marker_menu(feedback->marker_name,12);
}

void InteractiveMarkerInterface::targetPointMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("targetPointMenu callback");

  control_state_.head_on_ ^= true;
  control_state_.init_head_goal_ = true;
  
  if(control_state_.head_on_){
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::CHECKED);
    control_state_.look_auto_on_ = false;
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::HEAD_TARGET_POINT, jsk_interactive_marker::MarkerMenu::HEAD_MARKER);
  }else{
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);
  }
  menu_head_.reApply(*server_);

  initControlMarkers();
}

void InteractiveMarkerInterface::lookAutomaticallyMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  ROS_INFO("targetPointMenu callback");

  control_state_.head_on_ = false;
  control_state_.look_auto_on_ ^= true;
  control_state_.init_head_goal_ = true;

  if(control_state_.look_auto_on_){
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::CHECKED);
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::PUBLISH_MARKER, jsk_interactive_marker::MarkerMenu::HEAD_MARKER);
  }else{
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::HEAD_TARGET_POINT, jsk_interactive_marker::MarkerMenu::HEAD_MARKER);
  }
  menu_head_.reApply(*server_);
  initControlMarkers();
}


void InteractiveMarkerInterface::ConstraintCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last2 = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::CHECKED );

  switch(h_mode_last2-h_mode_constrained){
  case 0:
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker::MarkerMenu::MOVE_CONSTRAINT_T);
    ROS_INFO("send 23");
    break;
  case 1:
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker::MarkerMenu::MOVE_CONSTRAINT_NIL);
    ROS_INFO("send 24");
    break;
  default:
    ROS_INFO("Switching Arm Error");
    break;
  }

  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::useTorsoCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(feedback->menu_entry_id == use_torso_t_menu_){
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::USE_TORSO_T);
  }else if(feedback->menu_entry_id == use_torso_nil_menu_){
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::USE_TORSO_NIL);
  }else{
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::CHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::USE_FULLBODY);
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::usingIKCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(feedback->menu_entry_id == start_ik_menu_){
    menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::PLAN);
  }else if(feedback->menu_entry_id == stop_ik_menu_){
    menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::CANCEL_PLAN);
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::toggleStartIKCb( const std_msgs::EmptyConstPtr &msg)
{
  interactive_markers::MenuHandler::CheckState check_state;
  if(menu_handler.getCheckState( start_ik_menu_ , check_state)){

    if(check_state == interactive_markers::MenuHandler::CHECKED){
      //stop ik
      menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
      pub_marker_menu("", jsk_interactive_marker::MarkerMenu::CANCEL_PLAN);

    }else{
      //start ik
      menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
      menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      pub_marker_menu("" , jsk_interactive_marker::MarkerMenu::PLAN);
    }

    menu_handler.reApply( *server_ );
    server_->applyChanges();
  }
}


void InteractiveMarkerInterface::modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );

  switch(h_mode_last - h_mode_rightarm){
  case 0:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker::MarkerMenu::SET_MOVE_RARM);
    break;
  case 1:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker::MarkerMenu::SET_MOVE_LARM);
    break;
  case 2:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker::MarkerMenu::SET_MOVE_ARMS);
    break;
  default:
    ROS_INFO("Switching Arm Error");
    break;
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();
}

void InteractiveMarkerInterface::changeMoveArm( std::string m_name, int menu ){
  switch(menu){
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_RARM:
    pub_marker_menu(m_name,jsk_interactive_marker::MarkerMenu::SET_MOVE_RARM);
    control_state_.move_arm_ = ControlState::RARM;
    ROS_INFO("move Rarm");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_LARM:
    pub_marker_menu(m_name,jsk_interactive_marker::MarkerMenu::SET_MOVE_LARM);
    control_state_.move_arm_ = ControlState::LARM;
    ROS_INFO("move Larm");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_ARMS:
    pub_marker_menu(m_name,jsk_interactive_marker::MarkerMenu::SET_MOVE_ARMS);
    control_state_.move_arm_ = ControlState::ARMS;
    ROS_INFO("move Arms");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  default:
    ROS_INFO("Switching Arm Error");
    break;
  }
}

void InteractiveMarkerInterface::setOriginCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,  bool origin_hand){
  if(origin_hand){
    control_state_.move_origin_state_ = ControlState::HAND_ORIGIN;
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    if(control_state_.move_arm_ == ControlState::RARM){
      pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::SET_ORIGIN_RHAND);}else{
      pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::SET_ORIGIN_LHAND);}
  }else{
    control_state_.move_origin_state_ = ControlState::DESIGNATED_ORIGIN;
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    pub_marker_menuCb(feedback, jsk_interactive_marker::MarkerMenu::SET_ORIGIN);
  }
  

}


void InteractiveMarkerInterface::ikmodeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  //menu_handler.setCheckState( h_mode_last3, interactive_markers::MenuHandler::UNCHECKED );
  //h_mode_last3 = feedback->menu_entry_id;
  //menu_handler.setCheckState( h_mode_last3, interactive_markers::MenuHandler::CHECKED );

  if(feedback->menu_entry_id == rotation_t_menu_){
    menu_handler.setCheckState( rotation_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_T);
    ROS_INFO("Rotation Axis T");
  }else{
    menu_handler.setCheckState( rotation_t_menu_, interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menu(feedback->marker_name ,jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_NIL);
    ROS_INFO("Rotation Axis NIL");
  }


  menu_handler.setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );

  menu_handler.reApply( *server_ );
  server_->applyChanges();
}


void InteractiveMarkerInterface::toggleIKModeCb( const std_msgs::EmptyConstPtr &msg)
{
  interactive_markers::MenuHandler::CheckState check_state;
  if(menu_handler.getCheckState( rotation_t_menu_ , check_state)){
    if(check_state == interactive_markers::MenuHandler::CHECKED){
      //rotation axis nil
      menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      menu_handler.setCheckState( rotation_nil_menu_ , interactive_markers::MenuHandler::CHECKED );
      pub_marker_menu("", jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_NIL);

    }else{
      //rotation_axis t
      menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::CHECKED );
      menu_handler.setCheckState( rotation_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      pub_marker_menu("" , jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_T);
    }

    menu_handler.reApply( *server_ );
    server_->applyChanges();
  }
}

void InteractiveMarkerInterface::marker_menu_cb( const jsk_interactive_marker::MarkerMenuConstPtr &msg){
  switch (msg->menu){
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_RARM:
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_LARM:
  case jsk_interactive_marker::MarkerMenu::SET_MOVE_ARMS:
    changeMoveArm(msg->marker_name, msg->menu);
    break;
  case jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_NIL:
  case jsk_interactive_marker::MarkerMenu::IK_ROTATION_AXIS_T:
    {
      std_msgs::EmptyConstPtr empty;
      toggleIKModeCb(empty);
    }
    break;
  case jsk_interactive_marker::MarkerMenu::PLAN:
  case jsk_interactive_marker::MarkerMenu::CANCEL_PLAN:
    {
      std_msgs::EmptyConstPtr empty;
      toggleStartIKCb(empty);
    }
    break;
  default:
    pub_marker_menu(msg->marker_name , msg->menu, msg->type);
    break;
  }
}


void InteractiveMarkerInterface::updateHeadGoal( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ros::Time now = ros::Time(0);

  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM(    "Marker " << feedback->marker_name
			  << " control " << feedback->control_name
			  << " menu_entry_id " << feedback->menu_entry_id);
      break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      //proc_feedback(feedback, jsk_interactive_marker::MarkerPose::HEAD_MARKER);
      break;
    }
}

void InteractiveMarkerInterface::updateBase( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      proc_feedback(feedback, jsk_interactive_marker::MarkerPose::BASE_MARKER);
      break;
    }
}

void InteractiveMarkerInterface::updateFinger( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string hand)
{
  ros::Time now = ros::Time(0);

  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if(hand == "rhand"){
	control_state_.r_finger_on_ ^= true;
      }else if(hand == "lhand"){
	control_state_.l_finger_on_ ^= true;
      }
      initControlMarkers();
      ROS_INFO_STREAM( hand << feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( hand << feedback->marker_name << " was clicked on." );
      if(hand == "rhand"){
	proc_feedback(feedback, jsk_interactive_marker::MarkerPose::RFINGER_MARKER);
      }
      if(hand == "lhand"){
	proc_feedback(feedback, jsk_interactive_marker::MarkerPose::LFINGER_MARKER);
      }

      //proc_feedback(feedback);
      break;
    }
}


//im_mode
//0:normal move  1:operationModel 2:operationalModelFirst
void InteractiveMarkerInterface::changeMarkerForceMode( std::string mk_name , int im_mode){
  ROS_INFO("changeMarkerForceMode  marker:%s  mode:%d\n",mk_name.c_str(),im_mode);
  interactive_markers::MenuHandler reset_handler;
  menu_handler_force = reset_handler;
  menu_handler_force1 = reset_handler;
  menu_handler_force2 = reset_handler;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = base_frame;
  if ( target_frame != "" ) {
    /*
      tf::StampedTransform stf;
      geometry_msgs::TransformStamped mtf;
      tfl_.lookupTransform(target_frame, base_frame,
      ros::Time(0), stf);
      tf::transformStampedTFToMsg(stf, mtf);
      pose.pose.position.x = mtf.transform.translation.x;
      pose.pose.position.y = mtf.transform.translation.y;
      pose.pose.position.z = mtf.transform.translation.z;
      pose.pose.orientation = mtf.transform.rotation;
      pose.header = mtf.header;
    */
  }
  visualization_msgs::InteractiveMarker mk;
  //    mk.name = marker_name.c_str();
  mk.name = mk_name.c_str();
  mk.scale = 0.5;
  mk.header = pose.header;
  mk.pose = pose.pose;

  // visualization_msgs::InteractiveMarker mk =
  //   im_helpers::make6DofMarker(marker_name.c_str(), pose, 0.5,
  //                              true, false );
  visualization_msgs::InteractiveMarkerControl control;
    
  if ( false )
    {
      //int_marker.name += "_fixed";
      //int_marker.description += "\n(fixed orientation)";
      control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);
  // control.name = "move_z";
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  // mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);
  // control.name = "move_y";
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  // mk.controls.push_back(control);

  //add furuta
  switch(im_mode){
  case 0:
    menu_handler_force.insert("MoveMode",boost::bind( &InteractiveMarkerInterface::changeMoveModeCb, this, _1));
    menu_handler_force.insert("Delete Force",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::DELETE_FORCE));
    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler_force.apply(*server_,mk.name);
      
    server_->applyChanges();
    break;
  case 1:
    mk.scale = 0.5;
    menu_handler_force1.insert("MoveMode",boost::bind( &InteractiveMarkerInterface::changeMoveModeCb1, this, _1));
    menu_handler_force1.insert("Delete Force",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::DELETE_FORCE));
    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler_force1.apply(*server_,mk.name);
    
    server_->applyChanges();
    break;
  case 2:
    mk.scale = 0.5;
    menu_handler_force2.insert("MoveMode",boost::bind( &InteractiveMarkerInterface::changeMoveModeCb2, this, _1));
    menu_handler_force2.insert("Delete Force",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::DELETE_FORCE));
    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler_force2.apply(*server_,mk.name);
 
    server_->applyChanges();
    break;
  default:
    break;
  }


  //menu_handler_force.insert("ResetForce",boost::bind( &InteractiveMarkerInterface::resetForceCb, this, _1));

  std::list<visualization_msgs::InteractiveMarker>::iterator it = imlist.begin();

  while( it != imlist.end() )  // listの末尾まで
    {
      if(it->name == mk_name.c_str()){
	imlist.erase(it);
	break;
      }
      it++;
    }
  imlist.push_back( mk );

  /*
    it = imlist.begin();
    while( it != imlist.end() )  // listの末尾まで
    {
    server_->insert( *it );
	
    server_->setCallback( it->name,
    boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
	
    menu_handler_force.apply(*server_,it->name);
    it++;
    }*/
  ROS_INFO("add mk");
  /* add mk */

}

void InteractiveMarkerInterface::initBodyMarkers(void){
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time(0);

  double scale_factor = 1.02;

  //for head
  ps.header.frame_id = head_link_frame_;
  visualization_msgs::InteractiveMarker im =
    im_helpers::makeMeshMarker(head_link_frame_, head_mesh_, ps, scale_factor);
  makeIMVisible(im);
  server_->insert(im);
  menu_head_.apply(*server_, head_link_frame_);


  if(hand_type_ == "sandia_hand"){
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time(0);

    ps.header.frame_id = "/right_f0_base";
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
	visualization_msgs::InteractiveMarker fingerIm = 
	  makeSandiaHandInteractiveMarker(ps, "right", i, j);
	makeIMVisible(fingerIm);
	server_->insert(fingerIm, boost::bind( &InteractiveMarkerInterface::updateFinger, this, _1, "rhand"));
      }
    }

    ps.header.frame_id = "/left_f0_base";
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
	visualization_msgs::InteractiveMarker fingerIm = 
	  makeSandiaHandInteractiveMarker(ps, "left", i, j);
	makeIMVisible(fingerIm);
	server_->insert(fingerIm, boost::bind( &InteractiveMarkerInterface::updateFinger, this, _1, "lhand"));
      }
    }

  }else{
    //for right hand
    for(int i=0; i<rhand_mesh_.size(); i++){
      ps.header.frame_id = rhand_mesh_[i].link_name;
      ps.pose.orientation = rhand_mesh_[i].orientation;
      visualization_msgs::InteractiveMarker handIm =
	im_helpers::makeMeshMarker( rhand_mesh_[i].link_name, rhand_mesh_[i].mesh_file, ps, scale_factor);
      makeIMVisible(handIm);
      server_->insert(handIm);
    }
  }
}


void InteractiveMarkerInterface::initControlMarkers(void){
  //Head Marker
  if(control_state_.head_on_ && control_state_.init_head_goal_){
    control_state_.init_head_goal_ = false;
    head_goal_pose_.header.stamp = ros::Time(0);
    
    visualization_msgs::InteractiveMarker HeadGoalIm =
      im_helpers::makeHeadGoalMarker( "head_point_goal", head_goal_pose_, 0.1);
    makeIMVisible(HeadGoalIm);
    server_->insert(HeadGoalIm,
		    boost::bind( &InteractiveMarkerInterface::updateHeadGoal, this, _1));
    menu_head_target_.apply(*server_,"head_point_goal");
  }
  if(!control_state_.head_on_){
    server_->erase("head_point_goal");
  }

  //Base Marker
  if(control_state_.base_on_ ){
    geometry_msgs::PoseStamped ps;
    ps.pose.orientation.w = 1;
    ps.header.frame_id = move_base_frame;
    ps.header.stamp = ros::Time(0);
    visualization_msgs::InteractiveMarker baseIm =
      InteractiveMarkerInterface::makeBaseMarker( "base_control", ps, 0.75, false);
    makeIMVisible(baseIm);
    server_->insert(baseIm,
		    boost::bind( &InteractiveMarkerInterface::updateBase, this, _1 ));

    menu_base_.apply(*server_,"base_control");
  }else{
    server_->erase("base_control");
  }

  //finger Control Marker
  if(use_finger_marker_ && control_state_.r_finger_on_){
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time(0);
    ps.header.frame_id = "/right_f0_base";

    server_->insert(makeFingerControlMarker("right_finger", ps),
		    boost::bind( &InteractiveMarkerInterface::updateFinger, this, _1, "rhand"));
    menu_finger_r_.apply(*server_,"right_finger");
  }else{
    server_->erase("right_finger");
  }

  if(use_finger_marker_ && control_state_.l_finger_on_){
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time(0);
    ps.header.frame_id = "/left_f0_base";

    server_->insert(makeFingerControlMarker("left_finger", ps),
		    boost::bind( &InteractiveMarkerInterface::updateFinger, this, _1, "lhand"));
    menu_finger_l_.apply(*server_,"left_finger");
  }else{
    server_->erase("left_finger");
  }

  server_->applyChanges();
}


visualization_msgs::InteractiveMarker InteractiveMarkerInterface::makeBaseMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed)
{
  visualization_msgs::InteractiveMarker mk;
  mk.header =  stamped.header;
  mk.name = name;
  mk.scale = scale;
  mk.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;

  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);
  return mk;

}



void InteractiveMarkerInterface::initHandler(void){
  //use_arm=2;

  bool use_menu;
  pnh_.param("force_mode_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("ForceMode",boost::bind( &InteractiveMarkerInterface::changeForceModeCb, this, _1));
  }
    
  pnh_.param("move_menu", use_menu, false );
  if(use_menu){
    pnh_.param("move_safety_menu", use_menu, false );
    if(use_menu){
      interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
      sub_menu_move_ = menu_handler.insert( "Move" );
      menu_handler.insert( sub_menu_move_,"Plan",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::PLAN));
      menu_handler.insert( sub_menu_move_,"Execute",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::EXECUTE));
      menu_handler.insert( sub_menu_move_,"Plan And Execute",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::PLAN_EXECUTE));
      menu_handler.insert( sub_menu_move_,"Cancel", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::CANCEL_PLAN));
    }else{
      menu_handler.insert("Move",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE));
    }
  }

  pnh_.param("change_using_ik_menu", use_menu, false );
  if(use_menu){
    interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
    sub_menu_move_ = menu_handler.insert( "Whether To Use IK" );
    start_ik_menu_ = menu_handler.insert( sub_menu_move_,"Start IK",boost::bind( &InteractiveMarkerInterface::usingIKCb, this, _1));
    menu_handler.setCheckState( start_ik_menu_, interactive_markers::MenuHandler::CHECKED );

    stop_ik_menu_ = menu_handler.insert( sub_menu_move_,"Stop IK",boost::bind( &InteractiveMarkerInterface::usingIKCb, this, _1));
    menu_handler.setCheckState( stop_ik_menu_, interactive_markers::MenuHandler::UNCHECKED );
  }

  //menu_handler.insert("Touch It", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::TOUCH));
  pnh_.param("touch_it_menu", use_menu, false );
  if(use_menu){

    interactive_markers::MenuHandler::EntryHandle sub_menu_handle_touch_it;
    sub_menu_handle_touch_it = menu_handler.insert( "Touch It" );

    //  menu_handler.insert( sub_menu_handle_touch_it, "Preview", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::TOUCHIT_PREV));
    menu_handler.insert( sub_menu_handle_touch_it, "Execute", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::TOUCHIT_EXEC));
    menu_handler.insert( sub_menu_handle_touch_it, "Cancel", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::TOUCHIT_CANCEL));
  }
  pnh_.param("look_hand_menu", use_menu, false );
  if(use_menu){


    interactive_markers::MenuHandler::EntryHandle sub_menu_handle_look_hand;
    sub_menu_handle_look_hand = menu_handler.insert( "Look hand" );

    menu_handler.insert( sub_menu_handle_look_hand, "rarm", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::LOOK_RARM));
    menu_handler.insert( sub_menu_handle_look_hand, "larm", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::LOOK_LARM));
  }

  pnh_.param("force_move_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Force Move", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::FORCE_MOVE));
  }

  pnh_.param("pick_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Pick", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::PICK));
  }

  pnh_.param("grasp_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Grasp", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::START_GRASP));
  }

  pnh_.param("harf_grasp_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Harf Grasp", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::HARF_GRASP));
  }


  pnh_.param("stop_grasp_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Stop Grasp", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::STOP_GRASP));
  }

  pnh_.param("set_origin_menu", use_menu, false );
  if(use_menu){
    //menu_handler.insert("Set Origin To Hand", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::SET_ORIGIN));
    menu_handler.insert("Set Origin To Hand", boost::bind( &InteractiveMarkerInterface::setOriginCb, this, _1, true));

    menu_handler.insert("Set Origin", boost::bind( &InteractiveMarkerInterface::setOriginCb, this, _1, false));
  }

  /*
  pnh_.param("set_origin_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Set Origin", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::SET_ORIGIN));
  }

  pnh_.param("set_origin_to_rhand_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Set Origin To RHand", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::SET_ORIGIN_RHAND));
  }

  pnh_.param("set_origin_to_lhand_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Set Origin To LHand", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::SET_ORIGIN_LHAND));
  }
  */

  pnh_.param("reset_marker_pos_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Reset Marker Position", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::RESET_COORDS));
  }

  pnh_.param("manipulation_mode_menu", use_menu, false );
  if(use_menu){
    menu_handler.insert("Manipulation Mode", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MANIP_MODE));
  }

  //    menu_handler.insert("ResetForce",boost::bind( &InteractiveMarkerInterface::resetForceCb, this, _1));
    
  //menu_handler.insert("OperationModel",boost::bind( &InteractiveMarkerInterface::AutoMoveCb, this, _1));
    
  //    menu_handler.insert("StartTeaching",boost::bind( &InteractiveMarkerInterface::StartTeachingCb, this, _1));


  /*    sub_menu_handle2 = menu_handler.insert( "Constraint" );
    
	h_mode_last2 = menu_handler.insert( sub_menu_handle2, "constrained", boost::bind( &InteractiveMarkerInterface::ConstraintCb,this, _1 ));
	menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::UNCHECKED );
	h_mode_constrained = h_mode_last2;
	h_mode_last2 = menu_handler.insert( sub_menu_handle2, "unconstrained", boost::bind( &InteractiveMarkerInterface::ConstraintCb,this, _1 ));
	menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::CHECKED );
  */
    
  //    menu_handler.insert("StopTeaching",boost::bind( &InteractiveMarkerInterface::StopTeachingCb, this, _1));
  //menu_handler.setCheckState(menu_handler.insert("SetForce",boost::bind( &InteractiveMarkerInterface::enableCb, this, _1)),interactive_markers::MenuHandler::UNCHECKED);
    

  pnh_.param("select_arm_menu", use_menu, false );
  if(use_menu){
    sub_menu_handle = menu_handler.insert( "SelectArm" );
    h_mode_last = menu_handler.insert( sub_menu_handle, "Right Arm", boost::bind( &InteractiveMarkerInterface::modeCb,this, _1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );
    h_mode_rightarm = h_mode_last;
    h_mode_last = menu_handler.insert( sub_menu_handle, "Left Arm", boost::bind( &InteractiveMarkerInterface::modeCb,this, _1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
    h_mode_last = menu_handler.insert( sub_menu_handle, "Both Arms", boost::bind( &InteractiveMarkerInterface::modeCb,this, _1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
    h_mode_last = h_mode_rightarm;
  }

  pnh_.param("ik_mode_menu", use_menu, false );
  if(use_menu){
    sub_menu_handle_ik = menu_handler.insert( "IK mode" );

    rotation_t_menu_ = menu_handler.insert( sub_menu_handle_ik, "6D (Position + Rotation)", boost::bind( &InteractiveMarkerInterface::ikmodeCb,this, _1 ));
    menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::CHECKED );
    rotation_nil_menu_ = menu_handler.insert( sub_menu_handle_ik, "3D (Position)", boost::bind( &InteractiveMarkerInterface::ikmodeCb,this, _1 ));
    menu_handler.setCheckState( rotation_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
  }

  pnh_.param("use_torso_menu", use_menu, false );
  if(use_menu){
    use_torso_menu_ = menu_handler.insert( "Links To Use" );

    use_torso_nil_menu_ = menu_handler.insert( use_torso_menu_, "Arm", boost::bind( &InteractiveMarkerInterface::useTorsoCb,this, _1 ));
    menu_handler.setCheckState( use_torso_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
    use_torso_t_menu_ = menu_handler.insert( use_torso_menu_, "Arm and Torso", boost::bind( &InteractiveMarkerInterface::useTorsoCb,this, _1 ));
    menu_handler.setCheckState( use_torso_t_menu_, interactive_markers::MenuHandler::UNCHECKED );
    use_fullbody_menu_ = menu_handler.insert( use_torso_menu_, "Fullbody", boost::bind( &InteractiveMarkerInterface::useTorsoCb,this, _1 ));
    menu_handler.setCheckState( use_fullbody_menu_, interactive_markers::MenuHandler::CHECKED );

  }



  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_im_size;
  sub_menu_handle_im_size = menu_handler.insert( "IMsize" );
  menu_handler.insert( sub_menu_handle_im_size, "Large", boost::bind( &InteractiveMarkerInterface::IMSizeLargeCb, this, _1));
  menu_handler.insert( sub_menu_handle_im_size, "Middle", boost::bind( &InteractiveMarkerInterface::IMSizeMiddleCb, this, _1));
  menu_handler.insert( sub_menu_handle_im_size, "Small", boost::bind( &InteractiveMarkerInterface::IMSizeSmallCb, this, _1));

  pnh_.param("publish_marker_menu", use_menu, false );
  if(use_menu){
    //menu_handler.insert("ManipulationMode", boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MANIP_MODE));
    menu_handler.insert("Publish Marker",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::PUBLISH_MARKER));

  }




  //--------- menu_handler 1 ---------------
  menu_handler1.insert("ForceMode",boost::bind( &InteractiveMarkerInterface::changeForceModeCb1, this, _1));

  //--------- menu_handler 2 ---------------
  menu_handler2.insert("ForceMode",boost::bind( &InteractiveMarkerInterface::changeForceModeCb2, this, _1));
  menu_handler2.insert("Move",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE));


  /* porting from PR2 marker control */
  /* head marker */

  //menu_head_.insert("Take Snapshot", boost::bind( &InteractiveMarkerInterface::snapshotCB, this ) );



  head_target_handle_ = menu_head_.insert( "Target Point", 
					   boost::bind( &InteractiveMarkerInterface::targetPointMenuCB, this, _1 ) );
  menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);

  head_auto_look_handle_ = menu_head_.insert( "Look Automatically", boost::bind( &InteractiveMarkerInterface::lookAutomaticallyMenuCB,
										 this, _1 ) );
  menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::CHECKED);
  
  menu_head_target_.insert( "Look At",
			    boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1, jsk_interactive_marker::MarkerPose::HEAD_MARKER));
  //boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE, jsk_interactive_marker::MarkerMenu::HEAD_MARKER));


  /*
    projector_handle_ = menu_head_.insert("Projector", boost::bind( &InteractiveMarkerInterface::projectorMenuCB,
    this, _1 ) );
    menu_head_.setCheckState(projector_handle_, MenuHandler::UNCHECKED);
  */
    
  /*
    menu_head_.insert( "Move Head To Center", boost::bind( &InteractiveMarkerInterface::centerHeadCB,
    this ) );
  */


  /* base move menu*/
  pnh_.param("use_base_marker", use_menu, false );
  control_state_.base_on_ = use_menu;

  menu_base_.insert("Base Move",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE, jsk_interactive_marker::MarkerMenu::BASE_MARKER));
  menu_base_.insert("Reset Marker Position",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::RESET_COORDS, jsk_interactive_marker::MarkerMenu::BASE_MARKER));

  /*finger move menu*/
  pnh_.param("use_finger_marker", use_finger_marker_, false );

  menu_finger_r_.insert("Move Finger",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE, jsk_interactive_marker::MarkerMenu::RFINGER_MARKER));
  menu_finger_r_.insert("Reset Marker",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::RESET_COORDS, jsk_interactive_marker::MarkerMenu::RFINGER_MARKER));

  menu_finger_l_.insert("Move Finger",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::MOVE, jsk_interactive_marker::MarkerMenu::LFINGER_MARKER));
  menu_finger_l_.insert("Reset Marker",boost::bind( &InteractiveMarkerInterface::pub_marker_menuCb, this, _1, jsk_interactive_marker::MarkerMenu::RESET_COORDS, jsk_interactive_marker::MarkerMenu::LFINGER_MARKER));

}

void InteractiveMarkerInterface::addHandMarker(visualization_msgs::InteractiveMarker &im,std::vector < UrdfProperty > urdf_vec){
  if(urdf_vec.size() > 0){
    for(int i=0; i<urdf_vec.size(); i++){
      UrdfProperty up = urdf_vec[i];
      if(up.model){
        KDL::Frame origin_frame;
        tf::poseMsgToKDL(up.pose, origin_frame);

        LinkConstSharedPtr hand_root_link;
        hand_root_link = up.model->getLink(up.root_link_name);
        if(!hand_root_link){
          hand_root_link = up.model->getRoot();
        }
        im_utils::addMeshLinksControl(im, hand_root_link, origin_frame, !up.use_original_color, up.color, up.scale);
        for(int j=0; j<im.controls.size(); j++){
          if(im.controls[j].interaction_mode == visualization_msgs::InteractiveMarkerControl::BUTTON){
            im.controls[j].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
            im.controls[j].name = "center_sphere";
          }
        }
      }else{
        addSphereMarker(im, up.scale, up.color);
      }
    }
  }else{
    double center_marker_size = 0.2;
    //gray
    std_msgs::ColorRGBA color;
    color.r = color.g = color.b = 0.7;
    color.a = 0.5;
    addSphereMarker(im, center_marker_size, color);
  }
}

void InteractiveMarkerInterface::addSphereMarker(visualization_msgs::InteractiveMarker &im, double scale, std_msgs::ColorRGBA color){
    visualization_msgs::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;

    sphereMarker.scale.x = scale;
    sphereMarker.scale.y = scale;
    sphereMarker.scale.z = scale;

    sphereMarker.color = color;

    visualization_msgs::InteractiveMarkerControl sphereControl;
    sphereControl.name = "center_sphere";

    sphereControl.markers.push_back(sphereMarker);
    sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    im.controls.push_back(sphereControl);
}


void InteractiveMarkerInterface::makeCenterSphere(visualization_msgs::InteractiveMarker &mk, double mk_size){
  std::vector < UrdfProperty > null_urdf;
  if(control_state_.move_origin_state_ == ControlState::HAND_ORIGIN){
    if(control_state_.move_arm_ == ControlState::RARM){
      addHandMarker(mk, rhand_urdf_);
    }else if(control_state_.move_arm_ == ControlState::LARM){
      addHandMarker(mk, lhand_urdf_);
    }else{
      addHandMarker(mk, null_urdf);
    }
  }else{
    addHandMarker(mk, null_urdf);
  }

  //sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  //mk.controls.push_back(sphereControl);
}

//im_mode
//0:normal move  1:operationModel 2:operationalModelFirst
void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode){
  switch(im_mode){
  case 0:
    changeMarkerMoveMode( mk_name, im_mode , 0.5);
    break;
  case 1:
  case 2:
    changeMarkerMoveMode( mk_name, im_mode , 0.3);
    break;
  default:
    changeMarkerMoveMode( mk_name, im_mode , 0.3);
    break;
  }
}

void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size){
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = base_frame;
  pose.pose.orientation.w = 1.0;
  changeMarkerMoveMode( mk_name, im_mode , mk_size, pose);
}

void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size, geometry_msgs::PoseStamped dist_pose){
  ROS_INFO("changeMarkerMoveMode  marker:%s  mode:%d\n",mk_name.c_str(),im_mode);
  
  control_state_.marker_pose_ = dist_pose;

  interactive_markers::MenuHandler reset_handler;

  geometry_msgs::PoseStamped pose;

  if ( target_frame != "" ) {
    /*
      tf::StampedTransform stf;
      geometry_msgs::TransformStamped mtf;
      tfl_.lookupTransform(target_frame, base_frame,
      ros::Time(0), stf);
      tf::transformStampedTFToMsg(stf, mtf);
      pose.pose.position.x = mtf.transform.translation.x;
      pose.pose.position.y = mtf.transform.translation.y;
      pose.pose.position.z = mtf.transform.translation.z;
      pose.pose.orientation = mtf.transform.rotation;
      pose.header = mtf.header;
    */
  }else{
    pose = dist_pose;
  }

  visualization_msgs::InteractiveMarker mk;
  //0:normal move  1:operationModel 2:operationalModelFirst

  switch(im_mode){
  case 0:
    pose.header.stamp = ros::Time(0);

    mk = make6DofControlMarker(mk_name.c_str(), pose, mk_size,
			       true, false );

    if(use_center_sphere_){
      makeCenterSphere(mk, mk_size);
    }

    makeIMVisible(mk);

    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler.apply(*server_,mk.name);
    server_->applyChanges();
    break;
  case 1:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
				    true, false );
    mk.description = mk_name.c_str();
    makeIMVisible(mk);
    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler1.apply(*server_,mk.name);
    server_->applyChanges();
    break;
      
  case 2:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
				    true, false );
    mk.description = mk_name.c_str();
    makeIMVisible(mk);
      
    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    menu_handler2.apply(*server_,mk.name);
    server_->applyChanges();
    break;
  default:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
				    true, false );
    mk.description = mk_name.c_str();

    server_->insert( mk );
    server_->setCallback( mk.name,
			  boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
    server_->applyChanges();
    break;
  }

  std::list<visualization_msgs::InteractiveMarker>::iterator it = imlist.begin();

  while( it != imlist.end() )
    {
      if(it->name == mk_name.c_str()){
	imlist.erase(it);
	break;
      }
      it++;
    }
  imlist.push_back( mk );
}

void InteractiveMarkerInterface::changeMarkerOperationModelMode( std::string mk_name ){
  interactive_markers::MenuHandler reset_handler;
  menu_handler = reset_handler;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = base_frame;
  /*
    if ( target_frame != "" ) {
    tf::StampedTransform stf;
    geometry_msgs::TransformStamped mtf;
    tfl_.lookupTransform(target_frame, base_frame,
    ros::Time(0), stf);
    tf::transformStampedTFToMsg(stf, mtf);
    pose.pose.position.x = mtf.transform.translation.x;
    pose.pose.position.y = mtf.transform.translation.y;
    pose.pose.position.z = mtf.transform.translation.z;
    pose.pose.orientation = mtf.transform.rotation;
    pose.header = mtf.header;
    }*/

  visualization_msgs::InteractiveMarker mk =
    
    im_helpers::make6DofMarker(mk_name.c_str(), pose, 0.5,
			       true, false );
  mk.description = mk_name.c_str();
  menu_handler.insert("ForceMode",boost::bind( &InteractiveMarkerInterface::changeForceModeCb, this, _1));

  std::list<visualization_msgs::InteractiveMarker>::iterator it = imlist.begin(); // イテレータ

  while( it != imlist.end() )  // listの末尾まで
    {
      if(it->name == mk_name.c_str()){
	imlist.erase(it);
	break;
      }
      it++;
    }
  imlist.push_back( mk );
  server_->insert( mk );
     
  server_->setCallback( mk.name,
			boost::bind( &InteractiveMarkerInterface::proc_feedback, this, _1) );
     
  menu_handler.apply(*server_,mk.name);
  server_->applyChanges();
}


//InteractiveMarkerInterface::InteractiveMarkerInterface () : nh_(), pnh_("~"), tfl_(nh_) {
InteractiveMarkerInterface::InteractiveMarkerInterface () : nh_(), pnh_("~") {
  pnh_.param("marker_name", marker_name, std::string ( "100") );
  pnh_.param("server_name", server_name, std::string ("") );
  pnh_.param("base_frame", base_frame, std::string ("/base_link") );
  pnh_.param("move_base_frame", move_base_frame, std::string ("/base_link") );
  pnh_.param("target_frame", target_frame, std::string ("") );
  //pnh_.param("fix_marker", fix_marker, true);

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }

  pub_ =  pnh_.advertise<jsk_interactive_marker::MarkerPose> ("pose", 1);
  pub_update_ =  pnh_.advertise<geometry_msgs::PoseStamped> ("pose_update", 1);
  pub_move_ =  pnh_.advertise<jsk_interactive_marker::MarkerMenu> ("marker_menu", 1);

  serv_set_ = pnh_.advertiseService("set_pose",
				    &InteractiveMarkerInterface::set_cb, this);
  serv_markers_set_ = pnh_.advertiseService("set_markers",
					    &InteractiveMarkerInterface::markers_set_cb, this);
  serv_markers_del_ = pnh_.advertiseService("del_markers",
					    &InteractiveMarkerInterface::markers_del_cb, this);
  serv_reset_ = pnh_.advertiseService("reset_pose",
				      &InteractiveMarkerInterface::reset_cb, this);

  sub_marker_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped> ("move_marker", 1, boost::bind( &InteractiveMarkerInterface::move_marker_cb, this, _1));
  sub_marker_menu_ = pnh_.subscribe<jsk_interactive_marker::MarkerMenu> ("select_marker_menu", 1, boost::bind( &InteractiveMarkerInterface::marker_menu_cb, this, _1));

  sub_toggle_start_ik_ = pnh_.subscribe<std_msgs::Empty> ("toggle_start_ik", 1, boost::bind( &InteractiveMarkerInterface::toggleStartIKCb, this, _1));
  
  sub_toggle_ik_mode_ = pnh_.subscribe<std_msgs::Empty> ("toggle_ik_mode", 1, boost::bind( &InteractiveMarkerInterface::toggleIKModeCb, this, _1));

  ros::service::waitForService("set_dynamic_tf", -1);
  dynamic_tf_publisher_client_ = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf", true);

  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name));

  pnh_.param<std::string>("head_link_frame", head_link_frame_, "head_tilt_link");
  pnh_.param<std::string>("head_mesh", head_mesh_, "package://pr2_description/meshes/head_v0/head_tilt.dae");

  pnh_.param<std::string>("hand_type", hand_type_, "GENERIC");

  pnh_.param("use_head_marker", use_body_marker_, false );
  pnh_.param("use_center_sphere", use_center_sphere_, false );

  XmlRpc::XmlRpcValue v;
  pnh_.param("mesh_config", v, v);
  loadMeshes(v);

  head_goal_pose_.pose.position.x = 1.0;
  head_goal_pose_.pose.position.z = 1.0;
  head_goal_pose_.header.frame_id = base_frame;

  initHandler();
  if(use_body_marker_){
    initBodyMarkers();
  }
  initControlMarkers();
  changeMarkerMoveMode(marker_name.c_str(),0);
}

void InteractiveMarkerInterface::loadMeshes(XmlRpc::XmlRpcValue val){
  loadUrdfFromYaml(val, "r_hand", rhand_urdf_);
  loadUrdfFromYaml(val, "l_hand", lhand_urdf_);
}

void InteractiveMarkerInterface::loadUrdfFromYaml(XmlRpc::XmlRpcValue val, std::string name, std::vector<UrdfProperty>& mesh){
  if(val.hasMember(name)){
    for(int i=0; i< val[name].size(); i++){
      XmlRpc::XmlRpcValue nval = val[name][i];
      UrdfProperty up;
      //urdf file
      if(nval.hasMember("urdf_file")){
        std::string urdf_file = (std::string)nval["urdf_file"];
        std::cerr << "load urdf file: " << urdf_file << std::endl;
        up.model = im_utils::getModelInterface(urdf_file);
      }else if(nval.hasMember("urdf_param")){
        std::string urdf_param = (std::string)nval["urdf_param"];
	std::string urdf_model;
	nh_.getParam(urdf_param, urdf_model);
	up.model = parseURDF(urdf_model);
      }

      if(nval.hasMember("root_link")){
        std::string root_link_name = (std::string)nval["root_link"];
        std::cerr << "root link name: " << root_link_name << std::endl;
        up.root_link_name = root_link_name;
      }else{
        up.root_link_name = "";
      }

      up.pose.orientation.w = 1.0;
      //pose
      if(nval.hasMember("pose")){
        XmlRpc::XmlRpcValue pose = nval["pose"];
        if(pose.hasMember("position")){
          XmlRpc::XmlRpcValue position = pose["position"];
          up.pose.position.x = (double)position["x"];
          up.pose.position.y = (double)position["y"];
          up.pose.position.z = (double)position["z"];
        }

        if(pose.hasMember("orientation")){
          XmlRpc::XmlRpcValue orient = pose["orientation"];
          up.pose.orientation.x = (double)orient["x"];
          up.pose.orientation.y = (double)orient["y"];
          up.pose.orientation.z = (double)orient["z"];
          up.pose.orientation.w = (double)orient["w"];
        }
      }

      if(nval.hasMember("color")){
        XmlRpc::XmlRpcValue color = nval["color"];
        up.color.r = (double)color["r"];
        up.color.g = (double)color["g"];
        up.color.b = (double)color["b"];
        up.color.a = (double)color["a"];
      }else{
        up.color.r = 1.0;
        up.color.g = 1.0;
        up.color.b = 0.0;
        up.color.a = 0.7;
      }
      if(nval.hasMember("scale")){
        up.scale = (double)nval["scale"];
      }else{
        up.scale = 1.05; //make bigger a bit
      }
      mesh.push_back(up);
    }
  }
}


bool InteractiveMarkerInterface::markers_set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
						  jsk_interactive_marker::MarkerSetPose::Response &res ) {
  bool setalready = false;

  std::list<visualization_msgs::InteractiveMarker>::iterator it = imlist.begin();
  while( it != imlist.end() )  // listの末尾まで
    {
      if( it->name == req.marker_name){
	setalready = true;
	break;
      }
      it++;
    }

  if(setalready){
    server_->setPose(req.marker_name, req.pose.pose, req.pose.header);
    server_->applyChanges();
    return true;
  }else{
    /*
      if(req.marker_name==0){
      changeMarkerMoveMode(name,2);
      }else{
      changeMarkerMoveMode(name,1);
      }
      server_->setPose(name, req.pose.pose, req.pose.header);
      //    menu_handler.apply(*server_,mk.name)Z
      server_->applyChanges();
      return true;
    */
    return true;
  }
}
  
bool InteractiveMarkerInterface::markers_del_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
						  jsk_interactive_marker::MarkerSetPose::Response &res ) {
  
  server_->erase(req.marker_name);
  server_->applyChanges();
  std::list<visualization_msgs::InteractiveMarker>::iterator it = imlist.begin();
  while( it != imlist.end() )  // listの末尾まで
    {
      if( it->name == req.marker_name){
	imlist.erase(it);
	break;
      }
      it++;
    }
  
  return true;
    
}

void InteractiveMarkerInterface::move_marker_cb ( const geometry_msgs::PoseStampedConstPtr &msg){
  pub_marker_tf(msg->header, msg->pose);

  pub_marker_pose( msg->header, msg->pose, marker_name, jsk_interactive_marker::MarkerPose::GENERAL);

  server_->setPose(marker_name, msg->pose, msg->header);
  server_->applyChanges();
}


bool InteractiveMarkerInterface::set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
					  jsk_interactive_marker::MarkerSetPose::Response &res ) {

  if ( req.markers.size() > 0 ) {
    visualization_msgs::InteractiveMarker mk;
    if ( server_->get(req.marker_name, mk) ) {
      visualization_msgs::InteractiveMarkerControl mkc;
      mkc.name = "additional_marker";
      mkc.always_visible = true;
      mkc.markers = req.markers;
      // delete added marker
      for ( std::vector<visualization_msgs::InteractiveMarkerControl>::iterator it
	      =  mk.controls.begin();
	    it != mk.controls.end(); it++ ) {
	if ( it->name == mkc.name ){
	  mk.controls.erase( it );
	  break;
	}
      }
      mk.controls.push_back( mkc );
    }
  }
  std::string mName = req.marker_name;
  if(mName == ""){
    mName = marker_name;
  }
  pub_marker_tf(req.pose.header, req.pose.pose);

  server_->setPose(mName, req.pose.pose, req.pose.header);
  server_->applyChanges();
  pub_update_.publish(req.pose);
  return true;
}

bool InteractiveMarkerInterface::reset_cb ( jsk_interactive_marker::SetPose::Request &req,
					    jsk_interactive_marker::SetPose::Response &res ) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = base_frame;
  if ( target_frame != "" ) {
    /*
      tf::StampedTransform stf;
      geometry_msgs::TransformStamped mtf;
      tfl_.lookupTransform(base_frame, target_frame,
      ros::Time(0), stf);
      tf::transformStampedTFToMsg(stf, mtf);
      pose.pose.position.x = mtf.transform.translation.x;
      pose.pose.position.y = mtf.transform.translation.y;
      pose.pose.position.z = mtf.transform.translation.z;
      pose.pose.orientation = mtf.transform.rotation;
      // pose.header = mtf.header;
      // pose.header.stamp = ros::Time::Now();
      // pose.header.frame_id = target_frame;
      server_->setPose(marker_name, pose.pose, pose.header);
    */
  } else {
    server_->setPose(marker_name, pose.pose);
  }
  server_->applyChanges();
  return true;
}


void InteractiveMarkerInterface::makeIMVisible(visualization_msgs::InteractiveMarker &im){
  for(int i=0; i<im.controls.size(); i++){
    im.controls[i].always_visible = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_marker_interface");
  InteractiveMarkerInterface imi;
  ros::spin();

  return 0;
}
