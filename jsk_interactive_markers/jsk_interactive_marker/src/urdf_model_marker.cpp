#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>
#include <Eigen/Geometry>

#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace urdf;
using namespace std;




void UrdfModelMarker::addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, bool root){
  visualization_msgs::InteractiveMarkerControl control;
  if(root){
    im_helpers::add6DofControl(int_marker,false);
    for(int i=0; i<int_marker.controls.size(); i++){
      int_marker.controls[i].always_visible = true;
    }
  }else{
    boost::shared_ptr<Joint> parent_joint = link->parent_joint;
    Eigen::Vector3f origin_x(1,0,0);
    Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
    Eigen::Quaternionf qua;

    qua.setFromTwoVectors(origin_x, dest_x);
    control.orientation.x = qua.x();
    control.orientation.y = qua.y();
    control.orientation.z = qua.z();
    control.orientation.w = qua.w();

    //cout << parent_joint->axis.x << parent_joint->axis.y << parent_joint->axis.z<< endl;
    //cout << qua.x() << qua.y() << qua.z() << qua.w() << endl;

    //cout << parent_joint->type << endl;
    int_marker.scale = 0.5;

    switch(parent_joint->type){
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      //interactive_markers::makeDisc ( int_marker, control, 1);
      int_marker.controls.push_back(control);
      break;
    case Joint::PRISMATIC:
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      break;
    default:
      break;
    }
  }
}

void UrdfModelMarker::addInvisibleMeshMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, const std_msgs::ColorRGBA &color){
  visualization_msgs::InteractiveMarkerControl control;
  //    ps.pose = UrdfPose2Pose(link->visual->origin);
  visualization_msgs::Marker marker;

  //if (use_color) marker.color = color;
  marker.type = visualization_msgs::Marker::CYLINDER;
  double scale=0.01;
  marker.scale.x = scale;
  marker.scale.y = scale * 1;
  marker.scale.z = scale * 40;
  marker.color = color;
  //marker.pose = stamped.pose;
  //visualization_msgs::InteractiveMarkerControl control;
  boost::shared_ptr<Joint> parent_joint = link->parent_joint;
  Eigen::Vector3f origin_x(0,0,1);
  Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
  Eigen::Quaternionf qua;

  qua.setFromTwoVectors(origin_x, dest_x);
  marker.pose.orientation.x = qua.x();
  marker.pose.orientation.y = qua.y();
  marker.pose.orientation.z = qua.z();
  marker.pose.orientation.w = qua.w();

  control.markers.push_back( marker );
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  int_marker.controls.push_back(control);
  return;
}


void UrdfModelMarker::addGraspPointControl(visualization_msgs::InteractiveMarker &int_marker, std::string link_frame_name_){
  //yellow sphere
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  if(linkMarkerMap[link_frame_name_].gp.displayMoveMarker){
    im_helpers::add6DofControl(int_marker,false);
  }
}



geometry_msgs::Transform UrdfModelMarker::Pose2Transform( const geometry_msgs::Pose pose_msg){
  geometry_msgs::Transform tf_msg;
  tf_msg.translation.x = pose_msg.position.x;
  tf_msg.translation.y = pose_msg.position.y;
  tf_msg.translation.z = pose_msg.position.z;
  tf_msg.rotation = pose_msg.orientation;
  return tf_msg;
}

geometry_msgs::Pose UrdfModelMarker::UrdfPose2Pose( const urdf::Pose pose){
  geometry_msgs::Pose p_msg;
  double x, y, z, w;
  pose.rotation.getQuaternion(x,y,z,w);
  p_msg.orientation.x = x;
  p_msg.orientation.y = y;
  p_msg.orientation.z = z;
  p_msg.orientation.w = w;

  p_msg.position.x = pose.position.x;
  p_msg.position.y = pose.position.y;
  p_msg.position.z = pose.position.z;

  return p_msg;
}

void UrdfModelMarker::CallSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::Transform transform){
  dynamic_tf_publisher::SetDynamicTF SetTf;
  SetTf.request.freq = 10;
  SetTf.request.cur_tf.header.stamp = ros::Time::now();
  SetTf.request.cur_tf.header.frame_id = parent_frame_id;
  SetTf.request.cur_tf.child_frame_id = frame_id;
  SetTf.request.cur_tf.transform = transform;
  if (use_dynamic_tf_){
      dynamic_tf_publisher_client.call(SetTf);
  }
}

void UrdfModelMarker::publishMarkerPose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  jsk_interactive_marker::MarkerPose mp;
  mp.pose.header = feedback->header;
  mp.pose.pose = feedback->pose;
  mp.marker_name = feedback->marker_name;
  mp.type = jsk_interactive_marker::MarkerPose::GENERAL;
  pub_.publish( mp );
}

void UrdfModelMarker::publishMarkerMenu( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu ){
  jsk_interactive_marker::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu=menu;
  pub_move_.publish(m);
}

void UrdfModelMarker::publishMoveObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

  //jsk_interactive_marker::MoveObject m;
  jsk_interactive_marker::MarkerMenu m;
  /*
    m.goal.pose.header = feedback->header;
    m.goal.pose.pose = feedback->pose;

    m.origin.pose.header = feedback->header;
    m.origin.pose.pose = linkMarkerMap[feedback->marker_name].origin;

    m.graspPose = linkMarkerMap[feedback->marker_name].gp.pose;
    pub_move_.publish(m);*/
}

void UrdfModelMarker::publishJointState( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  sensor_msgs::JointState js;
  js.header = feedback->header;
  getJointState(model->getRoot(), js);
  pub_joint_state_.publish( js );
}

void UrdfModelMarker::republishJointState( sensor_msgs::JointState js){
  js.header.stamp = ros::Time::now();
  pub_joint_state_.publish( js );
}


void UrdfModelMarker::resetJointStatesCB( const sensor_msgs::JointStateConstPtr &msg){
  setJointState(model->getRoot(), msg);
  addChildLinkNames(model->getRoot(), true, false);
  republishJointState(*msg);

  if(mode_ == "visualization"){
    //fix fixed_link of Marker on fixed_link of Robot
    string marker_name =  model_name_ + "/" + model->getRoot()->name;

    tf::StampedTransform stf_robot;
    tfl_.lookupTransform(fixed_link_, model->getRoot()->name,
			 ros::Time(0), stf_robot);

    tf::StampedTransform stf_marker;
    tfl_.lookupTransform(marker_name, model_name_ + "/" + fixed_link_,
			 ros::Time(0), stf_marker);
    
    KDL::Frame robotFrame;
    KDL::Frame markerFrame;
    KDL::Frame robotMarkerFrame;
    tf::TransformTFToKDL(stf_robot, robotFrame);
    tf::TransformTFToKDL(stf_marker, markerFrame);
    
    robotMarkerFrame = robotFrame * markerFrame;

    //TODO set Link
    geometry_msgs::Pose pose;
    pose.position.y = 1;
    pose.orientation.w = 1;


    server_->setPose(marker_name, pose);
    linkMarkerMap[marker_name].pose = pose;
    CallSetDynamicTf(frame_id_, marker_name, Pose2Transform(pose));
    addChildLinkNames(model->getRoot(), true, false);
  }
}


void UrdfModelMarker::proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id){
  switch ( feedback->event_type ){
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    linkMarkerMap[frame_id].pose = feedback->pose;
    CallSetDynamicTf(parent_frame_id, frame_id, Pose2Transform(feedback->pose));
    cout << "parent:" << parent_frame_id << " frame:" << frame_id <<endl;
    publishMarkerPose(feedback);
    publishJointState(feedback);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << frame_id << mode_ << endl;
    //linkMarkerMap[frame_id].displayMoveMarker ^= true;
    if (mode_ != "visualization"){
      linkMarkerMap[linkMarkerMap[frame_id].movable_link].displayMoveMarker ^= true;
      addChildLinkNames(model->getRoot(), true, false);
    }
    break;
  }
}



void UrdfModelMarker::graspPointCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  //linkMarkerMap[feedback->marker_name].gp.pose = feedback->pose;
  //publishMarkerPose(feedback);
  //publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::MOVE);

  KDL::Vector graspVec(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
  KDL::Frame parentFrame;
  tf::PoseMsgToKDL (linkMarkerMap[feedback->marker_name].pose, parentFrame);

  graspVec = parentFrame.Inverse(graspVec);

  geometry_msgs::Pose p;
  p.position.x = graspVec.x();
  p.position.y = graspVec.y();
  p.position.z = graspVec.z();
  p.orientation = linkMarkerMap[feedback->marker_name].gp.pose.orientation;
  linkMarkerMap[feedback->marker_name].gp.pose = p;
  
  linkMarkerMap[feedback->marker_name].gp.displayGraspPoint = true;
  addChildLinkNames(model->getRoot(), true, false);
  //addChildLinkNames(model->getRoot(), true, false, true, 0);
}


void UrdfModelMarker::jointMoveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  publishJointState(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::JOINT_MOVE);
}

void UrdfModelMarker::resetMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  
  publishJointState(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::RESET_JOINT);
}

void UrdfModelMarker::registrationCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  
  publishJointState(feedback);
}



/*
  void UrdfModelMarker::moveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  publishMarkerPose(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::MOVE);
  }

  void UrdfModelMarker::setPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  publishMarkerPose(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::SET_ORIGIN);
  }
*/
/*
  void UrdfModelMarker::jointStatesCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  publishJointState();
  }
*/
void UrdfModelMarker::moveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  jsk_interactive_marker::MoveObject mo;
  mo.origin.header = feedback->header;
  mo.origin.pose = linkMarkerMap[feedback->marker_name].origin;

  mo.goal.header = feedback->header;
  mo.goal.pose = feedback->pose;

  mo.graspPose = linkMarkerMap[feedback->marker_name].gp.pose;
  
  pub_move_object_.publish( mo );

}

void UrdfModelMarker::setPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  cout << "setPose" <<endl;
  setOriginalPose(model->getRoot());
}

void UrdfModelMarker::hideMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  linkMarkerMap[linkMarkerMap[feedback->marker_name].movable_link].displayMoveMarker = false;
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideAllMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while( it != linkMarkerMap.end() )
    {
      (*it).second.displayMoveMarker = false;
      ++it;
    }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideModelMarkerCB( const std_msgs::EmptyConstPtr &msg){
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while( it != linkMarkerMap.end() )
    {
      (*it).second.displayModelMarker = false;
      ++it;
    }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::showModelMarkerCB( const std_msgs::EmptyConstPtr &msg){
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while( it != linkMarkerMap.end() )
    {
      (*it).second.displayModelMarker = true;
      ++it;
    }
  addChildLinkNames(model->getRoot(), true, false);

}




void UrdfModelMarker::graspPoint_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string link_name){
  switch ( feedback->event_type ){
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    linkMarkerMap[link_name].gp.pose = feedback->pose;
    publishMarkerPose(feedback);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << feedback->marker_name << endl;
    linkMarkerMap[link_name].gp.displayMoveMarker ^= true;
    addChildLinkNames(model->getRoot(), true, false);
    break;
  }
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color, bool use_color){
  visualization_msgs::Marker meshMarker;

  if (use_color) meshMarker.color = color;
  meshMarker.mesh_resource = mesh_resource;
  meshMarker.mesh_use_embedded_materials = !use_color;
  meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  
  meshMarker.scale.x = scale;
  meshMarker.scale.y = scale;
  meshMarker.scale.z = scale;
  meshMarker.pose = stamped.pose;
  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back( meshMarker );
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  
  return control;
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale)
{
  std_msgs::ColorRGBA color;
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, false);
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource,
										    const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color)
{
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, true);
}


void UrdfModelMarker::getJointState(boost::shared_ptr<const Link> link, sensor_msgs::JointState &js)
{
  string link_frame_name_ =  model_name_ + "/" + link->name;
  boost::shared_ptr<Joint> parent_joint = link->parent_joint;
  if(parent_joint != NULL){
    KDL::Frame initialFrame;
    KDL::Frame presentFrame;
    KDL::Rotation rot;
    KDL::Vector rotVec;
    KDL::Vector jointVec;
    double jointAngle;
    double jointAngleAllRange;
    switch(parent_joint->type){
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      tf::PoseMsgToKDL (linkMarkerMap[link_frame_name_].initial_pose, initialFrame);
      tf::PoseMsgToKDL (linkMarkerMap[link_frame_name_].pose, presentFrame);
      rot = initialFrame.M.Inverse() * presentFrame.M;
      jointAngle = rot.GetRotAngle(rotVec);
      jointVec = KDL::Vector(linkMarkerMap[link_frame_name_].joint_axis.x,
			     linkMarkerMap[link_frame_name_].joint_axis.y,
			     linkMarkerMap[link_frame_name_].joint_axis.z);
      if( KDL::dot(rotVec,jointVec) < 0){
	jointAngle = - jointAngle;
      }
      if(linkMarkerMap[link_frame_name_].joint_angle > M_PI/2 && jointAngle < -M_PI/2){
	linkMarkerMap[link_frame_name_].rotation_count += 1;
      }else if(linkMarkerMap[link_frame_name_].joint_angle < -M_PI/2 && jointAngle > M_PI/2){
	linkMarkerMap[link_frame_name_].rotation_count -= 1;
      }
      linkMarkerMap[link_frame_name_].joint_angle = jointAngle;
      jointAngleAllRange = jointAngle + linkMarkerMap[link_frame_name_].rotation_count * M_PI * 2;
      //check joint limit
      //cout << "aa" << linkMarkerMap[link_frame_name_].rotation_count << endl;
      //cout << jointAngleAllRange << endl;

      if(parent_joint->type == Joint::REVOLUTE && parent_joint->limits != NULL){
	bool changeMarkerAngle = false;
	if(jointAngleAllRange < parent_joint->limits->lower){
	  jointAngleAllRange = parent_joint->limits->lower + 0.001;
	  changeMarkerAngle = true;
	}
	if(jointAngleAllRange > parent_joint->limits->upper){
	  jointAngleAllRange = parent_joint->limits->upper - 0.001;
	  changeMarkerAngle = true;
	}

	if(changeMarkerAngle){
	  cout << "Detect Joint Limit" << endl;
	  cout << jointAngleAllRange <<endl;
	  setJointAngle(link, jointAngleAllRange);
	}
      }

      js.position.push_back(jointAngleAllRange);
      js.name.push_back(parent_joint->name);
      break;
    case Joint::PRISMATIC:
      //TODO
      break;
    default:
      break;
    }
  }
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    getJointState(*child, js);
  }
  return;
}

void UrdfModelMarker::setJointAngle(boost::shared_ptr<const Link> link, double joint_angle){
  string link_frame_name_ =  model_name_ + "/" + link->name;
  boost::shared_ptr<Joint> parent_joint = link->parent_joint;

  if(parent_joint == NULL){
    return;
  }

  KDL::Frame initialFrame;
  KDL::Frame presentFrame;
  KDL::Rotation rot;
  KDL::Vector rotVec;
  KDL::Vector jointVec;

  std_msgs::Header link_header;

  int rotation_count = 0;
  if(joint_angle > M_PI){
    rotation_count = (int)((joint_angle + M_PI) / (M_PI * 2));
    joint_angle -= rotation_count * M_PI * 2;
  }else if(joint_angle < -M_PI){
    rotation_count = (int)((- joint_angle + M_PI) / (M_PI * 2));
    joint_angle -= rotation_count * M_PI * 2;
  }
  
  cout<< "joint" << joint_angle << endl;
  cout<< "rot" << rotation_count << endl;

  linkMarkerMap[link_frame_name_].joint_angle = joint_angle;
  linkMarkerMap[link_frame_name_].rotation_count = rotation_count;

  tf::PoseMsgToKDL (linkMarkerMap[link_frame_name_].initial_pose, initialFrame);
  tf::PoseMsgToKDL (linkMarkerMap[link_frame_name_].initial_pose, presentFrame);
  jointVec = KDL::Vector(linkMarkerMap[link_frame_name_].joint_axis.x,
			 linkMarkerMap[link_frame_name_].joint_axis.y,
			 linkMarkerMap[link_frame_name_].joint_axis.z);
  cout << joint_angle << endl;
  presentFrame.M = KDL::Rotation::Rot(jointVec, joint_angle) * initialFrame.M;
  tf::PoseKDLToMsg(presentFrame, linkMarkerMap[link_frame_name_].pose);

  //link_header.stamp = ros::Time::now();
  link_header.stamp = ros::Time(0);
  link_header.frame_id = linkMarkerMap[link_frame_name_].frame_id;
  

  server_->setPose(link_frame_name_, linkMarkerMap[link_frame_name_].pose, link_header);
  server_->applyChanges();
  CallSetDynamicTf(linkMarkerMap[link_frame_name_].frame_id, link_frame_name_, Pose2Transform(linkMarkerMap[link_frame_name_].pose));
  //addChildLinkNames(model->getRoot(), true, false);
  
}

void UrdfModelMarker::setJointState(boost::shared_ptr<const Link> link, const sensor_msgs::JointStateConstPtr &js)
{
  string link_frame_name_ =  model_name_ + "/" + link->name;
  boost::shared_ptr<Joint> parent_joint = link->parent_joint;
  if(parent_joint != NULL){
    KDL::Frame initialFrame;
    KDL::Frame presentFrame;
    KDL::Rotation rot;
    KDL::Vector rotVec;
    KDL::Vector jointVec;
    double jointAngle;
    bool changeAngle = false;
    std_msgs::Header link_header;
    switch(parent_joint->type){
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      for(int i=0; i< js->name.size(); i++){
	if(js->name[i] == parent_joint->name){
	  jointAngle = js->position[i];
	  changeAngle = true;
	  break;
	}
      }
      if(!changeAngle){
	break;
      }
      setJointAngle(link, jointAngle);
      break;
    case Joint::PRISMATIC:
      //TODO
      break;
    default:
      break;
    }
  }
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    setJointState(*child, js);
  }
  return;
}



void UrdfModelMarker::setOriginalPose(boost::shared_ptr<const Link> link)
{
  string link_frame_name_ =  model_name_ + "/" + link->name;
  linkMarkerMap[link_frame_name_].origin =  linkMarkerMap[link_frame_name_].pose;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    setOriginalPose(*child);
  }
}

void UrdfModelMarker::addChildLinkNames(boost::shared_ptr<const Link> link, bool root, bool init){
  //addChildLinkNames(link, root, init, false, 0);
  addChildLinkNames(link, root, init, true, 0);
}

void UrdfModelMarker::addChildLinkNames(boost::shared_ptr<const Link> link, bool root, bool init, bool use_color, int color_index)
{
  geometry_msgs::PoseStamped ps;

  double scale_factor = 1.02;
  string link_frame_name_ =  model_name_ + "/" + link->name;
  string parent_link_frame_name_;

  if(root){
    parent_link_frame_name_ = frame_id_;
    ps.pose = root_pose_;
  }else{
    parent_link_frame_name_ = link->parent_joint->parent_link_name;
    parent_link_frame_name_ = model_name_ + "/" + parent_link_frame_name_;
    ps.pose = UrdfPose2Pose(link->parent_joint->parent_to_joint_origin_transform);
  }
  ps.header.frame_id =  parent_link_frame_name_;
  //ps.header.stamp = ros::Time::now();
  ps.header.stamp = ros::Time(0);

  //initialize linkProperty
  if(init){
    CallSetDynamicTf(parent_link_frame_name_, link_frame_name_, Pose2Transform(ps.pose));
    linkProperty lp;
    lp.pose = ps.pose;
    lp.origin = ps.pose;
    lp.initial_pose = ps.pose;
    if(link->parent_joint !=NULL){
      lp.joint_axis = link->parent_joint->axis;
    }
    lp.joint_angle = 0;
    lp.rotation_count=0;
    if(link->parent_joint !=NULL && link->parent_joint->type == Joint::FIXED){
      lp.movable_link = linkMarkerMap[parent_link_frame_name_].movable_link;
    }else{
      lp.movable_link = link_frame_name_;
    }

    linkMarkerMap.insert( map<string, linkProperty>::value_type( link_frame_name_, lp ) );
  }

  linkMarkerMap[link_frame_name_].frame_id = parent_link_frame_name_;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;

  int_marker.name = link_frame_name_;
  int_marker.scale = 1.0;
  int_marker.pose = ps.pose;

  if(!init){
    visualization_msgs::InteractiveMarker old_marker;
    if(server_->get(link_frame_name_, old_marker)){
      int_marker.pose = old_marker.pose;
    }
  }

  //hide marker
  if(!linkMarkerMap[link_frame_name_].displayModelMarker){
    server_->erase(link_frame_name_);
    server_->erase(model_name_ + "/" + link->name + "/grasp"); //grasp marker
  }else{


    //move Marker  
    if(linkMarkerMap[link_frame_name_].displayMoveMarker){
      addMoveMarkerControl(int_marker, link, root);
    }


    //model Mesh Marker
    std_msgs::ColorRGBA color;
    if(mode_ == "visualization"){
      color.r = (double)0xFF / 0xFF;
      color.g = (double)0xFF / 0xFF;
      color.b = (double)0x00 / 0xFF;
      color.a = 0.5;
    }else{
      switch(color_index %3){
      case 0:
	color.r = (double)0xFF / 0xFF;
	color.g = (double)0xC3 / 0xFF;
	color.b = (double)0x00 / 0xFF;
	break;
      case 1:
	color.r = (double)0x58 / 0xFF;
	color.g = (double)0x0E / 0xFF;
	color.b = (double)0xAD / 0xFF;
	break;
      case 2:
	color.r = (double)0x0C / 0xFF;
	color.g = (double)0x5A / 0xFF;
	color.b = (double)0xA6 / 0xFF;
	break;
      }
      color.a = 1.0;
    }

    if(link->visual.get() != NULL && link->visual->geometry.get() != NULL && link->visual->geometry->type == Geometry::MESH){
      boost::shared_ptr<const Mesh> mesh = boost::static_pointer_cast<const Mesh>(link->visual->geometry);
      string model_mesh_ = mesh->filename;
      //model_mesh_ = getModelFilePath(model_mesh_);
      model_mesh_ = getRosPathFromModelPath(model_mesh_);
    
      ps.pose = UrdfPose2Pose(link->visual->origin);
      cout << "mesh_file:" << model_mesh_ << endl;

      visualization_msgs::InteractiveMarkerControl meshControl;
      if(use_color){
	meshControl = makeMeshMarkerControl(model_mesh_, ps, scale_factor_, color);
      }else{
	meshControl = makeMeshMarkerControl(model_mesh_, ps, scale_factor_);
      }
      int_marker.controls.push_back( meshControl );

      server_->insert(int_marker);
      server_->setCallback( int_marker.name,
			    boost::bind( &UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_) );

      model_menu_.apply(*server_, link_frame_name_);
    }else{
      boost::shared_ptr<Joint> parent_joint = link->parent_joint;
      if(parent_joint != NULL){
	if(parent_joint->type==Joint::REVOLUTE || parent_joint->type==Joint::REVOLUTE){
	  addInvisibleMeshMarkerControl(int_marker, link, color);
	  server_->insert(int_marker);
	  server_->setCallback( int_marker.name,
				boost::bind( &UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_) );
	  model_menu_.apply(*server_, link_frame_name_);
	}
      }

    }
    if(!robot_mode_){
      //add Grasp Point Marker
      if(linkMarkerMap[link_frame_name_].gp.displayGraspPoint){
	visualization_msgs::InteractiveMarker grasp_int_marker;
	double grasp_scale_factor = 1.02;
	string grasp_link_frame_name_ = model_name_ + "/" + link->name + "/grasp";
	string grasp_parent_link_frame_name_ = model_name_ + "/" + link->name;

	geometry_msgs::PoseStamped grasp_ps;
	grasp_ps.pose = linkMarkerMap[link_frame_name_].gp.pose;
	grasp_ps.header.frame_id =  grasp_parent_link_frame_name_;

	grasp_int_marker.header = grasp_ps.header;
	grasp_int_marker.name = grasp_link_frame_name_;
	grasp_int_marker.scale = 1.0;
	grasp_int_marker.pose = grasp_ps.pose;

	addGraspPointControl(grasp_int_marker, link_frame_name_);

	server_->insert(grasp_int_marker);
	server_->setCallback( grasp_int_marker.name,
			      boost::bind( &UrdfModelMarker::graspPoint_feedback, this, _1, link_frame_name_));

      }
    }
  }
  server_->applyChanges();

  //  cout << "Link:" << link->name << endl;

  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    addChildLinkNames(*child, false, init, use_color, color_index + 1);
  }
}



UrdfModelMarker::UrdfModelMarker ()
{}

UrdfModelMarker::UrdfModelMarker (string model_name, string model_file, string frame_id, geometry_msgs::Pose root_pose, double scale_factor, string mode, bool robot_mode, bool registration, string fixed_link, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : nh_(), pnh_("~"), tfl_(nh_),use_dynamic_tf_(true) {
  pnh_.param("server_name", server_name, std::string ("") );

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  
  pnh_.getParam("use_dynamic_tf", use_dynamic_tf_);
  if (use_dynamic_tf_) {
      dynamic_tf_publisher_client = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf");
      ros::service::waitForService("set_dynamic_tf", -1);
  }
  std::cerr << "use_dynamic_tf_ is " << use_dynamic_tf_ << std::endl;

  server_ = server;
  model_name_ = model_name;
  model_file_ = model_file;
  frame_id_ = frame_id;
  root_pose_ = root_pose;
  scale_factor_ = scale_factor;
  robot_mode_ = robot_mode;
  registration_ = registration;
  mode_ = mode;
  fixed_link_ = fixed_link;

  pub_ =  pnh_.advertise<jsk_interactive_marker::MarkerPose> ("pose", 1);
  pub_move_ =  pnh_.advertise<jsk_interactive_marker::MarkerMenu> ("marker_menu", 1);
  pub_move_object_ =  pnh_.advertise<jsk_interactive_marker::MoveObject> ("move_object", 1);

  pub_joint_state_ =  pnh_.advertise<sensor_msgs::JointState> (model_name_ + "/joint_states", 1);

  sub_reset_joints_ = pnh_.subscribe<sensor_msgs::JointState> (model_name_ + "/reset_joint_states", 1, boost::bind( &UrdfModelMarker::resetJointStatesCB, this, _1));
  
  hide_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/hide_marker", 1, boost::bind( &UrdfModelMarker::hideModelMarkerCB, this, _1));
  show_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/show_marker", 1, boost::bind( &UrdfModelMarker::showModelMarkerCB, this, _1));

  /*
    serv_set_ = pnh_.advertiseService("set_pose",
    &InteractiveMarkerInterface::set_cb, this);
    serv_markers_set_ = pnh_.advertiseService("set_markers",
    &InteractiveMarkerInterface::markers_set_cb, this);
    serv_markers_del_ = pnh_.advertiseService("del_markers",
    &InteractiveMarkerInterface::markers_del_cb, this);
    serv_reset_ = pnh_.advertiseService("reset_pose",
    &InteractiveMarkerInterface::reset_cb, this);
  */
  if(mode_ == ""){
    if(registration_){
      mode_ = "registration";
    }else if(robot_mode_){
      mode_ = "robot";
    }else{
      mode_ = "model";
    }
  }

  if(mode_ == "registration"){
    model_menu_.insert( "Registration",
			boost::bind( &UrdfModelMarker::registrationCB, this, _1) );
  }else if(mode_ == "visualization"){

  }else if(mode_ == "robot"){
      interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
      sub_menu_move_ = model_menu_.insert( "Move" );
      model_menu_.insert( sub_menu_move_, "Yes", 
			  boost::bind( &UrdfModelMarker::jointMoveCB, this, _1) );
      //    model_menu_.insert( "Move" ,
      //boost::bind( &UrdfModelMarker::jointMoveCB, this, _1) );
      model_menu_.insert( "Reset Marker Pose",
			  boost::bind( &UrdfModelMarker::resetMarkerCB, this, _1) );
      
      

      interactive_markers::MenuHandler::EntryHandle sub_menu_pose_;
      sub_menu_pose_ = model_menu_.insert( "Special Pose" );

      model_menu_.insert( sub_menu_pose_, "Stand Pose",
			  boost::bind( &UrdfModelMarker::publishMarkerMenu, this, _1, 100) );

      model_menu_.insert( sub_menu_pose_, "Manip Pose",
			  boost::bind( &UrdfModelMarker::publishMarkerMenu, this, _1, 101) );

      model_menu_.insert( "Reset Marker Pose",
			  boost::bind( &UrdfModelMarker::resetMarkerCB, this, _1) );


      model_menu_.insert( "Hide Marker" ,
			  boost::bind( &UrdfModelMarker::hideMarkerCB, this, _1) );
      model_menu_.insert( "Hide All Marker" ,
			  boost::bind( &UrdfModelMarker::hideAllMarkerCB, this, _1) );


  }else if(mode_ == "model"){
      model_menu_.insert( "Grasp Point",
			  boost::bind( &UrdfModelMarker::graspPointCB, this, _1 ) );
      model_menu_.insert( "Move",
			  boost::bind( &UrdfModelMarker::moveCB, this, _1 ) );
      model_menu_.insert( "Set as present pose",
			  boost::bind( &UrdfModelMarker::setPoseCB, this, _1 ) );
      model_menu_.insert( "Hide Marker" ,
			  boost::bind( &UrdfModelMarker::hideMarkerCB, this, _1) );
      model_menu_.insert( "Hide All Marker" ,
			  boost::bind( &UrdfModelMarker::hideAllMarkerCB, this, _1) );
  }

  model_file_ = getFilePathFromRosPath(model_file_);

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(model_file_.c_str(), std::fstream::in);
  while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
  xml_file.close();

  std::cout << "model_file:" << model_file_ << std::endl;
  model = parseURDF(xml_string);
  if (!model){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return;
  }


  addChildLinkNames(model->getRoot(), true, true);

  // wait for joint\statelistner
  if ( pub_joint_state_.getNumSubscribers() == 0 ) {
      ROS_WARN_STREAM("urdf_model_marker wait for joint_state_subscriber .... ");
      ros::Duration(1.0).sleep();
  }
  ROS_WARN_STREAM("urdf_model_marker publish joint_state_subscriber, done");
  // start JointState
  sensor_msgs::JointState js;
  getJointState(model->getRoot(), js);
  pub_joint_state_.publish( js );

  return;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_model_marker_interface");
  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");

  string server_name;
  pnh_.param("server_name", server_name, std::string ("") );
  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }


  
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset( new interactive_markers::InteractiveMarkerServer(server_name, "", false) );



  XmlRpc::XmlRpcValue v;
  pnh_.param("model_config", v, v);
  for(int i=0; i< v.size(); i++){
    XmlRpc::XmlRpcValue model = v[i];
    std::cout << "name:" << model["name"] <<endl;
    geometry_msgs::Pose p = getPose(model["pose"]);
    double scale_factor = 1.02;
    if(model.hasMember("scale")){
      scale_factor = getXmlValue(model["scale"]);
    }
    string mode = "model";
    bool registration = false;
    string fixed_link = "";
    if(model.hasMember("registration")){
      registration = model["registration"];
      mode = "registration";
      if(model.hasMember("fixed-link")){
	fixed_link.assign( model["fixed-link"]);
	//fixed_link = model["fixed-link"];
      }
    }

    if(model["robot"]){
      mode = "robot";
    }
    if(model.hasMember("mode")){
      mode.assign(model["mode"]);
      //mode = model["mode"];
    }
    
    new UrdfModelMarker(model["name"], model["model"], model["frame-id"], p, scale_factor, mode , model["robot"], registration,fixed_link, server);
  }

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
