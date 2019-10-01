#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>
#include <Eigen/Geometry>

#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>
#include <jsk_topic_tools/log_utils.h>
#include <boost/filesystem.hpp>

using namespace urdf;
using namespace std;
using namespace im_utils;

void UrdfModelMarker::addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, LinkConstSharedPtr link, bool root) {
  visualization_msgs::InteractiveMarkerControl control;
  if (root) {
    im_helpers::add6DofControl(int_marker,false);
    for(int i=0; i<int_marker.controls.size(); i++) {
      int_marker.controls[i].always_visible = true;
    }
  }
  else {
    JointSharedPtr parent_joint = link->parent_joint;
    Eigen::Vector3f origin_x(1,0,0);
    Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
    Eigen::Quaternionf qua;

    qua.setFromTwoVectors(origin_x, dest_x);
    control.orientation.x = qua.x();
    control.orientation.y = qua.y();
    control.orientation.z = qua.z();
    control.orientation.w = qua.w();

    int_marker.scale = 0.5;

    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
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

void UrdfModelMarker::addInvisibleMeshMarkerControl(visualization_msgs::InteractiveMarker &int_marker, LinkConstSharedPtr link, const std_msgs::ColorRGBA &color) {
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
  JointSharedPtr parent_joint = link->parent_joint;
  Eigen::Vector3f origin_x(0,0,1);
  Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
  Eigen::Quaternionf qua;

  qua.setFromTwoVectors(origin_x, dest_x);
  marker.pose.orientation.x = qua.x();
  marker.pose.orientation.y = qua.y();
  marker.pose.orientation.z = qua.z();
  marker.pose.orientation.w = qua.w();

  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  int_marker.controls.push_back(control);
  return;
}


void UrdfModelMarker::addGraspPointControl(visualization_msgs::InteractiveMarker &int_marker, std::string link_frame_name_) {
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

  if (linkMarkerMap[link_frame_name_].gp.displayMoveMarker) {
    im_helpers::add6DofControl(int_marker,false);
  }
}


void UrdfModelMarker::callSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::Transform transform) {
  jsk_topic_tools::ScopedTimer timer = dynamic_tf_check_time_acc_.scopedTimer();
  dynamic_tf_publisher::SetDynamicTF SetTf;
  //SetTf.request.freq = 10;
  SetTf.request.freq = 20;
  //SetTf.request.freq = 100;
  SetTf.request.cur_tf.header.stamp = ros::Time::now();
  SetTf.request.cur_tf.header.frame_id = parent_frame_id;
  SetTf.request.cur_tf.child_frame_id = frame_id;
  SetTf.request.cur_tf.transform = transform;
  if (use_dynamic_tf_ || parent_frame_id == frame_id_) {
    //std::cout << parent_frame_id << frame_id << std::endl;
    dynamic_tf_publisher_client.call(SetTf);
  }
}

void UrdfModelMarker::callPublishTf() {
  if (use_dynamic_tf_) {
    std_srvs::Empty req;
    dynamic_tf_publisher_publish_tf_client.call(req);
  }
}

void UrdfModelMarker::publishBasePose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  publishBasePose(feedback->pose, feedback->header);
}

void UrdfModelMarker::publishBasePose(geometry_msgs::Pose pose, std_msgs::Header header) {
  geometry_msgs::PoseStamped ps;
  ps.pose = pose;
  ps.header = header;
  pub_base_pose_.publish(ps);
}



void UrdfModelMarker::publishMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  publishMarkerPose(feedback->pose, feedback->header, feedback->marker_name);
}

void UrdfModelMarker::publishMarkerPose(geometry_msgs::Pose pose, std_msgs::Header header, std::string marker_name) {
  jsk_interactive_marker::MarkerPose mp;
  mp.pose.header = header;
  mp.pose.pose = pose;
  mp.marker_name = marker_name;
  mp.type = jsk_interactive_marker::MarkerPose::GENERAL;
  pub_.publish(mp);
}


void UrdfModelMarker::publishMarkerMenu(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu) {
  jsk_interactive_marker::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu=menu;
  pub_move_.publish(m);
}

void UrdfModelMarker::publishMoveObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

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

void UrdfModelMarker::publishJointState(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  publishJointState();
}

void UrdfModelMarker::republishJointState(sensor_msgs::JointState js) {
  js.header.stamp = ros::Time::now();
  pub_joint_state_.publish(js);
}

void UrdfModelMarker::setRootPoseCB(const geometry_msgs::PoseStampedConstPtr &msg) {
  setRootPose(*msg);
}
void UrdfModelMarker::setRootPose (geometry_msgs::PoseStamped ps) {
  try {
    init_stamp_ = ps.header.stamp;
    tfl_.transformPose(frame_id_, ps, ps);

    geometry_msgs::Pose pose = getRootPose(ps.pose);

    string root_frame = tf_prefix_ + model->getRoot()->name;
    linkMarkerMap[frame_id_].pose = pose;
    callSetDynamicTf(frame_id_, root_frame, Pose2Transform(pose));
    root_pose_ = pose;
    addChildLinkNames(model->getRoot(), true, false);
    publishMarkerPose(pose, ps.header, root_frame);
    
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

}




void UrdfModelMarker::resetJointStatesCB(const sensor_msgs::JointStateConstPtr &msg, bool update_root) {
  boost::mutex::scoped_lock lock(joint_states_mutex_);
  if (is_joint_states_locked_) {
    return;
  }
  jsk_topic_tools::ScopedTimer timer = reset_joint_states_check_time_acc_.scopedTimer();
  setJointState(model->getRoot(), msg);
  republishJointState(*msg);

  //update root correctly
  //this may take long time
  if (update_root) {
    callPublishTf();
    ros::spinOnce();
  }
  resetRootForVisualization();
  server_->applyChanges();
}


void UrdfModelMarker::proc_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id) {
  ROS_INFO("proc_feedback");
  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    linkMarkerMap[frame_id].pose = feedback->pose;
    //root link
    if (parent_frame_id == frame_id_) {
      root_pose_ = feedback->pose;
      publishBasePose(feedback);
    }
    callSetDynamicTf(parent_frame_id, frame_id, Pose2Transform(feedback->pose));
    publishMarkerPose(feedback);
    publishJointState(feedback);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << frame_id << mode_ << endl;
    //linkMarkerMap[frame_id].displayMoveMarker ^= true;
    if (mode_ != "visualization") {
      linkMarkerMap[linkMarkerMap[frame_id].movable_link].displayMoveMarker ^= true;
      addChildLinkNames(model->getRoot(), true, false);
    }
    else {
      geometry_msgs::PoseStamped ps = getOriginPoseStamped();
      pub_selected_.publish(ps);
      jsk_recognition_msgs::Int32Stamped index_msg;
      index_msg.data = index_;
      index_msg.header.stamp = init_stamp_;
      pub_selected_index_.publish(index_msg);
    }
    break;

  }
  diagnostic_updater_->update();
}



void UrdfModelMarker::graspPointCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  //linkMarkerMap[feedback->marker_name].gp.pose = feedback->pose;
  //publishMarkerPose(feedback);
  //publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::MOVE);

  KDL::Vector graspVec(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
  KDL::Frame parentFrame;
  tf::poseMsgToKDL (linkMarkerMap[feedback->marker_name].pose, parentFrame);

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


void UrdfModelMarker::jointMoveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  publishJointState(feedback);
  sleep(0.5);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::JOINT_MOVE);
}

void UrdfModelMarker::resetMarkerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  
  publishJointState(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker::MarkerMenu::RESET_JOINT);
}

void UrdfModelMarker::resetBaseMsgCB(const std_msgs::EmptyConstPtr &msg) {
  resetBaseCB();
}

void UrdfModelMarker::resetBaseMarkerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  resetBaseCB();
}
void UrdfModelMarker::resetBaseCB() {
  resetRobotBase();

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame_id_;
  ps.pose = root_pose_;
  setRootPose(ps);

  // to update root link marker
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::resetRobotBase() {
  //set root_pose_ to robot base pose
  try {
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped ts_msg;
    tfl_.lookupTransform(frame_id_, model->getRoot()->name,
                         ros::Time(0), transform);
    tf::transformStampedTFToMsg(transform, ts_msg);

    root_pose_ = Transform2Pose(ts_msg.transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
}

void UrdfModelMarker::resetRootForVisualization() {
  if (fixed_link_.size() > 0 && (mode_ == "visualization" || mode_ == "robot")) {
    string marker_name =  tf_prefix_ + model->getRoot()->name;
    tf::StampedTransform st_offset;
    bool first_offset = true;
    for(int i=0; i<fixed_link_.size(); i++) {
      std::string link = fixed_link_[i];
      if (!link.empty()) {
        ROS_DEBUG_STREAM("fixed_link:" << tf_prefix_ + model->getRoot()->name << tf_prefix_ + link);
        const std::string source_frame = tf_prefix_ + link;
        const std::string target_frame = tf_prefix_ + model->getRoot()->name;
        try {
          tf::StampedTransform st_link_offset;
          ros::Time now = ros::Time(0);
          tfl_.waitForTransform(target_frame, source_frame, now, ros::Duration(5.0));
          tfl_.lookupTransform(target_frame, source_frame,
                               now, st_link_offset);

          if (first_offset) {
            st_offset.setRotation(st_link_offset.getRotation());
            st_offset.setOrigin(st_link_offset.getOrigin());
            first_offset = false;
          }
          else {
            st_offset.setRotation(st_link_offset.getRotation().slerp(st_offset.getRotation(), (i * 1.0)/(i + 1)));
            st_offset.setOrigin(st_link_offset.getOrigin().lerp(st_offset.getOrigin(), (i* 1.0)/(i+1)));
          }
        }
        catch (tf::TransformException ex) {
          ROS_ERROR("Failed to lookup transformation from %s to %s: %s",
                        source_frame.c_str(), target_frame.c_str(),
                        ex.what());
        }
      }
    }

    //multiply fixed_link_offset_
    tf::StampedTransform st_fixed_link_offset;
    geometry_msgs::TransformStamped ts_fixed_link_offset;
    ts_fixed_link_offset.transform.translation.x = fixed_link_offset_.position.x;
    ts_fixed_link_offset.transform.translation.y = fixed_link_offset_.position.y;
    ts_fixed_link_offset.transform.translation.z = fixed_link_offset_.position.z;
    ts_fixed_link_offset.transform.rotation = fixed_link_offset_.orientation;

    tf::transformStampedMsgToTF(ts_fixed_link_offset, st_fixed_link_offset);

    tf::Transform transform;
    transform = st_offset * st_fixed_link_offset;

    //convert to root_offset_
    geometry_msgs::Transform tf_msg;
    tf::transformTFToMsg(transform, tf_msg);

    root_offset_.position.x = tf_msg.translation.x;
    root_offset_.position.y = tf_msg.translation.y;
    root_offset_.position.z = tf_msg.translation.z;
    root_offset_.orientation = tf_msg.rotation;

    //reset root_pose_
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = frame_id_;
    ps.pose.orientation.w = 1.0;
    //setRootPose(ps);
    root_pose_ = ps.pose;

    geometry_msgs::Pose pose = getRootPose(ps.pose);

    string root_frame = tf_prefix_ + model->getRoot()->name;
    linkMarkerMap[frame_id_].pose = pose;
    callSetDynamicTf(frame_id_, root_frame, Pose2Transform(pose));

    addChildLinkNames(model->getRoot(), true, false);
  }
}




void UrdfModelMarker::registrationCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  
  publishJointState(feedback);
}

void UrdfModelMarker::moveCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  /* publish jsk_interactive_marker::MoveModel */
  jsk_interactive_marker::MoveModel mm;
  mm.header = feedback->header;
  mm.name = model_name_;
  mm.description = model_description_;
  mm.joint_state_origin = joint_state_origin_;
  mm.joint_state_goal = joint_state_;
  mm.pose_origin.header.frame_id = frame_id_;
  mm.pose_origin.pose = root_pose_origin_;
  mm.pose_goal.header.frame_id = frame_id_;
  mm.pose_goal.pose = root_pose_;
  pub_move_model_.publish(mm);


  /* publish jsk_interactive_marker::MoveObject */
  jsk_interactive_marker::MoveObject mo;
  mo.origin.header = feedback->header;
  mo.origin.pose = linkMarkerMap[feedback->marker_name].origin;

  mo.goal.header = feedback->header;
  mo.goal.pose = feedback->pose;

  mo.graspPose = linkMarkerMap[feedback->marker_name].gp.pose;
  pub_move_object_.publish(mo);

}

void UrdfModelMarker::setPoseCB() {
  cout << "setPose" <<endl;
  joint_state_origin_ = joint_state_;
  root_pose_origin_ = root_pose_;
  setOriginalPose(model->getRoot());
}

void UrdfModelMarker::setPoseCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setPoseCB();
}

void UrdfModelMarker::hideMarkerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  linkMarkerMap[linkMarkerMap[feedback->marker_name].movable_link].displayMoveMarker = false;
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideAllMarkerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayMoveMarker = false;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideModelMarkerCB(const std_msgs::EmptyConstPtr &msg) {
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayModelMarker = false;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::showModelMarkerCB(const std_msgs::EmptyConstPtr &msg) {
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayModelMarker = true;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);

}

void UrdfModelMarker::setUrdfCB(const std_msgs::StringConstPtr &msg) {
  //clear
  server_->clear();
  linkMarkerMap.clear();

  model = parseURDF(msg->data);
  if (!model) {
    ROS_ERROR("Model Parsing the xml failed");
    return;
  }
  addChildLinkNames(model->getRoot(), true, true);

  // start JointState
  publishJointState();
  return;
}


void UrdfModelMarker::graspPoint_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string link_name) {
  switch (feedback->event_type) {
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

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color, bool use_color) {
  visualization_msgs::Marker meshMarker;

  if (use_color) meshMarker.color = color;
  meshMarker.mesh_resource = mesh_resource;
  meshMarker.mesh_use_embedded_materials = !use_color;
  meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  
  meshMarker.scale = scale;
  meshMarker.pose = stamped.pose;
  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back(meshMarker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  
  return control;
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale)
{
  std_msgs::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 0;
  //color.a = 0.3;
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, false);
  //return makeMeshMarkerControl(mesh_resource, stamped, scale, color, true);
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource,
                                                                                    const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color)
{
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, true);
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeCylinderMarkerControl(const geometry_msgs::PoseStamped &stamped, double length,  double radius, const std_msgs::ColorRGBA &color, bool use_color) {
  visualization_msgs::Marker cylinderMarker;

  if (use_color) cylinderMarker.color = color;
  cylinderMarker.type = visualization_msgs::Marker::CYLINDER;
  cylinderMarker.scale.x = radius * 2;
  cylinderMarker.scale.y = radius * 2;
  cylinderMarker.scale.z = length;
  cylinderMarker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back(cylinderMarker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeBoxMarkerControl(const geometry_msgs::PoseStamped &stamped, Vector3 dim, const std_msgs::ColorRGBA &color, bool use_color) {
  visualization_msgs::Marker boxMarker;

  fprintf(stderr, "urdfModelMarker = %f %f %f\n", dim.x, dim.y, dim.z);
  if (use_color) boxMarker.color = color;
  boxMarker.type = visualization_msgs::Marker::CUBE;
  boxMarker.scale.x = dim.x;
  boxMarker.scale.y = dim.y;
  boxMarker.scale.z = dim.z;
  boxMarker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back(boxMarker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

visualization_msgs::InteractiveMarkerControl UrdfModelMarker::makeSphereMarkerControl(const geometry_msgs::PoseStamped &stamped, double rad, const std_msgs::ColorRGBA &color, bool use_color) {
  visualization_msgs::Marker sphereMarker;

  if (use_color) sphereMarker.color = color;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  sphereMarker.scale.x = rad * 2;
  sphereMarker.scale.y = rad * 2;
  sphereMarker.scale.z = rad * 2;
  sphereMarker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back(sphereMarker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

void UrdfModelMarker::publishJointState() {
  getJointState();
  pub_joint_state_.publish(joint_state_);
}

void UrdfModelMarker::getJointState() {
  sensor_msgs::JointState new_joint_state;
  joint_state_ = new_joint_state;
  joint_state_.header.stamp = ros::Time::now();
  getJointState(model->getRoot());
}

void UrdfModelMarker::getJointState(LinkConstSharedPtr link)
{
  string link_frame_name_ =  tf_prefix_ + link->name;
  JointSharedPtr parent_joint = link->parent_joint;
  if (parent_joint != NULL) {
    KDL::Frame initialFrame;
    KDL::Frame presentFrame;
    KDL::Rotation rot;
    KDL::Vector rotVec;
    KDL::Vector jointVec;
    double jointAngle;
    double jointAngleAllRange;
    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
    {
      linkProperty *link_property = &linkMarkerMap[link_frame_name_];
      tf::poseMsgToKDL (link_property->initial_pose, initialFrame);
      tf::poseMsgToKDL (link_property->pose, presentFrame);
      rot = initialFrame.M.Inverse() * presentFrame.M;
      jointAngle = rot.GetRotAngle(rotVec);
      jointVec = KDL::Vector(link_property->joint_axis.x,
                             link_property->joint_axis.y,
                             link_property->joint_axis.z);
      if (KDL::dot(rotVec,jointVec) < 0) {
        jointAngle = - jointAngle;
      }
      if (link_property->joint_angle > M_PI/2 && jointAngle < -M_PI/2) {
        link_property->rotation_count += 1;
      }
      else if (link_property->joint_angle < -M_PI/2 && jointAngle > M_PI/2) {
        link_property->rotation_count -= 1;
      }
      link_property->joint_angle = jointAngle;
      jointAngleAllRange = jointAngle + link_property->rotation_count * M_PI * 2;

      if (parent_joint->type == Joint::REVOLUTE && parent_joint->limits != NULL) {
        bool changeMarkerAngle = false;
        if (jointAngleAllRange < parent_joint->limits->lower) {
          jointAngleAllRange = parent_joint->limits->lower + 0.001;
          changeMarkerAngle = true;
        }
        if (jointAngleAllRange > parent_joint->limits->upper) {
          jointAngleAllRange = parent_joint->limits->upper - 0.001;
          changeMarkerAngle = true;
        }

        if (changeMarkerAngle) {
          setJointAngle(link, jointAngleAllRange);
        }
      }

      joint_state_.position.push_back(jointAngleAllRange);
      joint_state_.name.push_back(parent_joint->name);
      break;
    }
    case Joint::PRISMATIC:
    {
      KDL::Vector pos;
      linkProperty *link_property = &linkMarkerMap[link_frame_name_];
      tf::poseMsgToKDL (link_property->initial_pose, initialFrame);
      tf::poseMsgToKDL (link_property->pose, presentFrame);

      pos = presentFrame.p - initialFrame.p;

      jointVec = KDL::Vector(link_property->joint_axis.x,
                             link_property->joint_axis.y,
                             link_property->joint_axis.z);
      jointVec = jointVec / jointVec.Norm(); // normalize vector
      jointAngle = KDL::dot(jointVec, pos);

      link_property->joint_angle = jointAngle;
      jointAngleAllRange = jointAngle;

      if (parent_joint->type == Joint::PRISMATIC && parent_joint->limits != NULL) {
        bool changeMarkerAngle = false;
        if (jointAngleAllRange < parent_joint->limits->lower) {
          jointAngleAllRange = parent_joint->limits->lower + 0.003;
          changeMarkerAngle = true;
        }
        if (jointAngleAllRange > parent_joint->limits->upper) {
          jointAngleAllRange = parent_joint->limits->upper - 0.003;
          changeMarkerAngle = true;
        }
        if (changeMarkerAngle) {
          setJointAngle(link, jointAngleAllRange);
        }
      }

      joint_state_.position.push_back(jointAngleAllRange);
      joint_state_.name.push_back(parent_joint->name);
      break;
    }
    case Joint::FIXED:
      break;
    default:
      break;
    }
    server_->applyChanges();
  }

  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    getJointState(*child);
  }
  return;
}

void UrdfModelMarker::setJointAngle(LinkConstSharedPtr link, double joint_angle) {
  string link_frame_name_ =  tf_prefix_ + link->name;
  JointSharedPtr parent_joint = link->parent_joint;

  if (parent_joint == NULL) {
    return;
  }

  KDL::Frame initialFrame;
  KDL::Frame presentFrame;
  KDL::Rotation rot;
  KDL::Vector rotVec;
  KDL::Vector jointVec;

  std_msgs::Header link_header;

  int rotation_count = 0;
  
  switch(parent_joint->type) {
  case Joint::REVOLUTE:
  case Joint::CONTINUOUS:
  {
    if (joint_angle > M_PI) {
      rotation_count = (int)((joint_angle + M_PI) / (M_PI * 2));
      joint_angle -= rotation_count * M_PI * 2;
    }
    else if (joint_angle < -M_PI) {
      rotation_count = (int)((- joint_angle + M_PI) / (M_PI * 2));
      joint_angle -= rotation_count * M_PI * 2;
    }
    linkProperty *link_property = &linkMarkerMap[link_frame_name_];
    link_property->joint_angle = joint_angle;
    link_property->rotation_count = rotation_count;

    tf::poseMsgToKDL (link_property->initial_pose, initialFrame);
    tf::poseMsgToKDL (link_property->initial_pose, presentFrame);
    jointVec = KDL::Vector(link_property->joint_axis.x,
                           link_property->joint_axis.y,
                           link_property->joint_axis.z);

    presentFrame.M = KDL::Rotation::Rot(jointVec, joint_angle) * initialFrame.M;
    tf::poseKDLToMsg(presentFrame, link_property->pose);

    break;
  }
  case Joint::PRISMATIC:
  {
    linkProperty *link_property = &linkMarkerMap[link_frame_name_];
    link_property->joint_angle = joint_angle;
    link_property->rotation_count = rotation_count;
    tf::poseMsgToKDL (link_property->initial_pose, initialFrame);
    tf::poseMsgToKDL (link_property->initial_pose, presentFrame);
    jointVec = KDL::Vector(link_property->joint_axis.x,
                           link_property->joint_axis.y,
                           link_property->joint_axis.z);
    jointVec = jointVec / jointVec.Norm(); // normalize vector
    presentFrame.p = joint_angle * jointVec + initialFrame.p;
    tf::poseKDLToMsg(presentFrame, link_property->pose);
    break;
  }
  default:
    break;
  }

  link_header.stamp = ros::Time(0);
  link_header.frame_id = linkMarkerMap[link_frame_name_].frame_id;

  server_->setPose(link_frame_name_, linkMarkerMap[link_frame_name_].pose, link_header);
  //server_->applyChanges();
  callSetDynamicTf(linkMarkerMap[link_frame_name_].frame_id, link_frame_name_, Pose2Transform(linkMarkerMap[link_frame_name_].pose));
}

void UrdfModelMarker::setJointState(LinkConstSharedPtr link, const sensor_msgs::JointStateConstPtr &js)
{
  string link_frame_name_ =  tf_prefix_ + link->name;
  JointSharedPtr parent_joint = link->parent_joint;
  if (parent_joint != NULL) {
    KDL::Frame initialFrame;
    KDL::Frame presentFrame;
    KDL::Rotation rot;
    KDL::Vector rotVec;
    KDL::Vector jointVec;
    double jointAngle;
    bool changeAngle = false;
    std_msgs::Header link_header;
    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      for(int i=0; i< js->name.size(); i++) {
        if (js->name[i] == parent_joint->name) {
          jointAngle = js->position[i];
          changeAngle = true;
          break;
        }
      }
      if (!changeAngle) {
        break;
      }
      setJointAngle(link, jointAngle);
      break;
    case Joint::PRISMATIC:
      for(int i=0; i< js->name.size(); i++) {
        if (js->name[i] == parent_joint->name) {
          jointAngle = js->position[i];
          changeAngle = true;
          break;
        }
      }
      if (!changeAngle) {
        break;
      }
      setJointAngle(link, jointAngle);
      break;
    default:
      break;
    }
  }
  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    setJointState(*child, js);
  }
  return;
}

geometry_msgs::Pose UrdfModelMarker::getRootPose(geometry_msgs::Pose pose) {
  KDL::Frame pose_frame, offset_frame;
  tf::poseMsgToKDL(pose, pose_frame);
  tf::poseMsgToKDL(root_offset_, offset_frame);
  pose_frame = pose_frame * offset_frame.Inverse();
  tf::poseKDLToMsg(pose_frame, pose);
  return pose;
}

geometry_msgs::PoseStamped UrdfModelMarker::getOriginPoseStamped() {
  geometry_msgs::PoseStamped ps;
  geometry_msgs::Pose pose;
  pose = root_pose_;
  KDL::Frame pose_frame, offset_frame;
  tf::poseMsgToKDL(pose, pose_frame);
  tf::poseMsgToKDL(root_offset_, offset_frame);
  pose_frame = pose_frame * offset_frame;
  tf::poseKDLToMsg(pose_frame, pose);
  ps.pose = pose;
  ps.header.frame_id = frame_id_;
  ps.header.stamp = init_stamp_;
  return ps;
}


void UrdfModelMarker::setOriginalPose(LinkConstSharedPtr link)
{
  string link_frame_name_ =  tf_prefix_ + link->name;
  linkMarkerMap[link_frame_name_].origin =  linkMarkerMap[link_frame_name_].pose;
  
  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    setOriginalPose(*child);
  }
}

void UrdfModelMarker::addChildLinkNames(LinkConstSharedPtr link, bool root, bool init) {
  //addChildLinkNames(link, root, init, false, 0);
  addChildLinkNames(link, root, init, use_visible_color_, 0);
}

void UrdfModelMarker::addChildLinkNames(LinkConstSharedPtr link, bool root, bool init, bool use_color, int color_index)
{
  geometry_msgs::PoseStamped ps;

  double scale_factor = 1.02;
  string link_frame_name_ =  tf_prefix_ + link->name;
  string parent_link_frame_name_;
  ROS_INFO("link_frame_name: %s", link_frame_name_.c_str());
  if (root) {
    parent_link_frame_name_ = frame_id_;
    //ps.pose = root_pose_;
    ps.pose = getRootPose(root_pose_);
  }
  else {
    parent_link_frame_name_ = link->parent_joint->parent_link_name;
    parent_link_frame_name_ = tf_prefix_ + parent_link_frame_name_;
    ps.pose = UrdfPose2Pose(link->parent_joint->parent_to_joint_origin_transform);
  }
  ps.header.frame_id =  parent_link_frame_name_;
  ps.header.stamp = ros::Time(0);

  //initialize linkProperty
  if (init) {
    callSetDynamicTf(parent_link_frame_name_, link_frame_name_, Pose2Transform(ps.pose));
    linkProperty lp;
    lp.pose = ps.pose;
    lp.origin = ps.pose;
    lp.initial_pose = ps.pose;
    if (link->parent_joint !=NULL) {
      lp.joint_axis = link->parent_joint->axis;
    }
    lp.joint_angle = 0;
    lp.rotation_count=0;
    if (link->parent_joint !=NULL && link->parent_joint->type == Joint::FIXED) {
      lp.movable_link = linkMarkerMap[parent_link_frame_name_].movable_link;
    }
    else {
      lp.movable_link = link_frame_name_;
    }

    linkMarkerMap.insert(map<string, linkProperty>::value_type(link_frame_name_, lp));
  }

  linkMarkerMap[link_frame_name_].frame_id = parent_link_frame_name_;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;

  int_marker.name = link_frame_name_;
  if (root) {
    int_marker.description = model_description_;
  }
  int_marker.scale = 1.0;
  int_marker.pose = ps.pose;


  if (!init && !root) {
    visualization_msgs::InteractiveMarker old_marker;
    if (server_->get(link_frame_name_, old_marker)) {
      int_marker.pose = old_marker.pose;
    }
  }


  //hide marker
  if (!linkMarkerMap[link_frame_name_].displayModelMarker) {
    server_->erase(link_frame_name_);
    server_->erase(tf_prefix_ + link->name + "/grasp"); //grasp marker
  }
  else {

    //move Marker
    if (linkMarkerMap[link_frame_name_].displayMoveMarker) {
      addMoveMarkerControl(int_marker, link, root);
    }
    //model Mesh Marker
    std_msgs::ColorRGBA color;
    if (mode_ == "visualization") {
      color.r = (double)0xFF / 0xFF;
      color.g = (double)0xFF / 0xFF;
      color.b = (double)0x00 / 0xFF;
      color.a = 0.5;
    }
    else {
      switch(color_index %3) {
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

    //link_array
    std::vector<VisualSharedPtr > visual_array;
    if (link->visual_array.size() != 0) {
      visual_array = link->visual_array;
    }
    else if (link->visual.get() != NULL) {
      visual_array.push_back(link->visual);
    }
    for(int i=0; i<visual_array.size(); i++) {
      VisualSharedPtr link_visual = visual_array[i];
      if (link_visual.get() != NULL && link_visual->geometry.get() != NULL) {
        visualization_msgs::InteractiveMarkerControl meshControl;
        if (link_visual->geometry->type == Geometry::MESH) {
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
          MeshConstSharedPtr mesh = std::static_pointer_cast<const Mesh>(link_visual->geometry);
#else
          MeshConstSharedPtr mesh = boost::static_pointer_cast<const Mesh>(link_visual->geometry);
#endif
          string model_mesh_ = mesh->filename;
          if (linkMarkerMap[link_frame_name_].mesh_file == "") {
            model_mesh_ = getRosPathFromModelPath(model_mesh_);
            linkMarkerMap[link_frame_name_].mesh_file = model_mesh_;
          }
          else {
            model_mesh_ = linkMarkerMap[link_frame_name_].mesh_file;
          }
          ps.pose = UrdfPose2Pose(link_visual->origin);
          ROS_DEBUG_STREAM("mesh_file:" << model_mesh_);
          geometry_msgs::Vector3 mesh_scale;
          mesh_scale.x = mesh->scale.x;
          mesh_scale.y = mesh->scale.y;
          mesh_scale.z = mesh->scale.z;
          ROS_INFO("make mesh marker from %s", model_mesh_.c_str());
          if (use_color) {
            meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale, color);
          }
          else {
            meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale);
          }
        }
        else if (link_visual->geometry->type == Geometry::CYLINDER) {
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
          CylinderConstSharedPtr cylinder = std::static_pointer_cast<const Cylinder>(link_visual->geometry);
#else
          CylinderConstSharedPtr cylinder = boost::static_pointer_cast<const Cylinder>(link_visual->geometry);
#endif
          ps.pose = UrdfPose2Pose(link_visual->origin);
          double length = cylinder->length;
          double radius = cylinder->radius;
          ROS_INFO_STREAM("cylinder " << link->name << ", length =  " << length << ", radius " << radius);
          if (use_color) {
            meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
          }
          else {
            meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
          }
        }
        else if (link_visual->geometry->type == Geometry::BOX) {
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
          BoxConstSharedPtr box = std::static_pointer_cast<const Box>(link_visual->geometry);
#else
          BoxConstSharedPtr box = boost::static_pointer_cast<const Box>(link_visual->geometry);
#endif
          ps.pose = UrdfPose2Pose(link_visual->origin);
          Vector3 dim = box->dim;
          ROS_INFO_STREAM("box " << link->name << ", dim =  " << dim.x << ", " << dim.y << ", " << dim.z);
          if (use_color) {
            meshControl = makeBoxMarkerControl(ps, dim, color, true);
          }
          else {
            meshControl = makeBoxMarkerControl(ps, dim, color, true);
          }
        }
        else if (link_visual->geometry->type == Geometry::SPHERE) {
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
          SphereConstSharedPtr sphere = std::static_pointer_cast<const Sphere>(link_visual->geometry);
#else
          SphereConstSharedPtr sphere = boost::static_pointer_cast<const Sphere>(link_visual->geometry);
#endif
          ps.pose = UrdfPose2Pose(link_visual->origin);
          double rad = sphere->radius;
          if (use_color) {
            meshControl = makeSphereMarkerControl(ps, rad, color, true);
          }
          else {
            meshControl = makeSphereMarkerControl(ps, rad, color, true);
          }
        }
        int_marker.controls.push_back(meshControl);

        server_->insert(int_marker);
        server_->setCallback(int_marker.name,
                              boost::bind(&UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_));

        model_menu_.apply(*server_, link_frame_name_);

      }
      else {
        JointSharedPtr parent_joint = link->parent_joint;
        if (parent_joint != NULL) {
          if (parent_joint->type==Joint::REVOLUTE || parent_joint->type==Joint::REVOLUTE) {
            addInvisibleMeshMarkerControl(int_marker, link, color);
            server_->insert(int_marker);
            server_->setCallback(int_marker.name,
                                  boost::bind(&UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_));
            model_menu_.apply(*server_, link_frame_name_);
          }
        }
      }
    }
    if (!robot_mode_) {
      //add Grasp Point Marker
      if (linkMarkerMap[link_frame_name_].gp.displayGraspPoint) {
        visualization_msgs::InteractiveMarker grasp_int_marker;
        double grasp_scale_factor = 1.02;
        string grasp_link_frame_name_ = tf_prefix_ + link->name + "/grasp";
        string grasp_parent_link_frame_name_ = tf_prefix_ + link->name;

        geometry_msgs::PoseStamped grasp_ps;
        grasp_ps.pose = linkMarkerMap[link_frame_name_].gp.pose;
        grasp_ps.header.frame_id =  grasp_parent_link_frame_name_;

        grasp_int_marker.header = grasp_ps.header;
        grasp_int_marker.name = grasp_link_frame_name_;
        grasp_int_marker.scale = grasp_scale_factor;
        grasp_int_marker.pose = grasp_ps.pose;

        addGraspPointControl(grasp_int_marker, link_frame_name_);

        server_->insert(grasp_int_marker);
        server_->setCallback(grasp_int_marker.name,
                              boost::bind(&UrdfModelMarker::graspPoint_feedback, this, _1, link_frame_name_));

      }
    }
  }

  //initialize JointState
  if (init) {
    if (!root && initial_pose_map_.count(link->parent_joint->name) != 0) {
      setJointAngle(link, initial_pose_map_[link->parent_joint->name]);
    }
  }

  //  cout << "Link:" << link->name << endl;

  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    addChildLinkNames(*child, false, init, use_color, color_index + 1);
  }
  if (root) {
    server_->applyChanges();
  }
}



UrdfModelMarker::UrdfModelMarker ()
{}

UrdfModelMarker::UrdfModelMarker (string model_name, string model_description, string model_file, string frame_id, geometry_msgs::PoseStamped root_pose, geometry_msgs::Pose root_offset, double scale_factor, string mode, bool robot_mode, bool registration, vector<string> fixed_link, bool use_robot_description, bool use_visible_color, map<string, double> initial_pose_map, int index,  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : nh_(), pnh_("~"), tfl_(nh_),use_dynamic_tf_(true), is_joint_states_locked_(false) {
  diagnostic_updater_.reset(new diagnostic_updater::Updater);
  diagnostic_updater_->setHardwareID(ros::this_node::getName());
  diagnostic_updater_->add("Modeling Stats", boost::bind(&UrdfModelMarker::updateDiagnostic, this, _1));

  pnh_.param("server_name", server_name, std::string (""));

  if (server_name == "") {
    server_name = ros::this_node::getName();
  }

  pnh_.getParam("use_dynamic_tf", use_dynamic_tf_);
  if (use_dynamic_tf_) {
    ros::service::waitForService("set_dynamic_tf", -1);
    dynamic_tf_publisher_client = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf", true);
    ros::service::waitForService("publish_tf", -1);
    dynamic_tf_publisher_publish_tf_client = nh_.serviceClient<std_srvs::Empty>("publish_tf", true);
  }
  ROS_INFO_STREAM("use_dynamic_tf_ is " << use_dynamic_tf_);

  if (index != -1) {
    stringstream ss;
    ss << model_name << index;
    model_name_ = ss.str();
  }
  else {
    model_name_ = model_name;
  }

  model_description_ = model_description;
  server_ = server;
  model_file_ = model_file;
  frame_id_ = frame_id;
  root_offset_ = root_offset;
  fixed_link_offset_ = root_offset;
  root_pose_ = root_pose.pose;
  init_stamp_ = root_pose.header.stamp;
  scale_factor_ = scale_factor;
  robot_mode_ = robot_mode;
  registration_ = registration;
  mode_ = mode;
  fixed_link_ = fixed_link;
  use_robot_description_ = use_robot_description;
  use_visible_color_ = use_visible_color;
  tf_prefix_ = server_name + "/" + model_name_ + "/";
  initial_pose_map_ = initial_pose_map;
  index_ = index;

  pub_ =  pnh_.advertise<jsk_interactive_marker::MarkerPose> ("pose", 1);
  pub_move_ =  pnh_.advertise<jsk_interactive_marker::MarkerMenu> ("marker_menu", 1);
  pub_move_object_ =  pnh_.advertise<jsk_interactive_marker::MoveObject> ("move_object", 1);
  pub_move_model_ =  pnh_.advertise<jsk_interactive_marker::MoveModel> ("move_model", 1);
  pub_selected_ =  pnh_.advertise<geometry_msgs::PoseStamped> (model_name + "/selected", 1);
  pub_selected_index_ =  pnh_.advertise<jsk_recognition_msgs::Int32Stamped> (model_name + "/selected_index", 1);
  pub_joint_state_ =  pnh_.advertise<sensor_msgs::JointState> (model_name_ + "/joint_states", 1);
  
  sub_set_root_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped> (model_name_ + "/set_pose", 1, boost::bind(&UrdfModelMarker::setRootPoseCB, this, _1));
  sub_reset_joints_ = pnh_.subscribe<sensor_msgs::JointState> (model_name_ + "/reset_joint_states", 1, boost::bind(&UrdfModelMarker::resetJointStatesCB, this, _1, false));
  sub_reset_joints_and_root_ = pnh_.subscribe<sensor_msgs::JointState> (model_name_ + "/reset_joint_states_and_root", 1, boost::bind(&UrdfModelMarker::resetJointStatesCB, this, _1, true));
  
  hide_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/hide_marker", 1, boost::bind(&UrdfModelMarker::hideModelMarkerCB, this, _1));
  show_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/show_marker", 1, boost::bind(&UrdfModelMarker::showModelMarkerCB, this, _1));
  sub_set_urdf_ = pnh_.subscribe<std_msgs::String>(model_name_ + "/set_urdf", 1, boost::bind(&UrdfModelMarker::setUrdfCB, this, _1));

  show_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/reset_root_pose", 1, boost::bind(&UrdfModelMarker::resetBaseMsgCB, this, _1));

  pub_base_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>(model_name_ + "/base_pose", 1);



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
  if (mode_ == "") {
    if (registration_) {
      mode_ = "registration";
    }
    else if (robot_mode_) {
      mode_ = "robot";
    }
    else {
      mode_ = "model";
    }
  }
  serv_lock_joint_states_ = pnh_.advertiseService("lock_joint_states",
                                                  &UrdfModelMarker::lockJointStates,
                                                  this);
  serv_unlock_joint_states_ = pnh_.advertiseService("unlock_joint_states",
                                                    &UrdfModelMarker::unlockJointStates,
                                                    this);

  if (mode_ == "registration") {
    model_menu_.insert("Registration",
                        boost::bind(&UrdfModelMarker::registrationCB, this, _1));
  }
  else if (mode_ == "visualization") {

  }
  else if (mode_ == "robot") {
    interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
    sub_menu_move_ = model_menu_.insert("Move");
    model_menu_.insert(sub_menu_move_, "Joint",
                        boost::bind(&UrdfModelMarker::jointMoveCB, this, _1));
    model_menu_.insert(sub_menu_move_, "Base",
                        boost::bind(&UrdfModelMarker::publishMarkerMenu, this, _1, jsk_interactive_marker::MarkerMenu::MOVE));

    interactive_markers::MenuHandler::EntryHandle sub_menu_reset_;
    sub_menu_reset_ = model_menu_.insert("Reset Marker Pose");
    model_menu_.insert(sub_menu_reset_, "Joint",
                        boost::bind(&UrdfModelMarker::resetMarkerCB, this, _1));
    model_menu_.insert(sub_menu_reset_, "Base",
                        boost::bind(&UrdfModelMarker::resetBaseMarkerCB, this, _1));


    interactive_markers::MenuHandler::EntryHandle sub_menu_pose_;
    sub_menu_pose_ = model_menu_.insert("Special Pose");

    model_menu_.insert(sub_menu_pose_, "Stand Pose",
                        boost::bind(&UrdfModelMarker::publishMarkerMenu, this, _1, 100));

    model_menu_.insert(sub_menu_pose_, "Manip Pose",
                        boost::bind(&UrdfModelMarker::publishMarkerMenu, this, _1, 101));

    //model_menu_.insert("Reset Marker Pose",
    //boost::bind(&UrdfModelMarker::resetMarkerCB, this, _1));


    model_menu_.insert("Hide Marker" ,
                        boost::bind(&UrdfModelMarker::hideMarkerCB, this, _1));
    model_menu_.insert("Hide All Marker" ,
                        boost::bind(&UrdfModelMarker::hideAllMarkerCB, this, _1));


  }
  else if (mode_ == "model") {
    model_menu_.insert("Grasp Point",
                        boost::bind(&UrdfModelMarker::graspPointCB, this, _1));
    model_menu_.insert("Move",
                        boost::bind(&UrdfModelMarker::moveCB, this, _1));
    model_menu_.insert("Set as present pose",
                        boost::bind(&UrdfModelMarker::setPoseCB, this, _1));
    model_menu_.insert("Hide Marker" ,
                        boost::bind(&UrdfModelMarker::hideMarkerCB, this, _1));
    model_menu_.insert("Hide All Marker" ,
                        boost::bind(&UrdfModelMarker::hideAllMarkerCB, this, _1));
  }

  // get the entire file
  std::string xml_string;

  if (use_robot_description_) {
    ROS_INFO("loading robot_description from parameter %s", model_file_.c_str());
    nh_.getParam(model_file_, xml_string);

  }
  else {
    ROS_INFO_STREAM("loading model_file: " << model_file_);
    model_file_ = getFilePathFromRosPath(model_file_);
    model_file_ = getFullPathFromModelPath(model_file_);
    try {
      if (!boost::filesystem::exists(model_file_.c_str())) {
        ROS_ERROR("%s does not exists", model_file_.c_str());
      }
      else {
        std::fstream xml_file(model_file_.c_str(), std::fstream::in);
        while (xml_file.good())
        {
          std::string line;
          std::getline(xml_file, line);
          xml_string += (line + "\n");
        }
        xml_file.close();
        ROS_INFO_STREAM("finish loading model_file: " << model_file_);
      }
    }
    catch (...) {
      ROS_ERROR("model or mesh not found: %s", model_file_.c_str());
      ROS_ERROR("Please check GAZEBO_MODEL_PATH");
    }
  }
  ROS_INFO("xml_string is %lu size", xml_string.length());
  model = parseURDF(xml_string);

  if (!model) {
    ROS_ERROR("ERROR: Model Parsing the xml failed");
    return;
  }
  else {
    ROS_INFO("model name is %s", model->getName().c_str());
  }
  if (mode_ == "robot") {
    resetRobotBase();
  }
  addChildLinkNames(model->getRoot(), true, true);

  publishJointState();
  setPoseCB(); //init joint_state_origin

  resetRootForVisualization();
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  //callSetDynamicTf(parent_frame_id, frame_id, Pose2Transform(pose));
  return;
}

bool UrdfModelMarker::lockJointStates(std_srvs::EmptyRequest& req,
                                      std_srvs::EmptyRequest& res)
{
  boost::mutex::scoped_lock lock(joint_states_mutex_);
  is_joint_states_locked_ = true;
  return true;
}

bool UrdfModelMarker::unlockJointStates(std_srvs::EmptyRequest& req,
                                        std_srvs::EmptyRequest& res)
{
  boost::mutex::scoped_lock lock(joint_states_mutex_);
  is_joint_states_locked_ = false;
  return true;
}


void UrdfModelMarker::updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "UrdfModelMarker running");
  stat.add("Time to set dynamic tf (Avg.)",
           dynamic_tf_check_time_acc_.mean());
  stat.add("Time to set dynamic tf (Max)",
           dynamic_tf_check_time_acc_.max());
  stat.add("Time to set dynamic tf (Min)",
           dynamic_tf_check_time_acc_.min());
  stat.add("Time to set dynamic tf (Var.)",
           dynamic_tf_check_time_acc_.variance());

  stat.add("Time to set joint states (Avg.)",
           reset_joint_states_check_time_acc_.mean());
  stat.add("Time to set joint states (Max)",
           reset_joint_states_check_time_acc_.max());
  stat.add("Time to set joint states (Min)",
           reset_joint_states_check_time_acc_.min());
  stat.add("Time to set joint states (Var.)",
           reset_joint_states_check_time_acc_.variance());

}
