#include <jsk_transformable_interactive_marker/transformable_interactive_server.h>

using namespace jsk_transformable_interactive_marker;

TransformableInteractiveServer::TransformableInteractiveServer():n_(new ros::NodeHandle){
  n_->param("torus_udiv", torus_udiv_, 20);
  n_->param("torus_vdiv", torus_vdiv_, 20);

  setpose_sub_ = n_->subscribe("set_pose", 1, &TransformableInteractiveServer::setPose, this);
 setcolor_sub_ = n_->subscribe("set_color", 1, &TransformableInteractiveServer::setColor, this);

  set_r_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);
  set_sm_r_sub_ = n_->subscribe("set_small_radius", 1, &TransformableInteractiveServer::setSmallRadius, this);
  set_x_sub_ = n_->subscribe("set_x", 1, &TransformableInteractiveServer::setX, this);
  set_y_sub_ = n_->subscribe("set_y", 1, &TransformableInteractiveServer::setY, this);
  set_z_sub_ = n_->subscribe("set_z", 1, &TransformableInteractiveServer::setZ, this);

  addpose_sub_ = n_->subscribe("add_pose", 1, &TransformableInteractiveServer::addPose, this);

  setrad_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);

  focus_text_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_name", 1);
  focus_pose_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_pose", 1);

  get_pose_srv_ = n_->advertiseService("get_pose", &TransformableInteractiveServer::getPoseService, this);

  server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");

  insertNewBox(std::string("/map"), std::string("my_marker"), std::string("my_marker"));
  insertNewCylinder(std::string("/map"), std::string("my_marker2"), std::string("my_marker2"));
  insertNewTorus(std::string("/map"), std::string("my_marker3"), std::string("my_marker3"));
}

TransformableInteractiveServer::~TransformableInteractiveServer()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    delete itpairstri->second;
  }
  transformable_objects_map_.clear();
}

void TransformableInteractiveServer::processFeedback(
                                                     const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      focus_object_marker_name_ = feedback->marker_name;
      focusTextPublish();
      focusPosePublish();
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      TransformableObject* tobject = transformable_objects_map_[feedback->marker_name.c_str()];
      if(tobject){
        tobject->setPose(feedback->pose);
      }else
        ROS_ERROR("Invalid ObjectId Request Received %s", feedback->marker_name.c_str());
      break;
    }
}

void TransformableInteractiveServer::setColor(std_msgs::ColorRGBA msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setRGBA(msg.r, msg.g, msg.b, msg.a);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setRadius(std_msgs::Float32 msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setRadius(msg)){
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::setSmallRadius(std_msgs::Float32 msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setSmallRadius(msg)){
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::setX(std_msgs::Float32 msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setX(msg)){
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::setY(std_msgs::Float32 msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setY(msg)){
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::setZ(std_msgs::Float32 msg)
{
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setZ(msg)){
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::updateTransformableObject(TransformableObject* tobject)
{
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  server_->applyChanges();
}

void TransformableInteractiveServer::setPose(geometry_msgs::PoseStamped msg){
  TransformableObject* tobject =  transformable_objects_map_[focus_object_marker_name_];
  tobject->setPose(msg.pose);
  server_->setPose(focus_object_marker_name_, msg.pose, msg.header);
  server_->applyChanges();
}

bool TransformableInteractiveServer::getPoseService(jsk_transformable_interactive_marker::GetPose::Request &req,jsk_transformable_interactive_marker::GetPose::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.empty()){
    if(tobject = transformable_objects_map_[focus_object_marker_name_])
      res.pose = tobject->getPose();
  }else{
    if(tobject = transformable_objects_map_[req.target_name])
      res.pose = tobject->getPose();
  }
}


void TransformableInteractiveServer::addPose(geometry_msgs::Pose msg){
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::focusTextPublish(){
  jsk_rviz_plugins::OverlayText focus_text;
  focus_text.text = focus_object_marker_name_;
  focus_text.top = 0;
  focus_text.left = 0;
  focus_text.width = 300;
  focus_text.height = 50;
  focus_text.bg_color.r = 0.9;
  focus_text.bg_color.b = 0.9;
  focus_text.bg_color.g = 0.9;
  focus_text.bg_color.a = 0.1;
  focus_text.fg_color.r = 0.3;
  focus_text.fg_color.g = 0.3;
  focus_text.fg_color.b = 0.8;
  focus_text.fg_color.a = 1;
  focus_text.line_width = 1;
  focus_text.text_size = 30;
  focus_text_pub_.publish(focus_text);
}

void TransformableInteractiveServer::focusPosePublish(){
  geometry_msgs::Pose target_pose = transformable_objects_map_[focus_object_marker_name_]->getPose();
  std::stringstream ss;
  ss << "Pos x: " << target_pose.position.x  << " y: " << target_pose.position.y << " z: " << target_pose.position.z
     << std::endl
     << "Ori x: " << target_pose.orientation.x << " y: " << target_pose.orientation.y << " z: " << target_pose.orientation.z << " w: " << target_pose.orientation.w;

  jsk_rviz_plugins::OverlayText focus_pose;
  focus_pose.text = ss.str();
  focus_pose.top = 50;
  focus_pose.left = 0;
  focus_pose.width = 500;
  focus_pose.height = 50;
  focus_pose.bg_color.r = 0.9;
  focus_pose.bg_color.b = 0.9;
  focus_pose.bg_color.g = 0.9;
  focus_pose.bg_color.a = 0.1;
  focus_pose.fg_color.r = 0.8;
  focus_pose.fg_color.g = 0.3;
  focus_pose.fg_color.b = 0.3;
  focus_pose.fg_color.a = 1;
  focus_pose.line_width = 1;
  focus_pose.text_size = 15;
  focus_pose_pub_.publish(focus_pose);
}

void TransformableInteractiveServer::insertNewBox(std::string frame_id, std::string name, std::string description)
{
  TransformableBox* transformable_box = new TransformableBox(0.45, 0.45, 0.45, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_box, name);
}

void TransformableInteractiveServer::insertNewCylinder( std::string frame_id, std::string name, std::string description)
{
  TransformableCylinder* transformable_cylinder = new TransformableCylinder(0.45, 0.45, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_cylinder, name);
}

void TransformableInteractiveServer::insertNewTorus( std::string frame_id, std::string name, std::string description)
{
  TransformableTorus* transformable_torus = new TransformableTorus(0.45, 0.2, torus_udiv_, torus_vdiv_, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_torus, name);
}

void TransformableInteractiveServer::insertNewObject( TransformableObject* tobject , std::string name )
{
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  transformable_objects_map_[name] = tobject;
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  server_->applyChanges();

  focus_object_marker_name_ = name;
}

void TransformableInteractiveServer::run(){
  ros::spin();
}
