#include <jsk_interactive_marker/transformable_interactive_server.h>
#include <jsk_topic_tools/log_utils.h>

using namespace jsk_interactive_marker;

TransformableInteractiveServer::TransformableInteractiveServer():n_(new ros::NodeHandle("~")){
  n_->param("interactive_manipulator_orientation", interactive_manipulator_orientation_ , 0);
  n_->param("torus_udiv", torus_udiv_, 20);
  n_->param("torus_vdiv", torus_vdiv_, 20);
  n_->param("strict_tf", strict_tf_, false);
  tf_listener_.reset(new tf::TransformListener);
  setpose_sub_ = n_->subscribe<geometry_msgs::PoseStamped>("set_pose", 1, boost::bind(&TransformableInteractiveServer::setPose, this, _1, false));
  setcontrolpose_sub_ = n_->subscribe<geometry_msgs::PoseStamped>("set_control_pose", 1, boost::bind(&TransformableInteractiveServer::setPose, this, _1, true));
  setcolor_sub_ = n_->subscribe("set_color", 1, &TransformableInteractiveServer::setColor, this);

  set_r_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);
  set_sm_r_sub_ = n_->subscribe("set_small_radius", 1, &TransformableInteractiveServer::setSmallRadius, this);
  set_x_sub_ = n_->subscribe("set_x", 1, &TransformableInteractiveServer::setX, this);
  set_y_sub_ = n_->subscribe("set_y", 1, &TransformableInteractiveServer::setY, this);
  set_z_sub_ = n_->subscribe("set_z", 1, &TransformableInteractiveServer::setZ, this);

  addpose_sub_ = n_->subscribe("add_pose", 1, &TransformableInteractiveServer::addPose, this);
  addpose_relative_sub_ = n_->subscribe("add_pose_relative", 1, &TransformableInteractiveServer::addPoseRelative, this);
  
  setcontrol_relative_sub_ = n_->subscribe("set_control_relative_pose", 1, &TransformableInteractiveServer::setControlRelativePose, this);
  
  setrad_sub_ = n_->subscribe("set_radius", 1, &TransformableInteractiveServer::setRadius, this);

  focus_name_text_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_name_text", 1);
  focus_pose_text_pub_ = n_->advertise<jsk_rviz_plugins::OverlayText>("focus_marker_pose_text", 1);
  focus_object_marker_name_pub_ = n_->advertise<std_msgs::String>("focus_object_marker_name", 1);
  pose_pub_ = n_->advertise<geometry_msgs::PoseStamped>("pose", 1);
  pose_with_name_pub_ = n_->advertise<jsk_interactive_marker::PoseStampedWithName>("pose_with_name", 1);

  get_pose_srv_ = n_->advertiseService<jsk_interactive_marker::GetTransformableMarkerPose::Request, jsk_interactive_marker::GetTransformableMarkerPose::Response>("get_pose", boost::bind(&TransformableInteractiveServer::getPoseService, this, _1, _2, false));
  get_control_pose_srv_ = n_->advertiseService<jsk_interactive_marker::GetTransformableMarkerPose::Request, jsk_interactive_marker::GetTransformableMarkerPose::Response>("get_control_pose", boost::bind(&TransformableInteractiveServer::getPoseService, this, _1, _2, true));
  set_pose_srv_ = n_->advertiseService<jsk_interactive_marker::SetTransformableMarkerPose::Request ,jsk_interactive_marker::SetTransformableMarkerPose::Response>("set_pose", boost::bind(&TransformableInteractiveServer::setPoseService, this, _1, _2, false));
  set_control_pose_srv_ = n_->advertiseService<jsk_interactive_marker::SetTransformableMarkerPose::Request ,jsk_interactive_marker::SetTransformableMarkerPose::Response>("set_control_pose", boost::bind(&TransformableInteractiveServer::setPoseService, this, _1, _2, true));
  get_color_srv_ = n_->advertiseService("get_color", &TransformableInteractiveServer::getColorService, this);
  set_color_srv_ = n_->advertiseService("set_color", &TransformableInteractiveServer::setColorService, this);
  get_focus_srv_ = n_->advertiseService("get_focus", &TransformableInteractiveServer::getFocusService, this);
  set_focus_srv_ = n_->advertiseService("set_focus", &TransformableInteractiveServer::setFocusService, this);
  get_type_srv_ = n_->advertiseService("get_type", &TransformableInteractiveServer::getTypeService, this);
  get_exist_srv_ = n_->advertiseService("get_existence", &TransformableInteractiveServer::getExistenceService, this);
  set_dimensions_srv =  n_->advertiseService("set_dimensions", &TransformableInteractiveServer::setDimensionsService, this);
  get_dimensions_srv =  n_->advertiseService("get_dimensions", &TransformableInteractiveServer::getDimensionsService, this);
  hide_srv_ = n_->advertiseService("hide", &TransformableInteractiveServer::hideService, this);
  show_srv_ = n_->advertiseService("show", &TransformableInteractiveServer::showService, this);
  marker_dimensions_pub_ = n_->advertise<jsk_interactive_marker::MarkerDimensions>("marker_dimensions", 1);
  request_marker_operate_srv_ = n_->advertiseService("request_marker_operate", &TransformableInteractiveServer::requestMarkerOperateService, this);

  config_srv_ = std::make_shared <dynamic_reconfigure::Server<InteractiveSettingConfig> > (*n_);
  dynamic_reconfigure::Server<InteractiveSettingConfig>::CallbackType f =
    boost::bind (&TransformableInteractiveServer::configCallback, this, _1, _2);
  config_srv_->setCallback (f);

  tf_timer = n_->createTimer(ros::Duration(0.05), &TransformableInteractiveServer::tfTimerCallback, this);

  // initialize yaml-menu-handler
  std::string yaml_filename;
  n_->param("yaml_filename", yaml_filename, std::string(""));
  yaml_menu_handler_ptr_ = std::make_shared <YamlMenuHandler> (n_, yaml_filename);
  yaml_menu_handler_ptr_->_menu_handler.insert(
    "enable manipulator",
    boost::bind(&TransformableInteractiveServer::enableInteractiveManipulatorDisplay, this, _1, /*enable=*/true));
  yaml_menu_handler_ptr_->_menu_handler.insert(
    "disable manipulator",
    boost::bind(&TransformableInteractiveServer::enableInteractiveManipulatorDisplay, this, _1, /*enable=*/false));

  bool use_parent_and_child;
  n_->param("use_parent_and_child", use_parent_and_child, false);
  if (use_parent_and_child)
  {
    ROS_INFO("initialize parent and child marker");
    server_ = new jsk_interactive_marker::ParentAndChildInteractiveMarkerServer("simple_marker");
  }
  else
  {
    ROS_INFO("initialize simple marker");
    server_ = new interactive_markers::InteractiveMarkerServer("simple_marker");
  }
}

TransformableInteractiveServer::~TransformableInteractiveServer()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    delete itpairstri->second;
  }
  transformable_objects_map_.clear();
}

void TransformableInteractiveServer::configCallback(InteractiveSettingConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    int interaction_mode = config.interaction_mode;
    for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
      TransformableObject* tobject = itpairstri->second;
      tobject->setInteractiveMarkerSetting(config_);
      updateTransformableObject(tobject);
    }
  }


void TransformableInteractiveServer::processFeedback(
                                                     const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      focus_object_marker_name_ = feedback->marker_name;
      focusTextPublish();
      focusPosePublish();
      focusObjectMarkerNamePublish();
      focusInteractiveManipulatorDisplay();
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      TransformableObject* tobject = transformable_objects_map_[feedback->marker_name.c_str()];
      if(tobject){
        geometry_msgs::PoseStamped input_pose_stamped;
        input_pose_stamped.header = feedback->header;
        input_pose_stamped.pose = feedback->pose;
        setPoseWithTfTransformation(tobject, input_pose_stamped, true);
      }else{
        ROS_ERROR("Invalid ObjectId Request Received %s", feedback->marker_name.c_str());
      }
      focusTextPublish();
      focusPosePublish();
      focusObjectMarkerNamePublish();
      break;
    }
}

void TransformableInteractiveServer::setColor(std_msgs::ColorRGBA msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setRGBA(msg.r, msg.g, msg.b, msg.a);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setRadius(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setSmallRadius(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setSmallRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setX(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setX(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setY(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setY(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setZ(std_msgs::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setZ(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::updateTransformableObject(TransformableObject* tobject)
{
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

void TransformableInteractiveServer::setPose(const geometry_msgs::PoseStampedConstPtr &msg_ptr, bool for_interactive_control){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject =  transformable_objects_map_[focus_object_marker_name_];
  setPoseWithTfTransformation(tobject, *msg_ptr, for_interactive_control);
  std_msgs::Header header = msg_ptr->header;
  header.frame_id = tobject->getFrameId();
  server_->setPose(focus_object_marker_name_, tobject->pose_, header);
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

bool TransformableInteractiveServer::getPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req,jsk_interactive_marker::GetTransformableMarkerPose::Response &res, bool for_interactive_control)
{
  TransformableObject* tobject;
  geometry_msgs::PoseStamped transformed_pose_stamped;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  transformed_pose_stamped.header.stamp = ros::Time::now();
  transformed_pose_stamped.header.frame_id = tobject->frame_id_;
  transformed_pose_stamped.pose = tobject->getPose(for_interactive_control);
  res.pose_stamped = transformed_pose_stamped;
  return true;
}

bool TransformableInteractiveServer::setPoseService(jsk_interactive_marker::SetTransformableMarkerPose::Request &req,jsk_interactive_marker::SetTransformableMarkerPose::Response &res, bool for_interactive_control)
{
  TransformableObject* tobject;
  geometry_msgs::PoseStamped transformed_pose_stamped;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    focus_object_marker_name_ = req.target_name;
    tobject = transformable_objects_map_[req.target_name];
  }
  if(setPoseWithTfTransformation(tobject, req.pose_stamped, for_interactive_control)){
    std_msgs::Header header = req.pose_stamped.header;
    header.frame_id = tobject->getFrameId();
    server_->setPose(focus_object_marker_name_, tobject->pose_, header);
    yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
    server_->applyChanges();
  }
  return true;
}

bool TransformableInteractiveServer::getColorService(jsk_interactive_marker::GetTransformableMarkerColor::Request &req,jsk_interactive_marker::GetTransformableMarkerColor::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  tobject->getRGBA(res.color.r, res.color.g, res.color.b, res.color.a);
  return true;
}

bool TransformableInteractiveServer::setColorService(jsk_interactive_marker::SetTransformableMarkerColor::Request &req,jsk_interactive_marker::SetTransformableMarkerColor::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  tobject->setRGBA(req.color.r, req.color.g, req.color.b, req.color.a);
  updateTransformableObject(tobject);
  return true;
}

bool TransformableInteractiveServer::getFocusService(jsk_interactive_marker::GetTransformableMarkerFocus::Request &req,jsk_interactive_marker::GetTransformableMarkerFocus::Response &res)
{
  res.target_name = focus_object_marker_name_;
  return true;
}

bool TransformableInteractiveServer::setFocusService(jsk_interactive_marker::SetTransformableMarkerFocus::Request &req,jsk_interactive_marker::SetTransformableMarkerFocus::Response &res)
{
  focus_object_marker_name_ = req.target_name;
  focusTextPublish();
  focusPosePublish();
  focusObjectMarkerNamePublish();
  return true;
}

bool TransformableInteractiveServer::getTypeService(jsk_interactive_marker::GetType::Request &req,jsk_interactive_marker::GetType::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
    res.type = tobject->type_;
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
    res.type = tobject->type_;
  }
  return true;
}

bool TransformableInteractiveServer::getExistenceService(jsk_interactive_marker::GetTransformableMarkerExistence::Request &req,jsk_interactive_marker::GetTransformableMarkerExistence::Response &res)
{
  if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) {
    res.existence = false;
  } else {
    res.existence = true;
  }
  return true;
}

bool TransformableInteractiveServer::setDimensionsService(jsk_interactive_marker::SetMarkerDimensions::Request &req,jsk_interactive_marker::SetMarkerDimensions::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->setXYZ(req.dimensions.x, req.dimensions.y, req.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->setRZ(req.dimensions.radius, req.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->setRSR(req.dimensions.radius, req.dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE) {
    }
    publishMarkerDimensions();
    updateTransformableObject(tobject);
  }
  return true;
}

bool TransformableInteractiveServer::getDimensionsService(jsk_interactive_marker::GetMarkerDimensions::Request &req,jsk_interactive_marker::GetMarkerDimensions::Response &res)
{
  TransformableObject* tobject;
  if(req.target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req.target_name) == transformable_objects_map_.end()) { return true; }
    tobject = transformable_objects_map_[req.target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->getXYZ(res.dimensions.x, res.dimensions.y, res.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->getRZ(res.dimensions.radius, res.dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->getRSR(res.dimensions.radius, res.dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE) {
    }

    res.dimensions.type = tobject->getType();
  }
  return true;
}

bool TransformableInteractiveServer::hideService(std_srvs::Empty::Request& req,
                                                 std_srvs::Empty::Response& res)
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); 
       itpairstri != transformable_objects_map_.end();
       ++itpairstri) {
    TransformableObject* tobject = itpairstri->second;
    tobject->setDisplayInteractiveManipulator(false);
    updateTransformableObject(tobject);
  }
  return true;
}

bool TransformableInteractiveServer::showService(std_srvs::Empty::Request& req,
                                                 std_srvs::Empty::Response& res)
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin();
       itpairstri != transformable_objects_map_.end(); 
       ++itpairstri) {
    TransformableObject* tobject = itpairstri->second;
    tobject->setDisplayInteractiveManipulator(true);
    updateTransformableObject(tobject);
  }
  return true;
}
void TransformableInteractiveServer::publishMarkerDimensions()
{
  TransformableObject* tobject;
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  tobject = transformable_objects_map_[focus_object_marker_name_];
  if (tobject) {
    jsk_interactive_marker::MarkerDimensions marker_dimensions;
    if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      tobject->getXYZ(marker_dimensions.x, marker_dimensions.y, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      tobject->getRZ(marker_dimensions.radius, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      tobject->getRSR(marker_dimensions.radius, marker_dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE) {
    }
    marker_dimensions.type = tobject->type_;
    marker_dimensions_pub_.publish(marker_dimensions);
  }
}
  
bool TransformableInteractiveServer::requestMarkerOperateService(jsk_rviz_plugins::RequestMarkerOperate::Request &req,jsk_rviz_plugins::RequestMarkerOperate::Response &res)
{
  switch(req.operate.action){
  case jsk_rviz_plugins::TransformableMarkerOperate::INSERT:
    // validation
    if (req.operate.name.empty()) {
      ROS_ERROR("Non empty name is required to insert object.");
      return false;
    }

    if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      insertNewBox(req.operate.frame_id, req.operate.name, req.operate.description);
    } else if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      insertNewCylinder(req.operate.frame_id, req.operate.name, req.operate.description);
    } else if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      insertNewTorus(req.operate.frame_id, req.operate.name, req.operate.description);
    } else if (req.operate.type == jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE) {
      insertNewMesh(req.operate.frame_id, req.operate.name, req.operate.description, req.operate.mesh_resource, req.operate.mesh_use_embedded_materials);
    }
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASE:
    eraseObject(req.operate.name);
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASEALL:
    eraseAllObject();
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::ERASEFOCUS:
    eraseFocusObject();
    return true;
    break;
  case jsk_rviz_plugins::TransformableMarkerOperate::COPY:
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return true; }
    TransformableObject *tobject = transformable_objects_map_[focus_object_marker_name_], *new_tobject;
    if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::BOX) {
      float x, y, z;
      tobject->getXYZ(x, y, z);
      insertNewBox(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setXYZ(x, y, z);
      new_tobject->setPose(tobject->getPose());
    } else if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::CYLINDER) {
      float r, z;
      tobject->getRZ(r, z);
      insertNewCylinder(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setRZ(r, z);
      new_tobject->setPose(tobject->getPose());
    } else if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::TORUS) {
      float r, sr;
      tobject->getRSR(r, sr);
      insertNewTorus(tobject->frame_id_, req.operate.name, req.operate.description);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setRSR(r, sr);
      new_tobject->setPose(tobject->getPose());
    } else if (tobject->type_ == jsk_rviz_plugins::TransformableMarkerOperate::MESH_RESOURCE) {
      insertNewMesh(tobject->frame_id_, req.operate.name, req.operate.description, req.operate.mesh_resource, req.operate.mesh_use_embedded_materials);
      new_tobject = transformable_objects_map_[req.operate.name];
      new_tobject->setPose(tobject->getPose());
    }
    float r, g, b, a;
    tobject->getRGBA(r, g, b, a);
    new_tobject->setRGBA(r, g, b, a);
    return true;
    break;
  };
  return false;
}

void TransformableInteractiveServer::addPose(geometry_msgs::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,false);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::addPoseRelative(geometry_msgs::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,true);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setControlRelativePose(geometry_msgs::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  geometry_msgs::Pose pose = tobject->getPose(); //reserve marker pose
  tobject->control_offset_pose_ = msg;
  updateTransformableObject(tobject);
  tobject->setPose(pose);
  std_msgs::Header header;
  header.frame_id = tobject->getFrameId();
  header.stamp = ros::Time::now();
  server_->setPose(focus_object_marker_name_, tobject->pose_, header);
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

void TransformableInteractiveServer::enableInteractiveManipulatorDisplay(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
    const bool enable) {
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setDisplayInteractiveManipulator(enable);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::focusInteractiveManipulatorDisplay() {
  for (std::map<string, TransformableObject* >::iterator it = transformable_objects_map_.begin();
        it != transformable_objects_map_.end(); it++) {
    std::string object_name = it->first;
    TransformableObject* tobject = it->second;
    if (config_.display_interactive_manipulator && config_.display_interactive_manipulator_only_selected) {
      // display interactive manipulator only for the focused object
      tobject->setDisplayInteractiveManipulator(object_name == focus_object_marker_name_);
    }
    if (config_.display_description_only_selected) {
      // display description only for the focused object
      tobject->setDisplayDescription(object_name == focus_object_marker_name_);
    }
    updateTransformableObject(tobject);
  }
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
  focus_name_text_pub_.publish(focus_text);
}

void TransformableInteractiveServer::focusPosePublish(){
  geometry_msgs::Pose target_pose;
  std::stringstream ss;
  if (transformable_objects_map_.find(focus_object_marker_name_) != transformable_objects_map_.end()) {
    target_pose = transformable_objects_map_[focus_object_marker_name_]->getPose();
    ss << "Pos x: " << target_pose.position.x  << " y: " << target_pose.position.y << " z: " << target_pose.position.z
       << std::endl
       << "Ori x: " << target_pose.orientation.x << " y: " << target_pose.orientation.y << " z: " << target_pose.orientation.z << " w: " << target_pose.orientation.w;
  }

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
  focus_pose_text_pub_.publish(focus_pose);
}

void TransformableInteractiveServer::focusObjectMarkerNamePublish(){
  std_msgs::String msg;
  msg.data = focus_object_marker_name_;
  focus_object_marker_name_pub_.publish(msg);
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

void TransformableInteractiveServer::insertNewMesh( std::string frame_id, std::string name, std::string description, std::string mesh_resource, bool mesh_use_embedded_materials)
{
  TransformableMesh* transformable_mesh = new TransformableMesh(frame_id, name, description, mesh_resource, mesh_use_embedded_materials);
  insertNewObject(transformable_mesh, name);
}

void TransformableInteractiveServer::insertNewObject( TransformableObject* tobject , std::string name )
{
  SetInitialInteractiveMarkerConfig(tobject);
  visualization_msgs::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  transformable_objects_map_[name] = tobject;
  server_->insert(int_marker, boost::bind( &TransformableInteractiveServer::processFeedback,this, _1));
  yaml_menu_handler_ptr_->applyMenu(server_, name);
  server_->applyChanges();

  focus_object_marker_name_ = name;
  focusTextPublish();
  focusPosePublish();
  focusObjectMarkerNamePublish();
}

void TransformableInteractiveServer::SetInitialInteractiveMarkerConfig( TransformableObject* tobject )
{
  InteractiveSettingConfig config(config_);
  if (config.display_interactive_manipulator && !config.display_interactive_manipulator_only_selected) {
    config.display_interactive_manipulator = true;
  } else {
    config.display_interactive_manipulator = false;
  }
  tobject->setInteractiveMarkerSetting(config);
}

void TransformableInteractiveServer::eraseObject( std::string name )
{
  server_->erase(name);
  server_->applyChanges();
  if (focus_object_marker_name_.compare(name) == 0) {
    focus_object_marker_name_ = "";
    focusTextPublish();
    focusPosePublish();
    focusObjectMarkerNamePublish();
  }
  delete transformable_objects_map_[name];
  transformable_objects_map_.erase(name);
}

void TransformableInteractiveServer::eraseAllObject()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    eraseObject(itpairstri->first);
  }
}

void TransformableInteractiveServer::eraseFocusObject()
{
  eraseObject(focus_object_marker_name_);
}

void TransformableInteractiveServer::tfTimerCallback(const ros::TimerEvent&)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->publishTF();
}

bool TransformableInteractiveServer::setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::PoseStamped pose_stamped, bool for_interactive_control)
{
  try {
    geometry_msgs::PoseStamped transformed_pose_stamped;
    jsk_interactive_marker::PoseStampedWithName transformed_pose_stamped_with_name;
    ros::Time stamp;
    if (strict_tf_) {
      stamp = pose_stamped.header.stamp;
    }
    else {
      stamp = ros::Time(0.0);
      pose_stamped.header.stamp = stamp;
    }
    if (!strict_tf_ || tf_listener_->waitForTransform(tobject->getFrameId(),
                                                      pose_stamped.header.frame_id, stamp, ros::Duration(1.0))) {
      tf_listener_->transformPose(tobject->getFrameId(), pose_stamped, transformed_pose_stamped);
      tobject->setPose(transformed_pose_stamped.pose, for_interactive_control);
      transformed_pose_stamped.pose=tobject->getPose(true);
      pose_pub_.publish(transformed_pose_stamped);
      transformed_pose_stamped_with_name.pose = transformed_pose_stamped;
      transformed_pose_stamped_with_name.name = tobject->name_;
      //transformed_pose_stamped_with_name.name = focus_object_marker_name_;
      pose_with_name_pub_.publish(transformed_pose_stamped_with_name);
    }
    else {
      ROS_ERROR("failed to lookup transform %s -> %s", tobject->getFrameId().c_str(), 
                pose_stamped.header.frame_id.c_str());
      return false;
    }
  }
  catch (tf2::ConnectivityException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
    return false;
  }
  catch (tf2::InvalidArgumentException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
    return false;
  }
  return true;
}

void TransformableInteractiveServer::run(){
  ros::spin();
}
