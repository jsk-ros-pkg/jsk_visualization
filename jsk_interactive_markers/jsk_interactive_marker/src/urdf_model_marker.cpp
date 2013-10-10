#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>
#include <Eigen/Geometry>

using namespace urdf;
using namespace std;

void UrdfModelMarker::addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, bool root){
  visualization_msgs::InteractiveMarkerControl control;
  if(root){
    im_helpers::add6DofControl(int_marker,false);
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

    switch(parent_joint->type){
    case Joint::REVOLUTE:
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
  dynamic_tf_publisher_client.call(SetTf);
}

void UrdfModelMarker::publishMarkerPose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
    jsk_interactive_marker::MarkerPose mp;
    mp.pose.header = feedback->header;
    mp.pose.pose = feedback->pose;
    mp.marker_name = feedback->marker_name;

    pub_.publish( mp );
}

void UrdfModelMarker::proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id){
  switch ( feedback->event_type ){
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    CallSetDynamicTf(parent_frame_id, frame_id, Pose2Transform(feedback->pose));
    cout << "parent:" << parent_frame_id << " frame:" << frame_id <<endl;
    publishMarkerPose(feedback);
    
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << frame_id <<endl;
    linkMarkerMap[frame_id].displayMoveMarker ^= true;
    addChildLinkNames(model->getRoot(), true, false);
    break;
  }
}

void UrdfModelMarker::graspPointCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  
  addChildLinkNames(model->getRoot(), true, false); 
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


void UrdfModelMarker::addChildLinkNames(boost::shared_ptr<const Link> link, bool root, bool init)
{
  geometry_msgs::PoseStamped ps;

  double scale_factor = 1.02;
  string link_frame_name_ = link->name;
  string parent_link_frame_name_;

  link_frame_name_ = model_name_ + "/" + link_frame_name_;

  if(root){
    parent_link_frame_name_ = frame_id_;
    ps.pose = root_pose_;
  }else{
    parent_link_frame_name_ = link->parent_joint->parent_link_name;
    parent_link_frame_name_ = model_name_ + "/" + parent_link_frame_name_;
    ps.pose = UrdfPose2Pose(link->parent_joint->parent_to_joint_origin_transform);
  }
  ps.header.frame_id =  parent_link_frame_name_;

  if(init){
    CallSetDynamicTf(parent_link_frame_name_, link_frame_name_, Pose2Transform(ps.pose));
    //linkMarkerMap.insert( map<string, struct>::value_type( link_frame_name_, false ) );
    linkProperty lp;
    linkMarkerMap.insert( map<string, linkProperty>::value_type( link_frame_name_, lp ) );
  }

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;

  int_marker.name = link_frame_name_;
  int_marker.scale = 1.0;
  //int_marker.description = link_frame_name_;
  int_marker.pose = ps.pose;

  if(!init){
    visualization_msgs::InteractiveMarker old_marker;
    if(server_->get(link_frame_name_, old_marker)){
      int_marker.pose = old_marker.pose;
    }
  }

  //move Marker
  //if(linkMarkerMap[link_frame_name_]){
  if(linkMarkerMap[link_frame_name_].displayMoveMarker){
    addMoveMarkerControl(int_marker, link, root);
  }

  //model Mesh Marker
  string model_mesh_ = "";
  if(link->visual->geometry.get() != NULL && link->visual->geometry->type == Geometry::MESH){
    boost::shared_ptr<const Mesh> mesh = boost::static_pointer_cast<const Mesh>(link->visual->geometry);
    model_mesh_ = mesh->filename;
    model_mesh_ = getModelFilePath(model_mesh_);
    ps.pose = UrdfPose2Pose(link->visual->origin);

    visualization_msgs::InteractiveMarkerControl meshControl = makeMeshMarkerControl(model_mesh_, ps, scale_factor);
    int_marker.controls.push_back( meshControl );
  }

  server_->insert(int_marker);
  server_->setCallback( int_marker.name,
			boost::bind( &UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_) );



  model_menu_.apply(*server_, link_frame_name_);
  server_->applyChanges();

  cout << "Link:" << link->name << endl;

  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    addChildLinkNames(*child, false, init);
  }
}



UrdfModelMarker::UrdfModelMarker ()
{}

UrdfModelMarker::UrdfModelMarker (string model_name, string model_file, string frame_id, geometry_msgs::Pose root_pose, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : nh_(), pnh_("~"), tfl_(nh_) {
  pnh_.param("server_name", server_name, std::string ("") );

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  dynamic_tf_publisher_client = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf");

  server_ = server;
  model_name_ = model_name;
  model_file_ = model_file;
  frame_id_ = frame_id;
  root_pose_ = root_pose;

  pub_ =  pnh_.advertise<jsk_interactive_marker::MarkerPose> ("pose", 1);

  /*
    pub_ =  pnh_.advertise<jsk_interactive_marker::MarkerPose> ("pose", 1);
    pub_move_ =  pnh_.advertise<jsk_interactive_marker::MarkerMenu> ("move_flag", 1);

    serv_set_ = pnh_.advertiseService("set_pose",
    &InteractiveMarkerInterface::set_cb, this);
    serv_markers_set_ = pnh_.advertiseService("set_markers",
    &InteractiveMarkerInterface::markers_set_cb, this);
    serv_markers_del_ = pnh_.advertiseService("del_markers",
    &InteractiveMarkerInterface::markers_del_cb, this);
    serv_reset_ = pnh_.advertiseService("reset_pose",
    &InteractiveMarkerInterface::reset_cb, this);
  */
  model_menu_.insert( "Grasp Point",
		      boost::bind( &UrdfModelMarker::graspPointCB, this, _1 ) );


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
  return;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_model_marker_interface");
  ros::NodeHandle n;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset( new interactive_markers::InteractiveMarkerServer("jsk_model_marker_interface", "", false) );

  ros::NodeHandle pnh_("~");

  XmlRpc::XmlRpcValue v;
  pnh_.param("model_config", v, v);
  for(int i=0; i< v.size(); i++){
    XmlRpc::XmlRpcValue model = v[i];
    std::cout << "name:" << model["name"] <<endl;
    geometry_msgs::Pose p = getPose(model["pose"]);
    new UrdfModelMarker(model["name"], model["model"], model["frame-id"], p, server);
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
