#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>
#include <Eigen/Geometry>

using namespace urdf;
using namespace std;

void UrdfModelMarker::addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, bool root){
  visualization_msgs::InteractiveMarkerControl control;
  if(root){
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    return;
  }else{
    boost::shared_ptr<Joint> parent_joint = link->parent_joint;
    Eigen::Vector3f origin_x(1,0,0);
    Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
    Eigen::Quaternionf qua;
    switch(parent_joint->type){
    case Joint::REVOLUTE:
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      qua.setFromTwoVectors(origin_x, dest_x);
      control.orientation.x = qua.x();
      control.orientation.y = qua.y();
      control.orientation.z = qua.z();
      control.orientation.w = qua.w();

      int_marker.controls.push_back(control);
      break;
    default:
      break;

    }
  }
}

void UrdfModelMarker::proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id){
  tf::Transform transform;
  geometry_msgs::Point point =  feedback->pose.position;
  transform.setOrigin( tf::Vector3(point.x, point.y, point.z));
  geometry_msgs::Quaternion qua =  feedback->pose.orientation;
  transform.setRotation( tf::Quaternion(qua.x, qua.y, qua.z, qua.w));

  dynamic_tf_publisher::SetDynamicTF SetTf;
  
  SetTf.request.freq = 10;
  SetTf.request.cur_tf.header.stamp = ros::Time::now();
  SetTf.request.cur_tf.header.frame_id = parent_frame_id;
  SetTf.request.cur_tf.child_frame_id = frame_id;
  SetTf.request.cur_tf.transform.translation.x = feedback->pose.position.x;
  SetTf.request.cur_tf.transform.translation.y = feedback->pose.position.y;
  SetTf.request.cur_tf.transform.translation.z = feedback->pose.position.z;
  SetTf.request.cur_tf.transform.rotation = feedback->pose.orientation;

  dynamic_tf_publisher_client.call(SetTf);
  cout << "parent:" << parent_frame_id << " frame:" << frame_id <<endl;
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


void UrdfModelMarker::addChildLinkNames(boost::shared_ptr<const Link> link, bool root)
{
  geometry_msgs::PoseStamped ps;

  double scale_factor = 1.02;
  string link_frame_name_ = link->name;
  string parent_link_frame_name_;

  if(root){
    //need slash ? 
    parent_link_frame_name_ = "root";
  }else{
    parent_link_frame_name_ = link->parent_joint->parent_link_name;
  }

  ps.header.frame_id = "/" + parent_link_frame_name_;

  if(!root){
    //double r, p, y;
    double x, y, z, w;
    //link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
    link->parent_joint->parent_to_joint_origin_transform.rotation.getQuaternion(x,y,z,w);
    //cout << r << p << y << endl;
    //TODO add rpy

    ps.pose.position.x = link->parent_joint->parent_to_joint_origin_transform.position.x;
    ps.pose.position.y = link->parent_joint->parent_to_joint_origin_transform.position.y;
    ps.pose.position.z = link->parent_joint->parent_to_joint_origin_transform.position.z;
    ps.pose.orientation.x = x;
    ps.pose.orientation.y = y;
    ps.pose.orientation.z = z;
    ps.pose.orientation.w = w;
    cout << "x" << ps.pose.position.x << endl;
    cout << "y" << ps.pose.position.y << endl;
    cout << "z" << ps.pose.position.z << endl;


    dynamic_tf_publisher::SetDynamicTF SetTf;
  
    SetTf.request.freq = 10;
    SetTf.request.cur_tf.header.stamp = ros::Time::now();
    SetTf.request.cur_tf.header.frame_id = parent_link_frame_name_;
    SetTf.request.cur_tf.child_frame_id = link_frame_name_;
    SetTf.request.cur_tf.transform.translation.x = link->parent_joint->parent_to_joint_origin_transform.position.x;
    SetTf.request.cur_tf.transform.translation.y = link->parent_joint->parent_to_joint_origin_transform.position.y;
    SetTf.request.cur_tf.transform.translation.z = link->parent_joint->parent_to_joint_origin_transform.position.z;
    //SetTf.request.cur_tf.transform.rotation = feedback->pose.orientation;
    SetTf.request.cur_tf.transform.rotation.w = 1;

    dynamic_tf_publisher_client.call(SetTf);
  }

  /*  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0));
  transform.setRotation( tf::Quaternion(0,0,0,1));
  cout << "tf" << "parent:" << parent_link_frame_name_  << "  link:" << link_frame_name_ << endl;
  tfb_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_link_frame_name_, link_frame_name_));
*/
  cout << "parent:" << parent_link_frame_name_  << "  link:" << link_frame_name_ << endl;

  visualization_msgs::InteractiveMarker int_marker;

  int_marker.header = ps.header;
  cout << int_marker.header << endl;

  int_marker.name = link_frame_name_;
  int_marker.scale = 1.0;
  int_marker.description = link_frame_name_;
  int_marker.pose = ps.pose;

  addMoveMarkerControl(int_marker, link, root);

  //model Mesh
  string model_mesh_ = "";
  if(link->visual->geometry.get() != NULL && link->visual->geometry->type == Geometry::MESH){
    boost::shared_ptr<const Mesh> mesh = boost::static_pointer_cast<const Mesh>(link->visual->geometry);
    model_mesh_ = mesh->filename;

    //TODO find model under GAZEBO_MODEL_PATH
    model_mesh_.erase(0,8);
    //model_mesh_ = "package://hrpsys_gazebo_tutorials/environment_models/" + model_mesh_;
    model_mesh_ = "package://hrpsys_gazebo_tutorials/robot_models/" + model_mesh_;

    cout << model_mesh_ << endl;

    visualization_msgs::InteractiveMarkerControl meshControl = makeMeshMarkerControl(model_mesh_, ps, scale_factor);
    int_marker.controls.push_back( meshControl );
  }

  server_->insert(int_marker);

  server_->setCallback( int_marker.name,
			boost::bind( &UrdfModelMarker::proc_feedback, this, _1, parent_link_frame_name_, link_frame_name_) );

  //menu_head_.apply(*server_, link_frame_name_);
  server_->applyChanges();

  cout << "Link" << link->name << endl;

  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    addChildLinkNames(*child, false);
}

void UrdfModelMarker::addChildJointNames(boost::shared_ptr<const Link> link, ofstream& os)
{
  double r, p, y;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
    /*
      os << "\"" << link->name << "\" -> \"" << (*child)->parent_joint->name 
      << "\" [label=\"xyz: "
      << (*child)->parent_joint->parent_to_joint_origin_transform.position.x << " " 
      << (*child)->parent_joint->parent_to_joint_origin_transform.position.y << " " 
      << (*child)->parent_joint->parent_to_joint_origin_transform.position.z << " " 
      << "\\nrpy: " << r << " " << p << " " << y << "\"]" << endl;
      os << "\"" << (*child)->parent_joint->name << "\" -> \"" << (*child)->name << "\"" << endl;*/
    addChildJointNames(*child, os);
  }
}



void UrdfModelMarker::printTree(boost::shared_ptr<const Link> link, string file)
{
  //std::ofstream os;
  //os.open(file.c_str());
  //os << "digraph G {" << endl;

  //os << "node [shape=box];" << endl;
  addChildLinkNames(link, true);

  //os << "node [shape=ellipse, color=blue, fontcolor=blue];" << endl;
  //addChildJointNames(link, os);

  //os << "}" << endl;
  //os.close();
}



int UrdfModelMarker::main(std::string file)
{

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(file.c_str(), std::fstream::in);
  while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
  xml_file.close();
  
  std::cout << "model_file:" << file;
  //std::cout << xml_string;
  boost::shared_ptr<ModelInterface> model = parseURDF(xml_string);
  if (!model){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  string output = model->getName();

  // print entire tree to file
  printTree(model->getRoot(), output+".gv");

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/ROOT_link";

  //int_marker.header.stamp = ros::Time::now();
  int_marker.name = "test";
  int_marker.scale = 1.0;
  int_marker.description = "test^^^^-------------------^^^^^test";
  //im_helpers::add6DofControl(int_marker,false);
  im_helpers::add6DofControl(int_marker,false);
  //visualization_msgs::InteractiveMarkerControl control;
  //int_marker.controls.push_back(control);
  //server_->insert(int_marker);
  server_->applyChanges();
  //ros::spin();
  /*
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    }*/
  //cout << "Created file " << output << ".gv" << endl;

  //string command = "dot -Tpdf "+output+".gv  -o "+output+".pdf";
  //system(command.c_str());
  //cout << "Created file " << output << ".pdf" << endl;
  return 0;
}


UrdfModelMarker::UrdfModelMarker (string file) : nh_(), pnh_("~"), tfl_(nh_) {
  pnh_.param("server_name", server_name, std::string ("") );
  

  if ( server_name == "" ) {
    server_name = ros::this_node::getName();
  }
  dynamic_tf_publisher_client = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf");

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
  server_.reset( new interactive_markers::InteractiveMarkerServer(server_name, "sid", false) );
  main(file);
  //ros::spin();
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_model_marker_interface");
  //UrdfModelMarker umm("/home/furuta/ros/fuerte/rtm-ros-robotics/rtmros_gazebo/hrpsys_gazebo_tutorials/environment_models/room73b2-cupboard-left-0/model.urdf");
  UrdfModelMarker umm("/home/furuta/ros/fuerte/rtm-ros-robotics/rtmros_gazebo/hrpsys_gazebo_tutorials/robot_models/SampleRobot/SampleRobot.urdf");
  //  UrdfModelMarker umm2("/home/furuta/ros/fuerte/rtm-ros-robotics/rtmros_gazebo/hrpsys_gazebo_tutorials/environment_models/simple-box/model.urdf");

  ros::spin();
  return 0;
}
