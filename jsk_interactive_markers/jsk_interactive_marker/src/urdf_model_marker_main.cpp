#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <memory>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <geometry_msgs/PoseArray.h>

using namespace urdf;
using namespace std;

class UrdfModelSettings {
private:
  ros::NodeHandle pnh_;
  ros::Subscriber display_marker_sub_;
  XmlRpc::XmlRpcValue model_config_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  string model_name_;
  double scale_factor_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Pose root_offset_;
  bool use_visible_color_;
  string mode_;
  string model_file_;
  bool registration_;
  string fixed_link_;
  string frame_id_;
  bool use_robot_description_;
  bool robot_mode_;
  bool display_;
  map<string, double> initial_pose_map_;

  typedef boost::shared_ptr<UrdfModelMarker> umm_ptr;
  typedef vector<umm_ptr> umm_vec;
  umm_vec umm_vec_;

public:
  void displayMarkerArrayCB( const geometry_msgs::PoseArrayConstPtr &msg){
    std_msgs::Header header = msg->header;
    geometry_msgs::PoseStamped ps;
    ps.header = header;

    int msg_size = msg->poses.size();
    int umm_vec_size = umm_vec_.size();

    if(msg_size <= umm_vec_size){
      for (int i = 0; i < msg_size; i++){
	//change pose
	ps.pose = msg->poses[i];
	umm_vec_[i]->setRootPose(ps);
      }
      for (int i = msg_size; i< umm_vec_size; i++){
	//move far away
	ps.pose.position.x = ps.pose.position.y = ps.pose.position.z = 1000000;
	ps.pose.orientation.w = 1;
	umm_vec_[i]->setRootPose(ps);
      }
    }else{
      for (int i = 0; i < umm_vec_size; i++){
	//change pose
	ps.pose = msg->poses[i];
	umm_vec_[i]->setRootPose(ps);
      }
      for (int i = umm_vec_size; i < msg_size; i++){
	geometry_msgs::Pose pose = msg->poses[i];
	umm_vec_.push_back(umm_ptr(new UrdfModelMarker(model_name_, model_file_, header.frame_id, pose, root_offset_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_, initial_pose_map_, i, server_)));
      }
    }
  }

  void init(){
    //name
    model_name_.assign(model_config_["name"]);
    //scale
    scale_factor_ = 1.02;
    if(model_config_.hasMember("scale")){
      scale_factor_ = getXmlValue(model_config_["scale"]);
    }
    //pose
    pose_ = getPose(model_config_["pose"]);
    root_offset_ = getPose(model_config_["offset"]);

    //color
    use_visible_color_ = false;
    if(model_config_.hasMember("use_visible_color")){
      use_visible_color_ = model_config_["use_visible_color"];
    }
    //frame id
    frame_id_.assign(model_config_["frame-id"]);

    //mode
    mode_ = "model";
    model_file_ = "";
    registration_ = false;
    fixed_link_ = "";
    if(model_config_.hasMember("registration")){
      registration_ = model_config_["registration"];
      mode_ = "registration";
      if(model_config_.hasMember("fixed-link")){
	fixed_link_.assign( model_config_["fixed-link"]);
      }
    }
    if(model_config_.hasMember("model")){
      model_file_.assign(model_config_["model"]);
    }
    use_robot_description_ = false;
    if(model_config_.hasMember("use_robot_description")){
      model_file_ = "/robot_description";
      use_robot_description_ = model_config_["use_robot_description"];
    }
    if(model_config_.hasMember("model_param")){
      use_robot_description_ = true;
      model_file_.assign(model_config_["model_param"]);
    }
    if(model_config_["robot"]){
      mode_ = "robot";
    }
    if(model_config_.hasMember("mode")){
      mode_.assign(model_config_["mode"]);
    }

    robot_mode_ = model_config_["robot"];

    //initial pose
    if(model_config_.hasMember("initial_joint_state")){
      XmlRpc::XmlRpcValue initial_pose = model_config_["initial_joint_state"];
      for(int i=0; i< initial_pose.size(); i++){
	XmlRpc::XmlRpcValue v = initial_pose[i];
	string name;
	double position;
	if(v.hasMember("name") && v.hasMember("position")){
	  name.assign(v["name"]);
	  position = getXmlValue(v["position"]);
	  initial_pose_map_[name] = position;
	}
      }
    }

    //default display
    if(model_config_.hasMember("display")){
      display_ = model_config_["display"];
    }else{
      display_ = true;
    }
  }

  void addUrdfMarker(){
    new UrdfModelMarker(model_name_, model_file_, frame_id_, pose_,root_offset_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_,initial_pose_map_, -1, server_);
  }

  UrdfModelSettings(XmlRpc::XmlRpcValue model,   boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : pnh_("~"){
    model_config_ = model;
    server_ = server;
    init();

    if(display_){
      addUrdfMarker();
    }
    display_marker_sub_ = pnh_.subscribe<geometry_msgs::PoseArray> (model_name_ + "/pose_array", 1, boost::bind( &UrdfModelSettings::displayMarkerArrayCB, this, _1));
  }
};



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
    new UrdfModelSettings(v[i], server);
  }
  ros::spin();
  return 0;
}


