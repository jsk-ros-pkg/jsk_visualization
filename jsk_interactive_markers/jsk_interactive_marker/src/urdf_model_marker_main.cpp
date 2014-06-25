#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

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
  bool use_visible_color_;
  string mode_;
  string model_file_;
  bool registration_;
  string fixed_link_;
  string frame_id_;
  bool use_robot_description_;
  bool robot_mode_;
  map<string, double> initial_pose_map_;

public:
  void displayMarkerCB( const std_msgs::EmptyConstPtr &e){
    server_->clear();
    //addUrdfMarker();
    new UrdfModelMarker(model_name_, model_file_, frame_id_, pose_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_,initial_pose_map_, server_);
  }
  /*
  void displayMarkerArrayCB( const geometry_msgs::PoseArrayConstPtr &msg){
    //addUrdfMarker();
    //geometry_msgs::Pose pose[] = msg->pose;
    //std_msgs::header header = msg->header;
    /*
    for (int i=0; i<pose.size(); i++){
      new UrdfModelMarker(model_name_ + i, model_file_, header.frame_id, pose[i], scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_, server_);
      }
  }*/

  void init(){
    model_name_.assign(model_config_["name"]);
    scale_factor_ = 1.02;
    if(model_config_.hasMember("scale")){
      scale_factor_ = getXmlValue(model_config_["scale"]);
    }
    pose_ = getPose(model_config_["pose"]);

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
  }

  void addUrdfMarker(){
    new UrdfModelMarker(model_name_, model_file_, frame_id_, pose_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_,initial_pose_map_, server_);
  }

  UrdfModelSettings(XmlRpc::XmlRpcValue model,   boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : pnh_("~"){
    model_config_ = model;
    server_ = server;
    init();
    addUrdfMarker();
    display_marker_sub_ = pnh_.subscribe<std_msgs::Empty> (model_name_ + "/hoge", 1, boost::bind( &UrdfModelSettings::displayMarkerCB, this, _1));
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
    /*
    XmlRpc::XmlRpcValue model = v[i];
    std::cout << "name:" << model["name"] <<endl;
    geometry_msgs::Pose p = getPose(model["pose"]);
    double scale_factor = 1.02;
    if(model.hasMember("scale")){
      scale_factor = getXmlValue(model["scale"]);
    }
    string mode = "model";
    string model_name;
    model_name.assign (model["name"]);
    string model_file = "";
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
    if(model.hasMember("model")){
      model_file.assign(model["model"]);
    }
    bool use_robot_description = false;
    if(model.hasMember("use_robot_description")){
      model_file = "/robot_description";
      use_robot_description = model["use_robot_description"];
    }
    if(model.hasMember("model_param")){
      use_robot_description = true;
      model_file.assign(model["model_param"]);
    }

    bool use_visible_color = false;
    if(model.hasMember("use_visible_color")){
      use_visible_color = model["use_visible_color"];
    }
    std::cout << use_visible_color <<std::endl;

    if(model["robot"]){
      mode = "robot";
    }
    if(model.hasMember("mode")){
      mode.assign(model["mode"]);
    }

    new UrdfModelMarker(model_name, model_file, model["frame-id"], p, scale_factor, mode , model["robot"], registration,fixed_link, use_robot_description, use_visible_color, server);
    ros::Subscriber display_marker_ = pnh_.subscribe<std_msgs::Empty> (model_name + "/display_marker", 1, viewMarkerCB);
    */
  }
  ros::spin();
  return 0;
}


