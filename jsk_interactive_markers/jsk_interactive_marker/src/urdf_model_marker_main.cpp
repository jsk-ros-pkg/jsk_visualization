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
public:
  void displayMarkerCB(std_msgs::Empty e){

  }
  void addUrdfMarker(){
    XmlRpc::XmlRpcValue model = model_config_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server = server_;
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
  }
  
  UrdfModelSettings(XmlRpc::XmlRpcValue model,   boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : pnh_("~"){
    model_config_ = model;
    server_ = server;
    addUrdfMarker();
    //display_marker_sub_ = pnh_.subscribe<std_msgs::Empty> ("hoge", 1, boost::bind( &UrdfModelSettings::displayMarkerCB, this, _1));


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


