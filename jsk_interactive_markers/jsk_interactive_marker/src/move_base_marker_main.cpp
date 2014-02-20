#include <iostream>
#include <jsk_interactive_marker/move_base_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_marker");
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
      }
    }
    bool use_robot_description = false;
    if(model.hasMember("use_robot_description")){
      use_robot_description = model["use_robot_description"];
    }

    bool use_visible_color = false;
    if(model.hasMember("use_visible_color")){
      use_visible_color = model["use_visible_color"];
    }
    std::cout << "use_visible_color: " << use_visible_color <<std::endl;


    if(model["robot"]){
      mode = "robot";
    }
    if(model.hasMember("mode")){
      mode.assign(model["mode"]);
    }


    new MoveBaseMarker(model["name"], model["model"], model["frame-id"], p, scale_factor, mode , model["robot"], registration,fixed_link, use_robot_description, use_visible_color, server);

  }

  ros::spin();
  return 0;
}
