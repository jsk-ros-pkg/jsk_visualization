#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <stdlib.h>
#include <ros/package.h>
#include "urdf_parser/urdf_parser.h"

#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace urdf;
using namespace std;
using namespace boost;
using namespace boost::filesystem;

namespace im_utils {

  //transform msgs
  geometry_msgs::Transform Pose2Transform( const geometry_msgs::Pose pose_msg){
    geometry_msgs::Transform tf_msg;
    tf_msg.translation.x = pose_msg.position.x;
    tf_msg.translation.y = pose_msg.position.y;
    tf_msg.translation.z = pose_msg.position.z;
    tf_msg.rotation = pose_msg.orientation;
    return tf_msg;
  }

  geometry_msgs::Pose Transform2Pose( const geometry_msgs::Transform tf_msg){
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x =  tf_msg.translation.x;
    pose_msg.position.y = tf_msg.translation.y;
    pose_msg.position.z = tf_msg.translation.z;
    pose_msg.orientation = tf_msg.rotation;
    return pose_msg;
  }

  geometry_msgs::Pose UrdfPose2Pose( const urdf::Pose pose){
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


  visualization_msgs::InteractiveMarkerControl makeCylinderMarkerControl(const geometry_msgs::PoseStamped &stamped, double length,  double radius, const std_msgs::ColorRGBA &color, bool use_color){
    visualization_msgs::Marker cylinderMarker;

    if (use_color) cylinderMarker.color = color;
    cylinderMarker.type = visualization_msgs::Marker::CYLINDER;
    cylinderMarker.scale.x = radius * 2;
    cylinderMarker.scale.y = radius * 2;
    cylinderMarker.scale.z = length;
    cylinderMarker.pose = stamped.pose;

    visualization_msgs::InteractiveMarkerControl control;
    control.markers.push_back( cylinderMarker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    return control;
  }

  visualization_msgs::InteractiveMarkerControl makeBoxMarkerControl(const geometry_msgs::PoseStamped &stamped, Vector3 dim, const std_msgs::ColorRGBA &color, bool use_color){
    visualization_msgs::Marker boxMarker;

    fprintf(stderr, "urdfModelMarker = %f %f %f\n", dim.x, dim.y, dim.z);
    if (use_color) boxMarker.color = color;
    boxMarker.type = visualization_msgs::Marker::CUBE;
    boxMarker.scale.x = dim.x;
    boxMarker.scale.y = dim.y;
    boxMarker.scale.z = dim.z;
    boxMarker.pose = stamped.pose;

    visualization_msgs::InteractiveMarkerControl control;
    control.markers.push_back( boxMarker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    return control;
  }

  visualization_msgs::InteractiveMarkerControl makeSphereMarkerControl(const geometry_msgs::PoseStamped &stamped, double rad, const std_msgs::ColorRGBA &color, bool use_color){
    visualization_msgs::Marker sphereMarker;

    if (use_color) sphereMarker.color = color;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.scale.x = rad * 2;
    sphereMarker.scale.y = rad * 2;
    sphereMarker.scale.z = rad * 2;
    sphereMarker.pose = stamped.pose;

    visualization_msgs::InteractiveMarkerControl control;
    control.markers.push_back( sphereMarker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    return control;
  }



  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color, bool use_color){
    visualization_msgs::Marker meshMarker;

    if (use_color) meshMarker.color = color;
    meshMarker.mesh_resource = mesh_resource;
    meshMarker.mesh_use_embedded_materials = !use_color;
    meshMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  
    meshMarker.scale = scale;
    meshMarker.pose = stamped.pose;
    visualization_msgs::InteractiveMarkerControl control;
    control.markers.push_back( meshMarker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;
  
    return control;
  }

  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale)
  {
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 0;
    return makeMeshMarkerControl(mesh_resource, stamped, scale, color, false);
  }

  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource,
                                                                     const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color)
  {
    return makeMeshMarkerControl(mesh_resource, stamped, scale, color, true);
  }

  void addMeshLinksControl(visualization_msgs::InteractiveMarker &im, boost::shared_ptr<const Link> link, KDL::Frame previous_frame, bool use_color, std_msgs::ColorRGBA color){
    if(link->parent_joint){
      KDL::Frame parent_to_joint_frame;
      geometry_msgs::Pose parent_to_joint_pose = UrdfPose2Pose(link->parent_joint->parent_to_joint_origin_transform);
      tf::poseMsgToKDL(parent_to_joint_pose, parent_to_joint_frame);
      previous_frame =  previous_frame * parent_to_joint_frame;
    }

    //    KDL::Frame pose_frame, offset_frame;
    //    tf::poseMsgToKDL(pose, pose_frame);
    //    tf::poseMsgToKDL(root_offset_, offset_frame);
    //    pose_frame = pose_frame * offset_frame;

    geometry_msgs::PoseStamped ps;
    //link_array
    std::vector<boost ::shared_ptr<Visual> > visual_array;
    if(link->visual_array.size() != 0){
      visual_array = link->visual_array;
    }else if(link->visual.get() != NULL){
      visual_array.push_back(link->visual);
    }
    for(int i=0; i<visual_array.size(); i++){
      boost::shared_ptr<Visual> link_visual = visual_array[i];
      if(link_visual.get() != NULL && link_visual->geometry.get() != NULL){
	visualization_msgs::InteractiveMarkerControl meshControl;
	if(link_visual->geometry->type == Geometry::MESH){
	  boost::shared_ptr<const Mesh> mesh = boost::static_pointer_cast<const Mesh>(link_visual->geometry);
	  string model_mesh_ = mesh->filename;
          model_mesh_ = getRosPathFromModelPath(model_mesh_);
          
	  //ps.pose = UrdfPose2Pose(link_visual->origin);
          KDL::Frame pose_frame, origin_frame;

          tf::poseMsgToKDL(UrdfPose2Pose(link_visual->origin), origin_frame);
          pose_frame =  previous_frame * origin_frame;
          geometry_msgs::Pose pose;
          tf::poseKDLToMsg(pose_frame, pose);
          ps.pose = pose;

	  cout << "mesh_file:" << model_mesh_ << endl;

	  geometry_msgs::Vector3 mesh_scale;
	  mesh_scale.x = mesh->scale.x;
	  mesh_scale.y = mesh->scale.y;
	  mesh_scale.z = mesh->scale.z;
	  if(use_color){
	    meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale, color);
	  }else{
	    meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale);
	  }
	}else if(link_visual->geometry->type == Geometry::CYLINDER){
	  boost::shared_ptr<const Cylinder> cylinder = boost::static_pointer_cast<const Cylinder>(link_visual->geometry);
	  std::cout << "cylinder " << link->name;
	  ps.pose = UrdfPose2Pose(link_visual->origin);
	  double length = cylinder->length;
	  double radius = cylinder->radius;
	  std::cout << ", length =  " << length << ", radius " << radius << std::endl;
	  if(use_color){
	    meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
	  }else{
	    meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
	  }
	}else if(link_visual->geometry->type == Geometry::BOX){
	  boost::shared_ptr<const Box> box = boost::static_pointer_cast<const Box>(link_visual->geometry);
	  std::cout << "box " << link->name;
	  ps.pose = UrdfPose2Pose(link_visual->origin);
	  Vector3 dim = box->dim;
	  std::cout << ", dim =  " << dim.x << ", " << dim.y << ", " << dim.z << std::endl;
	  if(use_color){
	    meshControl = makeBoxMarkerControl(ps, dim, color, true);
	  }else{
	    meshControl = makeBoxMarkerControl(ps, dim, color, true);
	  }
	}else if(link_visual->geometry->type == Geometry::SPHERE){
	  boost::shared_ptr<const Sphere> sphere = boost::static_pointer_cast<const Sphere>(link_visual->geometry);
	  ps.pose = UrdfPose2Pose(link_visual->origin);
	  double rad = sphere->radius;
	  if(use_color){
	    meshControl = makeSphereMarkerControl(ps, rad, color, true);
	  }else{
	    meshControl = makeSphereMarkerControl(ps, rad, color, true);
	  }
	}
	im.controls.push_back( meshControl );

      }
    }
    for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
      addMeshLinksControl(im, *child, previous_frame, use_color, color);
    }
  }

  boost::shared_ptr<ModelInterface> getModelInterface(std::string model_file){
    boost::shared_ptr<ModelInterface> model;
    model_file = getFilePathFromRosPath(model_file);
    model_file = getFullPathFromModelPath(model_file);
    std::string xml_string;
    std::fstream xml_file(model_file.c_str(), std::fstream::in);
    while ( xml_file.good() )
      {
	std::string line;
	std::getline( xml_file, line);
	xml_string += (line + "\n");
      }
    xml_file.close();

    std::cout << "model_file:" << model_file << std::endl;
  
    model = parseURDF(xml_string);
    if (!model){
      std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
      return model;
    }
    return model;
  }

  //sample program
  visualization_msgs::InteractiveMarker makeLinksMarker(boost::shared_ptr<const Link> link, bool use_color, std_msgs::ColorRGBA color, geometry_msgs::PoseStamped marker_ps, geometry_msgs::Pose origin_pose)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header = marker_ps.header;
    int_marker.pose = marker_ps.pose;

    int_marker.name = "sample";
    int_marker.scale = 1.0;

    KDL::Frame origin_frame;
    tf::poseMsgToKDL(origin_pose, origin_frame);
    addMeshLinksControl(int_marker, link, origin_frame, use_color, color);
    return int_marker;

  }
}


visualization_msgs::InteractiveMarker makeFingerControlMarker(const char *name, geometry_msgs::PoseStamped ps){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;
  int_marker.scale = 0.5;

  visualization_msgs::InteractiveMarkerControl control;

  //control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;

  int_marker.controls.push_back(control);

  return int_marker;

}

/*

  visualization_msgs::InteractiveMarker makeSandiaHandMarker(geometry_msgs::PoseStamped ps){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;

  visualization_msgs::InteractiveMarkerControl control;

  control.markers.push_back(makeSandiaFinger0Marker("/right_f0_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f0_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f0_2"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger0Marker("/right_f1_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f1_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f1_2"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger0Marker("/right_f2_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f2_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f2_2"));
  int_marker.controls.push_back(control);

  control.markers.push_back(makeSandiaFinger0Marker("/right_f3_0"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger1Marker("/right_f3_1"));
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.markers.push_back(makeSandiaFinger2Marker("/right_f3_2"));
  int_marker.controls.push_back(control);

  return int_marker;

  }

*/


visualization_msgs::InteractiveMarker makeSandiaHandInteractiveMarker(geometry_msgs::PoseStamped ps, std::string hand, int finger, int link){
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;
  int_marker.pose = ps.pose;
  

  visualization_msgs::InteractiveMarkerControl control;
  std::stringstream ss;
  //std::string frame = "/" + hand + "_" + finger + "_" + link;
  ss << hand << "_f" << finger << "_" << link;
  
  int_marker.name = ss.str() + "Marker";
  int_marker.header.frame_id = ss.str();
  //  std::string frame_id = "odom";
  std::string frame_id = "utorso";
  int_marker.header.frame_id = frame_id;
  std::cout << ss.str() << std::endl;

  //frame_id = ss.str();
  switch(link){
  case 0:
    control.markers.push_back(makeSandiaFinger0Marker(frame_id));
    break;
  case 1:
    control.markers.push_back(makeSandiaFinger1Marker(frame_id));
    break;
  case 2:
    control.markers.push_back(makeSandiaFinger2Marker(frame_id));
    break;
  default:
    break;
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back(control);

  return int_marker;
}



visualization_msgs::Marker makeSandiaFinger0Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  //marker.header.frame_id = "odom";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.003;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.023;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

visualization_msgs::Marker makeSandiaFinger1Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.024;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.067;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

visualization_msgs::Marker makeSandiaFinger2Marker(std::string frame_id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.024;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.018;
  marker.scale.y = 0.018;
  marker.scale.z = 0.057;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

std::string getRosPathFromModelPath(std::string path){
  return getRosPathFromFullPath(getFullPathFromModelPath(path));
}

std::string getRosPathFromFullPath(std::string path){
  std::string ros_package_path = "";
  FILE* fp;
  char buf[1000000];

  //set $ROS_PACKAGE_PATH
  if ((fp = popen("echo $ROS_PACKAGE_PATH", "r")) == NULL) {
    std::cout << "popen error" << std::endl;
  }
  while (fgets(buf, sizeof(buf), fp) != NULL) {
    ros_package_path += buf;
  }
  pclose(fp);
  
  ros::package::V_string all_package;
  ros::package::getAll(all_package);

  if( path.find("file://", 0) == 0 ){
    path.erase(0,7);
    size_t current = 0, found;
    while((found = path.find_first_of("/", current)) != std::string::npos){
      std::string search_path = std::string(path, current, found - current);
      current = found + 1;
      std::string package_path;
      if( ros::package::getPath(search_path) != ""){
	return "package://" + search_path + path.erase(0, current-1);
      }
    }
  }

  return path;
}

std::string getFullPathFromModelPath(std::string path){
  std::string gazebo_model_path="";
  
  FILE* fp;
  char buf[1000000];

  //set $GAZEBO_MODEL_PATH
  if ((fp = popen("echo $GAZEBO_MODEL_PATH", "r")) == NULL) {
    std::cout << "popen error" << std::endl;
  }
  while (fgets(buf, sizeof(buf), fp) != NULL) {
    gazebo_model_path += buf;
  }
  pclose(fp);
  if( path.find("model://", 0) == 0 ){
    path.erase(0,8);
    //    ??
    //path.erase(0,9);
    size_t current = 0, found;
    while((found = gazebo_model_path.find_first_of(":", current)) != std::string::npos){
      std::string search_path = std::string(gazebo_model_path, current, found - current);
      current = found + 1;
      recursive_directory_iterator iter = recursive_directory_iterator(search_path);
      recursive_directory_iterator end = recursive_directory_iterator();
      for (; iter != end; ++iter) {
	if (is_regular_file(*iter)) {
	  int locate = iter->path().string().find( path, 0 );
	  if( locate != std::string::npos){
	    //for example file:///hoge/fuga.dae
	    return "file://" + iter->path().string();
	  }
	}
      }
    }
  }
  return path;
}

//convert package:// path to full path
std::string getFilePathFromRosPath( std::string rospath){
  std::string path = rospath;
  if (path.find("package://") == 0){
    path.erase(0, strlen("package://"));
    size_t pos = path.find("/");
    if (pos == std::string::npos){
      std::cout << "Could not parse package:// format" <<std::endl;
      return "";
    }
    std::string package = path.substr(0, pos);
    path.erase(0, pos);
    std::string package_path = ros::package::getPath(package);
    if (package_path.empty())
      {
        std::cout <<  "Package [" + package + "] does not exist" << std::endl;
      }
 
    path = package_path + path;
  }
  return path;
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


