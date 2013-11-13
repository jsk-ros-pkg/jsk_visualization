#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <stdlib.h>
#include <ros/package.h>

using namespace boost;
using namespace boost::filesystem;


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
  int_marker.header.frame_id = "odom";
  std::cout << ss.str() << std::endl;
  std::string frame_id = "odom";
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

  if( path.find("file://", 0) == 0 ){
    path.erase(0,7);
    size_t current = 0, found;
    while((found = ros_package_path.find_first_of(":", current)) != std::string::npos){
      std::string search_path = std::string(ros_package_path, current, found - current);
      current = found + 1;
      
      if( path.find(search_path, 0) == 0){
	path.erase(0, search_path.length() + 1);
	return "package://" + path;
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
    path.erase(0,9);
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

