#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <math.h>
#include <jsk_interactive_marker/MarkerMenu.h>
#include <jsk_interactive_marker/MarkerPose.h>

#include <std_msgs/Int8.h>

visualization_msgs::InteractiveMarker makeFingerControlMarker(const char *name, geometry_msgs::PoseStamped ps);
visualization_msgs::InteractiveMarker makeSandiaHandMarker(geometry_msgs::PoseStamped ps);

visualization_msgs::Marker makeSandiaFinger0Marker(std::string frame_id);
visualization_msgs::Marker makeSandiaFinger1Marker(std::string frame_id);
visualization_msgs::Marker makeSandiaFinger2Marker(std::string frame_id);

visualization_msgs::InteractiveMarker makeSandiaHandInteractiveMarker(geometry_msgs::PoseStamped ps, std::string hand, int finger, int link);

std::string getModelFilePath(std::string path);
std::string getFilePathFromRosPath( std::string rospath);
