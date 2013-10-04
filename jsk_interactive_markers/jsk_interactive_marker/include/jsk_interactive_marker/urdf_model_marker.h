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

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
using namespace urdf;
using namespace std;


class UrdfModelMarker {
 public:
  UrdfModelMarker(string file);

  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color);


  void addChildLinkNames(boost::shared_ptr<const Link> link, bool root);
  void addChildJointNames(boost::shared_ptr<const Link> link, ofstream& os);
  void printTree(boost::shared_ptr<const Link> link, string file);
  int main(string file);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pub_;
  ros::Publisher pub_move_;
  ros::ServiceServer serv_reset_;
  ros::ServiceServer serv_set_;  
  ros::ServiceServer serv_markers_set_;
  ros::ServiceServer serv_markers_del_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  ros::ServiceClient dynamic_tf_publisher_client;

  std::string marker_name;
  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;


};

