#ifndef _URDF_MODEL_MARKER_H_
#define _URDF_MODEL_MARKER_H_


#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <math.h>
#include <jsk_interactive_marker/MarkerMenu.h>
#include <jsk_interactive_marker/MarkerPose.h>
#include <jsk_interactive_marker/MoveObject.h>

#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
using namespace urdf;
using namespace std;


class UrdfModelMarker {
 public:
  //  UrdfModelMarker(string file);
  UrdfModelMarker(string file, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker(string model_name, string model_file, string frame_id, geometry_msgs::Pose root_pose, double scale_factor, string mode, bool robot_mode, bool registration, string fixed_link, bool use_robot_description, bool use_visible_color, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker();

  void addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, bool root);
  void addInvisibleMeshMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, const std_msgs::ColorRGBA &color);
  void addGraspPointControl(visualization_msgs::InteractiveMarker &int_marker, std::string link_frame_name_);

  void publishMarkerPose ( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishMarkerPose ( geometry_msgs::Pose pose, std_msgs::Header header, std::string marker_name);
  void publishMarkerMenu( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu );
  void publishMoveObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void publishJointState( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void republishJointState( sensor_msgs::JointState js);

  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id);

  void graspPoint_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string link_name);
  void moveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void setPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideAllMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void hideModelMarkerCB( const std_msgs::EmptyConstPtr &msg);
  void showModelMarkerCB( const std_msgs::EmptyConstPtr &msg);

  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color);



  visualization_msgs::InteractiveMarkerControl makeCylinderMarkerControl(const geometry_msgs::PoseStamped &stamped, double length, double radius, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeBoxMarkerControl(const geometry_msgs::PoseStamped &stamped, Vector3 dim, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeSphereMarkerControl(const geometry_msgs::PoseStamped &stamped, double rad, const std_msgs::ColorRGBA &color, bool use_color);

  void getJointState(boost::shared_ptr<const Link> link, sensor_msgs::JointState &js);
  void setJointState(boost::shared_ptr<const Link> link, const sensor_msgs::JointStateConstPtr &js);
  void setJointAngle(boost::shared_ptr<const Link> link, double joint_angle);
  void setOriginalPose(boost::shared_ptr<const Link> link);
  void addChildLinkNames(boost::shared_ptr<const Link> link, bool root, bool init);
  void addChildLinkNames(boost::shared_ptr<const Link> link, bool root, bool init, bool use_color, int color_index);

  geometry_msgs::Transform Pose2Transform(geometry_msgs::Pose pose_msg);
  geometry_msgs::Pose UrdfPose2Pose( const urdf::Pose pose);
  void CallSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::Transform transform);

  int main(string file);

  void graspPointCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void jointMoveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void resetMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void registrationCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  
  void setRootPoseCB( const geometry_msgs::PoseStampedConstPtr &msg );
  void resetJointStatesCB( const sensor_msgs::JointStateConstPtr &msg);
  
 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pub_;
  ros::Publisher pub_move_;
  ros::Publisher pub_move_object_;
  ros::Publisher pub_joint_state_;
  
  ros::Subscriber sub_reset_joints_;
  ros::Subscriber sub_set_root_pose_;
  ros::Subscriber hide_marker_;
  ros::Subscriber show_marker_;

  ros::ServiceServer serv_reset_;
  ros::ServiceServer serv_set_;
  ros::ServiceServer serv_markers_set_;
  ros::ServiceServer serv_markers_del_;

  interactive_markers::MenuHandler model_menu_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  ros::ServiceClient dynamic_tf_publisher_client;

  std::string marker_name;
  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;


  boost::shared_ptr<ModelInterface> model;
  std::string model_name_;
  std::string frame_id_;
  std::string model_file_;
  geometry_msgs::Pose root_pose_;
  double scale_factor_;
  bool robot_mode_;
  bool registration_;
  bool use_dynamic_tf_;
  string mode_;
  string fixed_link_;
  bool use_robot_description_;
  bool use_visible_color_;
  std::string tf_prefix_;

  struct graspPoint{
    graspPoint(){
      displayMoveMarker = false;
      displayGraspPoint = false;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
    }
    bool displayGraspPoint;
    bool displayMoveMarker;
    geometry_msgs::Pose pose;
  };


  struct linkProperty{
    linkProperty(){
      displayMoveMarker = false;
      displayModelMarker = true;
      mesh_file = "";
    }
    bool displayMoveMarker;
    bool displayModelMarker;
    graspPoint gp;
    string frame_id;
    string movable_link;
    //pose from frame_id
    geometry_msgs::Pose pose;
    geometry_msgs::Pose origin;
    geometry_msgs::Pose initial_pose;
    urdf::Vector3 joint_axis;
    double joint_angle;
    int rotation_count;
    string mesh_file;
  };

  map<string, linkProperty> linkMarkerMap;
};

geometry_msgs::Pose getPose( XmlRpc::XmlRpcValue val);
double getXmlValue( XmlRpc::XmlRpcValue val );

#endif
