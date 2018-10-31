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
#include <jsk_interactive_marker/MoveModel.h>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <jsk_recognition_msgs/Int32Stamped.h>

#include <jsk_topic_tools/time_accumulator.h>

#if ROS_VERSION_MINIMUM(1,12,0) // kinetic
#include <urdf_model/types.h>
#include <urdf_world/types.h>
#else
namespace urdf {
typedef boost::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;
typedef boost::shared_ptr<const Link> LinkConstSharedPtr;
typedef boost::shared_ptr<Joint> JointSharedPtr;
}
#endif

using namespace urdf;
using namespace std;


class UrdfModelMarker {
 public:
  //  UrdfModelMarker(string file);
  UrdfModelMarker(string file, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker(string model_name, string model_description, string model_file, string frame_id, geometry_msgs::PoseStamped  root_pose, geometry_msgs::Pose root_offset, double scale_factor, string mode, bool robot_mode, bool registration, vector<string> fixed_link, bool use_robot_description, bool use_visible_color, map<string, double> initial_pose_map, int index, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker();

  void addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, LinkConstSharedPtr link, bool root);
  void addInvisibleMeshMarkerControl(visualization_msgs::InteractiveMarker &int_marker, LinkConstSharedPtr link, const std_msgs::ColorRGBA &color);
  void addGraspPointControl(visualization_msgs::InteractiveMarker &int_marker, std::string link_frame_name_);

  void publishBasePose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void publishBasePose( geometry_msgs::Pose pose, std_msgs::Header header);
  void publishMarkerPose ( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishMarkerPose ( geometry_msgs::Pose pose, std_msgs::Header header, std::string marker_name);
  void publishMarkerMenu( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu );
  void publishMoveObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void publishJointState( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void republishJointState( sensor_msgs::JointState js);

  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id);

  void graspPoint_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string link_name);
  void moveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void setPoseCB();
  void setPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideAllMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void hideModelMarkerCB( const std_msgs::EmptyConstPtr &msg);
  void showModelMarkerCB( const std_msgs::EmptyConstPtr &msg);
  void setUrdfCB( const std_msgs::StringConstPtr &msg);

  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, geometry_msgs::Vector3 scale, const std_msgs::ColorRGBA &color);



  visualization_msgs::InteractiveMarkerControl makeCylinderMarkerControl(const geometry_msgs::PoseStamped &stamped, double length, double radius, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeBoxMarkerControl(const geometry_msgs::PoseStamped &stamped, Vector3 dim, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeSphereMarkerControl(const geometry_msgs::PoseStamped &stamped, double rad, const std_msgs::ColorRGBA &color, bool use_color);
  //joint state methods
  void publishJointState();
  void getJointState();
  void getJointState(LinkConstSharedPtr link);

  void setJointState(LinkConstSharedPtr link, const sensor_msgs::JointStateConstPtr &js);
  void setJointAngle(LinkConstSharedPtr link, double joint_angle);
  geometry_msgs::Pose getRootPose(geometry_msgs::Pose pose);
  geometry_msgs::PoseStamped getOriginPoseStamped();
  void setOriginalPose(LinkConstSharedPtr link);
  void addChildLinkNames(LinkConstSharedPtr link, bool root, bool init);
  void addChildLinkNames(LinkConstSharedPtr link, bool root, bool init, bool use_color, int color_index);

  void callSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::Transform transform);
  void callPublishTf();

  int main(string file);

  void graspPointCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void jointMoveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void resetMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void resetBaseMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void resetBaseMsgCB( const std_msgs::EmptyConstPtr &msg);
  void resetBaseCB();

  void resetRobotBase();
  void resetRootForVisualization();
  void registrationCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  
  void setRootPoseCB( const geometry_msgs::PoseStampedConstPtr &msg );
  void setRootPose( geometry_msgs::PoseStamped ps);
  void resetJointStatesCB( const sensor_msgs::JointStateConstPtr &msg, bool update_root);
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
  bool lockJointStates(std_srvs::EmptyRequest& req,
                       std_srvs::EmptyRequest& res);
  bool unlockJointStates(std_srvs::EmptyRequest& req,
                         std_srvs::EmptyRequest& res);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  
  /* diagnostics */
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  jsk_topic_tools::TimeAccumulator reset_joint_states_check_time_acc_;
  jsk_topic_tools::TimeAccumulator dynamic_tf_check_time_acc_;

  /* publisher */
  ros::Publisher pub_;
  ros::Publisher pub_move_;
  ros::Publisher pub_move_object_;
  ros::Publisher pub_move_model_;
  ros::Publisher pub_joint_state_;
  ros::Publisher pub_base_pose_;
  ros::Publisher pub_selected_;
  ros::Publisher pub_selected_index_;

  /* subscriber */
  ros::Subscriber sub_reset_joints_;
  ros::Subscriber sub_reset_joints_and_root_;
  ros::Subscriber sub_set_root_pose_;
  ros::Subscriber hide_marker_;
  ros::Subscriber show_marker_;
  ros::Subscriber sub_set_urdf_;

  ros::ServiceServer serv_reset_;
  ros::ServiceServer serv_set_;
  ros::ServiceServer serv_markers_set_;
  ros::ServiceServer serv_markers_del_;
  ros::ServiceServer serv_lock_joint_states_;
  ros::ServiceServer serv_unlock_joint_states_;
  boost::mutex joint_states_mutex_;
  bool is_joint_states_locked_;
  interactive_markers::MenuHandler model_menu_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  ros::ServiceClient dynamic_tf_publisher_client;
  ros::ServiceClient dynamic_tf_publisher_publish_tf_client;

  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;


  ModelInterfaceSharedPtr model;
  std::string model_name_;
  std::string model_description_;
  std::string frame_id_;
  std::string model_file_;
  geometry_msgs::Pose root_pose_;
  geometry_msgs::Pose root_pose_origin_;
  geometry_msgs::Pose root_offset_;
  geometry_msgs::Pose fixed_link_offset_; //used when fixel_link_ is used
  double scale_factor_;
  bool robot_mode_;
  bool registration_;
  bool use_dynamic_tf_;
  string mode_;
  vector<string> fixed_link_;
  bool use_robot_description_;
  bool use_visible_color_;
  std::string tf_prefix_;
  map<string, double> initial_pose_map_;
  int index_;
  ros::Time init_stamp_;

  //joint states
  sensor_msgs::JointState joint_state_;
  sensor_msgs::JointState joint_state_origin_;

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

#endif
