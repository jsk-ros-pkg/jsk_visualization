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
  UrdfModelMarker(string model_name, string model_file, string frame_id, geometry_msgs::Pose root_pose, double scale_factor, bool robot_mode, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker();

  void addMoveMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, bool root);
  void addInvisibleMeshMarkerControl(visualization_msgs::InteractiveMarker &int_marker, boost::shared_ptr<const Link> link, const std_msgs::ColorRGBA &color);
  void addGraspPointControl(visualization_msgs::InteractiveMarker &int_marker, std::string link_frame_name_);

  void publishMarkerPose ( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void publishMarkerMenu( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu );
  void publishMoveObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void publishJointState( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string parent_frame_id, string frame_id);

  void graspPoint_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, string link_name);
  void moveCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void setPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void hideAllMarkerCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );



  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color, bool use_color);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale);
  visualization_msgs::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::PoseStamped &stamped, float scale, const std_msgs::ColorRGBA &color);

  void getJointState(boost::shared_ptr<const Link> link, sensor_msgs::JointState &js);
  void setJointState(boost::shared_ptr<const Link> link, const sensor_msgs::JointStateConstPtr &js);
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
  
  void resetJointStatesCB( const sensor_msgs::JointStateConstPtr &msg);
  
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pub_;
  ros::Publisher pub_move_;
  ros::Publisher pub_move_object_;
  ros::Publisher pub_joint_state_;
  
  ros::Subscriber sub_reset_joints_;

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

  struct graspPoint{
    graspPoint(){
      displayMoveMarker = false;
      displayGraspPoint = false;
    }
    bool displayGraspPoint;
    bool displayMoveMarker;
    geometry_msgs::Pose pose;
  };


  struct linkProperty{
    linkProperty(){
      displayMoveMarker = false;
    }
    bool displayMoveMarker;
    graspPoint gp;
    string frame_id;
    //pose from frame_id
    geometry_msgs::Pose pose;
    geometry_msgs::Pose origin;
    geometry_msgs::Pose initial_pose;
    urdf::Vector3 joint_axis;
    double joint_angle;
    int rotation_count;
  };

  map<string, linkProperty> linkMarkerMap;
};

geometry_msgs::Pose getPose( XmlRpc::XmlRpcValue val);
double getXmlValue( XmlRpc::XmlRpcValue val );
