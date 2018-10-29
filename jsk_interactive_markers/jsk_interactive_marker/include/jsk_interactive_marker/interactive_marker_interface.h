#include <ros/ros.h>

#include <tf/tf.h>
//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>

#include <interactive_markers/menu_handler.h>
#include <jsk_interactive_marker/SetPose.h>
#include <jsk_interactive_marker/MarkerSetPose.h>

#include <math.h>
#include <jsk_interactive_marker/MarkerMenu.h>
#include <jsk_interactive_marker/MarkerPose.h>

#include <std_msgs/Int8.h>
#include "urdf_parser/urdf_parser.h"
#if ROS_VERSION_MINIMUM(1,12,0) // kinetic
#include <urdf_world/types.h>
#else
namespace urdf {
typedef boost::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;
}
#endif

class InteractiveMarkerInterface {
 private:
  struct MeshProperty{
    std::string link_name;
    std::string mesh_file;
    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

  };

  struct UrdfProperty{
    urdf::ModelInterfaceSharedPtr model;
    std::string root_link_name;
    geometry_msgs::Pose pose;
    double scale;
    std_msgs::ColorRGBA color;
    bool use_original_color;
  };

 public:
  visualization_msgs::InteractiveMarker make6DofControlMarker( std::string name, geometry_msgs::PoseStamped &stamped, float scale, bool fixed_position, bool fixed_rotation);

  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void proc_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int type );
  void pub_marker_tf ( std_msgs::Header header, geometry_msgs::Pose pose);
  void pub_marker_pose ( std_msgs::Header header, geometry_msgs::Pose pose, std::string name, int type );

  void pub_marker_menu(std::string marker,int menu, int type);
  void pub_marker_menu(std::string marker,int menu);
  
  void pub_marker_menuCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu );

  void pub_marker_menuCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int menu, int type);

  void IMSizeLargeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void IMSizeMiddleCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void IMSizeSmallCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb1( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeForceModeCb2( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  
  void targetPointMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void lookAutomaticallyMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void ConstraintCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void changeMoveArm( std::string m_name, int menu );
  
  void setOriginCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, bool origin_hand);

  void ikmodeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void toggleIKModeCb( const std_msgs::EmptyConstPtr &msg);
  void useTorsoCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void usingIKCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void marker_menu_cb( const jsk_interactive_marker::MarkerMenuConstPtr &msg);

  void updateHeadGoal( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateBase( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void updateFinger( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string hand);

  visualization_msgs::InteractiveMarker makeBaseMarker( const char *name, const geometry_msgs::PoseStamped &stamped, float scale, bool fixed);



  void changeMarkerForceMode( std::string mk_name , int im_mode);
  
  void toggleStartIKCb( const std_msgs::EmptyConstPtr &msg);

  void initControlMarkers(void);

  void initBodyMarkers(void);

  void initHandler(void);

  void changeMarkerMoveMode( std::string mk_name , int im_mode);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size, geometry_msgs::PoseStamped dist_pose);

  void changeMarkerOperationModelMode( std::string mk_name );

  //void addHandMarker(visualization_msgs::InteractiveMarkerControl &imc,std::vector < MeshProperty > mesh_vec, double mk_size);
  void addHandMarker(visualization_msgs::InteractiveMarker &im,std::vector < UrdfProperty > urdf_vec);
  void addSphereMarker(visualization_msgs::InteractiveMarker &im, double scale, std_msgs::ColorRGBA color);
  void makeCenterSphere(visualization_msgs::InteractiveMarker &mk, double mk_size);

  InteractiveMarkerInterface ();

  bool markers_set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
			jsk_interactive_marker::MarkerSetPose::Response &res );

  bool markers_del_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
			jsk_interactive_marker::MarkerSetPose::Response &res );

  void move_marker_cb ( const geometry_msgs::PoseStampedConstPtr &msg);

  bool set_cb ( jsk_interactive_marker::MarkerSetPose::Request &req,
                jsk_interactive_marker::MarkerSetPose::Response &res );

  bool reset_cb ( jsk_interactive_marker::SetPose::Request &req,
                  jsk_interactive_marker::SetPose::Response &res );

  void loadMeshFromYaml(XmlRpc::XmlRpcValue val, std::string name, std::vector<MeshProperty>& mesh);
  void loadUrdfFromYaml(XmlRpc::XmlRpcValue val, std::string name, std::vector<UrdfProperty>& mesh);
  void loadMeshes(XmlRpc::XmlRpcValue val);

  void makeIMVisible(visualization_msgs::InteractiveMarker &im);

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pub_;
  ros::Publisher pub_update_;
  ros::Publisher pub_move_;
  ros::ServiceServer serv_reset_;
  ros::ServiceServer serv_set_;
  ros::ServiceServer serv_markers_set_;
  ros::ServiceServer serv_markers_del_;
  ros::Subscriber sub_marker_pose_;
  ros::Subscriber sub_marker_menu_;
  ros::Subscriber sub_toggle_start_ik_;
  ros::Subscriber sub_toggle_ik_mode_;
  //tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;
  ros::ServiceClient dynamic_tf_publisher_client_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler menu_handler1;
  interactive_markers::MenuHandler menu_handler2;
  interactive_markers::MenuHandler menu_handler_force;
  interactive_markers::MenuHandler menu_handler_force1;
  interactive_markers::MenuHandler menu_handler_force2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_ik;

  interactive_markers::MenuHandler menu_head_;
  interactive_markers::MenuHandler::EntryHandle head_target_handle_;
  interactive_markers::MenuHandler::EntryHandle head_auto_look_handle_;

  interactive_markers::MenuHandler menu_head_target_;

  interactive_markers::MenuHandler menu_base_;
  interactive_markers::MenuHandler menu_finger_r_;
  interactive_markers::MenuHandler menu_finger_l_;

  // parameters
  std::string marker_name;
  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;
  bool fix_marker;
  interactive_markers::MenuHandler::EntryHandle h_mode_last;
  interactive_markers::MenuHandler::EntryHandle h_mode_last2;
  interactive_markers::MenuHandler::EntryHandle h_mode_last3;

  interactive_markers::MenuHandler::EntryHandle rotation_t_menu_;
  interactive_markers::MenuHandler::EntryHandle rotation_nil_menu_;

  interactive_markers::MenuHandler::EntryHandle use_torso_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_t_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_nil_menu_;
  interactive_markers::MenuHandler::EntryHandle use_fullbody_menu_;


  interactive_markers::MenuHandler::EntryHandle start_ik_menu_;
  interactive_markers::MenuHandler::EntryHandle stop_ik_menu_;

  int h_mode_rightarm;
  int h_mode_constrained;
  int h_mode_ikmode;
  int use_arm;

  

  std::list<visualization_msgs::InteractiveMarker> imlist;
  //interactive_markers::MenuHandler menu_handler;

  struct GripperState{
  GripperState() : on_(false), view_facing_(false), edit_control_(false), torso_frame_(false) {}

    bool on_;
    bool view_facing_;
    bool edit_control_;
    bool torso_frame_;
  };

  struct ControlState{
  ControlState() : posture_r_(false), posture_l_(false), torso_on_(false), head_on_(false),
      projector_on_(false), init_head_goal_(false), base_on_(true),  r_finger_on_(false), l_finger_on_(false), move_arm_(RARM), move_origin_state_(HAND_ORIGIN) {}

    void print()
    {
      ROS_DEBUG_NAMED("control_state", "gripper: on[%d|%d][%d], edit[%d|%d][%d], torso[%d|%d]",
                      l_gripper_.on_, r_gripper_.on_, dual_grippers_.on_, l_gripper_.edit_control_, r_gripper_.edit_control_, dual_grippers_.edit_control_, l_gripper_.torso_frame_, r_gripper_.torso_frame_);
      ROS_DEBUG_NAMED("control_state", "posture[%d|%d] torso[%d] base[%d] head[%d] projector[%d]",
                      posture_l_, posture_r_, torso_on_, base_on_, head_on_, projector_on_ );
    }

    enum MoveArmState { RARM, LARM, ARMS};
    enum MoveOriginState { HAND_ORIGIN, DESIGNATED_ORIGIN};

    MoveArmState move_arm_;
    MoveOriginState move_origin_state_;
    
    geometry_msgs::PoseStamped marker_pose_;

    bool posture_r_;
    bool posture_l_;
    bool torso_on_;
    bool head_on_;
    bool projector_on_;
    bool look_auto_on_;
    bool init_head_goal_;
    bool base_on_;
    bool planar_only_;
    bool r_finger_on_;
    bool l_finger_on_;
    GripperState dual_grippers_;
    GripperState r_gripper_;
    GripperState l_gripper_;
  };

  bool use_finger_marker_;
  bool use_body_marker_;
  bool use_center_sphere_;


  ControlState control_state_;

  geometry_msgs::PoseStamped head_goal_pose_;

  std::string hand_type_;

  std::string head_link_frame_;
  std::string head_mesh_;
  std::vector< MeshProperty > rhand_mesh_, lhand_mesh_;
  std::vector< UrdfProperty > rhand_urdf_, lhand_urdf_;

};

