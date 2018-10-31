#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <dynamic_tf_publisher/SetDynamicTF.h>

using namespace visualization_msgs;

class UrdfControlMarker {
public:
  UrdfControlMarker();
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void makeControlMarker( bool fixed );
  void set_pose_cb( const geometry_msgs::PoseStampedConstPtr &msg );
  void show_marker_cb ( const std_msgs::BoolConstPtr &msg);
  void markerUpdate ( std_msgs::Header header, geometry_msgs::Pose pose);
  void publish_pose_cb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void callDynamicTf(const std_msgs::Header& header,
                     const std::string& child_frame,
                     const geometry_msgs::Transform& pose,
                     bool until_success = false);
private:
  bool use_dynamic_tf_, move_2d_;
  tf::TransformListener tf_listener_;
  ros::ServiceClient dynamic_tf_publisher_client_;
  ros::Subscriber sub_set_pose_, sub_show_marker_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_selected_pose_;
  std::string frame_id_, marker_frame_id_, fixed_frame_id_;
  std::string center_marker_;
  std_msgs::ColorRGBA color_;
  bool mesh_use_embedded_materials_;
  double marker_scale_, center_marker_scale_;
  interactive_markers::MenuHandler marker_menu_;
  geometry_msgs::Pose center_marker_pose_;
};

UrdfControlMarker::UrdfControlMarker() : nh_(), pnh_("~"){
  server_.reset( new interactive_markers::InteractiveMarkerServer("urdf_control_marker","",false) );

  pnh_.param("move_2d", move_2d_, false);
  pnh_.param("use_dynamic_tf", use_dynamic_tf_, false);
  pnh_.param<std::string>("frame_id", frame_id_, "/map");
  pnh_.param<std::string>("fixed_frame_id", fixed_frame_id_, "/odom_on_ground");
  pnh_.param<std::string>("marker_frame_id", marker_frame_id_, "/urdf_control_marker");
  pnh_.param<std::string>("center_marker", center_marker_, "");
  pnh_.param("marker_scale", marker_scale_, 1.0);
  pnh_.param("center_marker_scale", center_marker_scale_, 1.0);
  //set color
  if(pnh_.hasParam("center_marker_color")){
    mesh_use_embedded_materials_ = false;
    std::map<std::string, double> color_map;
    pnh_.getParam("color", color_map);
    color_.r = color_map["r"];
    color_.g = color_map["g"];
    color_.b = color_map["b"];
    color_.a = color_map["a"];
  }else{
    mesh_use_embedded_materials_ = true;
  }

  //set pose
  if(pnh_.hasParam("center_marker_pose")){
    XmlRpc::XmlRpcValue pose_v;
    pnh_.param("center_marker_pose", pose_v, pose_v);
    center_marker_pose_ = im_utils::getPose(pose_v);

  }else{
    center_marker_pose_.orientation.w = 1.0;
  }

  //dynamic_tf_publisher
  if (use_dynamic_tf_) {
    dynamic_tf_publisher_client_ = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf", true);
    dynamic_tf_publisher_client_.waitForExistence();
  }
  pub_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_selected_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("selected_pose", 1);
  sub_set_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped> ("set_pose", 1, boost::bind( &UrdfControlMarker::set_pose_cb, this, _1));
  sub_show_marker_ = pnh_.subscribe<std_msgs::Bool> ("show_marker", 1, boost::bind( &UrdfControlMarker::show_marker_cb, this, _1));

  marker_menu_.insert( "Publish Pose",
		       boost::bind( &UrdfControlMarker::publish_pose_cb, this, _1) );

  makeControlMarker( false );

  ros::spin();
  server_.reset();
}

void UrdfControlMarker::publish_pose_cb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::PoseStamped ps;
  ps.header = feedback->header;
  ps.pose = feedback->pose;
  pub_selected_pose_.publish(ps);
}

void UrdfControlMarker::set_pose_cb ( const geometry_msgs::PoseStampedConstPtr &msg){
  // Convert PoseStamped frame_id to fixed_frame_id_
  geometry_msgs::PoseStamped in_pose(*msg);
  geometry_msgs::PoseStamped out_pose;
  in_pose.header.stamp = ros::Time(0.0);
  tf_listener_.transformPose(fixed_frame_id_, in_pose, out_pose);
  out_pose.header.stamp = msg->header.stamp;
  server_->setPose("urdf_control_marker", out_pose.pose, out_pose.header);
  server_->applyChanges();
  markerUpdate(out_pose.header, out_pose.pose);
}

void UrdfControlMarker::show_marker_cb ( const std_msgs::BoolConstPtr &msg){
  if(msg->data){
    makeControlMarker( false );
  }else{
    server_->clear();
    server_->applyChanges();
  }
}


void UrdfControlMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  markerUpdate( feedback->header, feedback->pose);
}

void UrdfControlMarker::callDynamicTf(
  const std_msgs::Header& header,
  const std::string& child_frame,
  const geometry_msgs::Transform& transform,
  bool until_success)
{
  dynamic_tf_publisher::SetDynamicTF SetTf;
  SetTf.request.freq = 20;
  SetTf.request.cur_tf.header = header;
  SetTf.request.cur_tf.child_frame_id = child_frame;
  SetTf.request.cur_tf.transform = transform;
  ros::Rate r(1);
  while (true) {
    if (!dynamic_tf_publisher_client_.call(SetTf)) {
      ROS_ERROR("Failed to call dynamic_tf: %s => %s",
                header.frame_id.c_str(),
                child_frame.c_str());
      // Re-create connection to service server
      dynamic_tf_publisher_client_ = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf", true);
      dynamic_tf_publisher_client_.waitForExistence();
      if (!until_success) {
        break;
      }
    }
    else {
      break;
    }
    r.sleep();
  }
}

void UrdfControlMarker::markerUpdate ( std_msgs::Header header, geometry_msgs::Pose pose){
  if (use_dynamic_tf_){
    geometry_msgs::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
    callDynamicTf(header, marker_frame_id_, transform);
  }
  geometry_msgs::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  pub_pose_.publish(ps);
}




void UrdfControlMarker::makeControlMarker( bool fixed )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.scale = marker_scale_;

  int_marker.name = "urdf_control_marker";

  //add center marker
  if(center_marker_ != ""){
    InteractiveMarkerControl center_marker_control;
    center_marker_control.name = "center_marker";
    center_marker_control.always_visible = true;
    center_marker_control.orientation.w = 1.0;
    center_marker_control.orientation.y = 1.0;

    if(move_2d_){
      center_marker_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    }else{
      center_marker_control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    }
    Marker center_marker;
    center_marker.type = Marker::MESH_RESOURCE;
    center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = center_marker_scale_;
    center_marker.mesh_use_embedded_materials = mesh_use_embedded_materials_;
    center_marker.mesh_resource = center_marker_;
    center_marker.pose = center_marker_pose_;
    center_marker.color = color_;

    center_marker_control.markers.push_back(center_marker);
    int_marker.controls.push_back(center_marker_control);
  }

  InteractiveMarkerControl control;

  if ( fixed )
    {
      int_marker.name += "_fixed";
      control.orientation_mode = InteractiveMarkerControl::FIXED;
    }
  control.always_visible = true;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind( &UrdfControlMarker::processFeedback, this, _1));
  marker_menu_.apply(*server_, int_marker.name);
  server_->applyChanges();
  if (use_dynamic_tf_) {
    /* First initialize dynamic tf as identity */
    std_msgs::Header header;
    header.frame_id = fixed_frame_id_;
    header.stamp = ros::Time::now();
    geometry_msgs::Transform transform;
    transform.rotation.w = 1.0;
    callDynamicTf(header, marker_frame_id_, transform, true);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  UrdfControlMarker ucm;
}
