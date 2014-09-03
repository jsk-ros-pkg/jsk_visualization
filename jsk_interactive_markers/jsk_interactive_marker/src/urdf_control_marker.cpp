#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <dynamic_tf_publisher/SetDynamicTF.h>
using namespace visualization_msgs;

class UrdfControlMarker {
public:
  UrdfControlMarker();
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void makeControlMarker( bool fixed );
  void set_pose_cb( const geometry_msgs::PoseStampedConstPtr &msg );
  void markerUpdate ( std_msgs::Header header, geometry_msgs::Pose pose);

private:
  bool use_dynamic_tf_, move_2d_;
  ros::ServiceClient dynamic_tf_publisher_client_;
  ros::Subscriber sub_set_pose_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_;
  std::string frame_id_, marker_frame_id_;
};

UrdfControlMarker::UrdfControlMarker() : nh_(), pnh_("~"){
  server_.reset( new interactive_markers::InteractiveMarkerServer("urdf_control_marker","",false) );

  pnh_.param("move_2d", move_2d_, false);
  pnh_.getParam("use_dynamic_tf", use_dynamic_tf_);
  pnh_.param<std::string>("frame_id", frame_id_, "/map");
  pnh_.param<std::string>("marker_frame_id", marker_frame_id_, "/urdf_control_marker");
  if (use_dynamic_tf_) {
    ros::service::waitForService("set_dynamic_tf", -1);
    dynamic_tf_publisher_client_ = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf", true);
  }
  pub_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  sub_set_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped> ("set_pose", 1, boost::bind( &UrdfControlMarker::set_pose_cb, this, _1));


  makeControlMarker( false );
  server_->applyChanges();
  ros::spin();
  server_.reset();
}

void UrdfControlMarker::set_pose_cb ( const geometry_msgs::PoseStampedConstPtr &msg){
  server_->setPose("urdf_control_marker", msg->pose, msg->header);
  server_->applyChanges();
  markerUpdate( msg->header, msg->pose);
}


void UrdfControlMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  markerUpdate( feedback->header, feedback->pose);
}

void UrdfControlMarker::markerUpdate ( std_msgs::Header header, geometry_msgs::Pose pose){
  dynamic_tf_publisher::SetDynamicTF SetTf;
  SetTf.request.freq = 20;
  SetTf.request.cur_tf.header = header;
  SetTf.request.cur_tf.child_frame_id = marker_frame_id_;
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;

  SetTf.request.cur_tf.transform = transform;
  if (use_dynamic_tf_){
    dynamic_tf_publisher_client_.call(SetTf);
  }

  geometry_msgs::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  pub_pose_.publish(ps);

  server_->applyChanges();
}




void UrdfControlMarker::makeControlMarker( bool fixed )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.scale = 1;

  int_marker.name = "urdf_control_marker";

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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_controls");
  UrdfControlMarker ucm;
}
