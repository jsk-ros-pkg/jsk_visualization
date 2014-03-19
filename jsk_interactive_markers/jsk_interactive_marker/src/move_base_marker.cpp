#include <iostream>
#include <interactive_markers/tools.h>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/move_base_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>


MoveBaseMarker::MoveBaseMarker (string model_name, string model_file, string frame_id, geometry_msgs::Pose root_pose, double scale_factor, string mode, bool robot_mode, bool registration, string fixed_link, bool use_robot_description, bool use_visible_color, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : UrdfModelMarker(model_name, model_file, frame_id, root_pose, scale_factor, mode, robot_mode, registration, fixed_link, use_robot_description, use_visible_color, server) {
  pub_base_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("base_pose", 1);
  sub_marker_pose_ =  pnh_.subscribe<jsk_interactive_marker::MarkerPose> ("pose", 1, boost::bind( &MoveBaseMarker::markerPoseCB ,this, _1) );
}

void MoveBaseMarker::markerPoseCB (  const jsk_interactive_marker::MarkerPoseConstPtr &msg){
  if( msg -> marker_name == tf_prefix_ + model->getRoot()->name){
    pub_base_pose_.publish(msg->pose);
  }
}
