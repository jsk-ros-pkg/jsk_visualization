#include <ros/ros.h>
#include <jsk_interactive_marker/interactive_point_cloud.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_marker_control");

  InteractivePointCloud interactive_point_cloud("interactive_manipulation_snapshot",
						"interactive_point_cloud", "interactive_manipulation_snapshot_server", 
						"/odom_combined");
  ros::Duration(1.0).sleep();
  ros::spin();
}

