#include <jsk_interactive_marker/transformable_interactive_server.h>

using namespace jsk_interactive_marker;

int main(int argc, char** argv)
{
#if ROS_VERSION_MAJOR != 1
  rclcpp::init(argc, argv);
  ROS1_ROS2_COMPAT::g_node = rclcpp::Node::make_shared("simple_marker");
#else
  ros::init(argc, argv, "simple_marker");
#endif

  TransformableInteractiveServer* ti_server = new TransformableInteractiveServer();

  ti_server->run();
#if ROS_VERSION_MAJOR != 1
  rclcpp::shutdown();
#endif

}
