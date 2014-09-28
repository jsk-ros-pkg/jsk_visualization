#include <ros/ros.h>
#include <jsk_interactive_marker/transformable_interactive_server.h>

using namespace jsk_interactive_marker;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  TransformableInteractiveServer* ti_server = new TransformableInteractiveServer();

  ti_server->run();

}
