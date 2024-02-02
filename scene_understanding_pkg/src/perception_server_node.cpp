#include "perception_server.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "perception_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  perception::PerceptionServer node(nh, nh_private);

  ros::spin();
  return 0;
}