
#include "ros/ros.h"
//#include </home/arpl/luca_ws/devel/include/scene_understanding_pkg_msgs/MeshPos.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <sys/stat.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include </home/luca/luca_ws/src/scene_understanding_pkg/include/replay_rosbag.h>




int main(int argc, char **argv)
{
  ros::init(argc, argv,"ReplayRosbagWithCorrectTimeStamp");
  

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
   
  
  ReplayRosBag replay_rosbag;
 
  ros::Rate r(50);
  while (nh.ok())
  {
   
    
    //kimera_data.publish_realsense_tf();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}