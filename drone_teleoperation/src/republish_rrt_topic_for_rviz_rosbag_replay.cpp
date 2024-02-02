#include "ros/ros.h"

#include <sstream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <sys/stat.h> 
#include <chrono>
#include <std_msgs/Int8.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "std_srvs/SetBool.h"



using namespace std;
using namespace std::chrono;


int main(int argc, char **argv)
{

  
  ros::Rate r(50);
  while (nh.ok())
  {


  
   ros::spinOnce();
   r.sleep();
}
}
