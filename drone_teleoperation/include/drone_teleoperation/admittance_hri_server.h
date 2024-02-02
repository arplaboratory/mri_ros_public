#ifndef ADMITTANCE_HRI_SERVER_H
#define ADMITTANCE_HRI_SERVER_H



// ROS includes
// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
//#include "sensor_msgs/msg/Joy.hpp"
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
//Services 
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <mav_manager/Vec4.h>
#include <trackers_msgs/Transition.h>

#include <scene_understanding_pkg_msgs/RRTPathPoints2D.h>
#include <scene_understanding_pkg_msgs/RRTObstaclesCoo.h>
#include <scene_understanding_pkg_msgs/ObstacleRepForce.h>

#include <stdio.h>
#include <vector>
#include <fstream>
#include <numeric>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h> 
// #include "ros_visualization.h
#include "utils/admittance_utils.h"
#include "core/admittance_core.h"
#include "drone_teleoperation/core/Admittance_Controller.h"
#include "drone_teleoperation/utils/ros_visualization.h"
#include "drone_teleoperation/utils/log_files_writing.h"

#define C_PI (double)3.141592653589793

using namespace std;


namespace hri_admittance {
class HRIControlServer
{
  public:
  HRIControlServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, string path);
  virtual ~HRIControlServer() {}
  

  //Subscriber Callback Declaration 
  void quad_odom_callback(nav_msgs::Odometry msg);
  void Keyboard_input_callback(const std_msgs::Int32 msg);
  void Keyboard_input_yaw_callback(const std_msgs::Int32 msg); 
  void interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback pose);
  void haptic_command_callback(const geometry_msgs::PoseStamped position);
  void unity_marker_position_callback(const geometry_msgs::Point position);
  void obstacle_force_res_callback(const scene_understanding_pkg_msgs::ObstacleRepForce obstacles);
  void visualize_goals_callback(const std_msgs::Bool msg);
  void rotate_yaw_callback(const std_msgs::Bool msg);
  
  void send_position_command();
  void send_perceived_force();
  void publish_position_cmd(quadrotor_msgs::PositionCommand msg);
  void publish_f_obs_history(const ros::TimerEvent& event);
  void publish_odom_to_unity(const ros::TimerEvent& event);
  bool disable_tracker_manager();
  bool hovering();

  
  
  int FPVI_exit_task_counter_=0;
  int APVI_exit_task_counter_=0;

  protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  ros::Timer publish_f_obs_history_timer;
  ros::Timer publish_odom_to_unity_timer;

  //Services 
  ros::ServiceClient disable_tracker;
  ros::ServiceClient hovering_srv;
  
  ros::Subscriber quad_odom;
  ros::Subscriber rviz_interactiveMarker;
  ros::Subscriber Keyboard_input_various; //Implement key input for key switcher and yaw with the same topic
  ros::Subscriber Keyboard_input_change_yaw;
  ros::Subscriber under_teleoperation_unity; //Verify if required also on unity side 
  ros::Subscriber interaction_marker_pose_from_unity;
  ros::Subscriber obtscles_force_res;
  ros::Subscriber visualize_goals;
  ros::Subscriber haptic_odom;
  ros::Subscriber rotate_yaw_180;

  ros::Publisher position_cmd_pub;
  ros::Publisher end_planning_mode;
  ros::Publisher interactive_marker_position;
  ros::Publisher odom_to_unity_pub;
  ros::Publisher force_perceived;

  
  AdmittanceUtils utils;
  AdmittanceCore IC_core;
  ROSVisualization visualizer;

  

  
  string frame_id;
   //Robot Variables 
   Eigen::Vector3f kx, kv;


   Eigen::Vector3f quad_position;
   Eigen::Vector3f quad_euler_orientation;
   Eigen::Vector4f quad_quat_orientation;

   Eigen::Vector3f quad_lin_vel;
   Eigen::Vector3f quad_ang_vel;

   Eigen::Vector3f des_lin_vel;
   Eigen::Vector3f des_position;

   //Save odom to republish
   nav_msgs::Odometry odom_to_unity;
   
   float des_yaw = 0.0;
   
   //Timing 
   double dt = 0.01; 
   double t_stamp_old = 0.0;
   
   //Rviz Interactive Marker 
   Eigen::Vector3f marker_position;
   Eigen::Vector3f marker_unity_position;
  Eigen::Vector3f haptic_marker_position;

   //Global force feedback 
    Eigen::Vector3f force_global;
   
   //Obstacle Force
   Eigen::Vector3f force_obs_vec;
   float force_obs_Magn;
   Eigen::Vector2f p_distance;
   Eigen::Vector2f p_distance_dot;
   /*
   Scenarios:
   Only Sim: 1
   Sim + Unity or Holo: 2
   Real + Rviz: 3
   Real + Holo: 4

   Modality:
   FPVI: 1
   APVI: 2
   */
   int scenario = 0;
   int modality = 1;
   int state = 0;
   int pub_path_history_every_nsec = 2;
   float unity_odom_rate = 10;
   int counter = 0;

  
   bool tracker_transition_success = false;
   bool init = true;
   bool init_timer_odom = true;

   //Change Yaw
   bool increasing_yaw = false;
   bool decreasing_yaw = false; 
   bool under_unity_manipulation;

   //
   bool haptic_feedback_flag = false;

 


};

}
#endif