#ifndef DRONE_VOXL_H
#define DRONE_VOXL_H



// ROS includes
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
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


#define C_PI (double)3.141592653589793

using namespace std;

class arpl_drone_voxl
{
  ros::NodeHandle nh_;

  tf::TransformBroadcaster voxl_odom_broadcaster;

  //Only in simuation case 
  tf::TransformBroadcaster world_to_map_tf;
  tf::TransformBroadcaster map_to_mocap_tf;
  tf::TransformBroadcaster mocap_to_mocap_sr_tf;
  tf::TransformBroadcaster mocap_sr_to_simulator_tf;
  
  //Services
  ros::ServiceClient take_off_srv;
  ros::ServiceClient motors_on_srv;
  ros::ServiceClient goTo_srv;
  ros::ServiceClient disable_tracker;
  //Define Services required to safe fly
  
  //Subscribers 
  ros::Subscriber quad_sim_odom_sub;
  ros::Subscriber interactiveMarker;
  ros::Subscriber Keyboard_input;
  ros::Subscriber Keyboard_input_to_change_yaw;
  ros::Subscriber under_teleoperation_unity; 
  ros::Subscriber interactive_marker_unity_position_world;
  ros::Subscriber rrt_path_2D;
  ros::Subscriber rrt_start_assistive_modality;
  ros::Subscriber rrt_goal_reached;
  ros::Subscriber rrt_virtual_obstacles;
  ros::Subscriber rrt_final_goal;
  ros::Subscriber take_off_keyboard;
  ros::Subscriber obtscles_force_res;
  ros::Publisher position_cmd_pub;


  public:
  ros::Publisher line_to_follow_marker;
  ros::Publisher force_arrow_marker;
  ros::Publisher force_obs_arrow_marker_hist;
  ros::Publisher virtual_obstacle_marker_1;
  ros::Publisher virtual_obstacle_marker_2;
   ros::Publisher virtual_obstacle_marker_3;
   ros::Publisher rrt_fixed_marker_for_manouver_publisher;
  ros::Publisher path2D_to_unity;
  ros::Publisher exit_from_rrt_search_mode;
  ros::Publisher QuadrotorMarker;
  ros::Publisher final_goal_marker;
  ros::Publisher interactive_marker_position;
  ros::Publisher rrt_inside_bypass_manouver;
  ros::Publisher rrt_marker_for_manouver_in_unity;
  ros::Publisher publish_pose_history;



  geometry_msgs::Point position_GF;
  geometry_msgs::Point velocity_GF;
  geometry_msgs::Point ang_velocity_GF;
  geometry_msgs::Quaternion quat_orientation;
  geometry_msgs::Point rpy_orientation;
  
  //Recovery data 
  geometry_msgs::Point old_pos_GF_for_hovering;
  geometry_msgs::Point old_rpy_orientation_for_hovering;

  geometry_msgs::Point des_pos_GF_old;
    geometry_msgs::Point des_user_pos_GF_old;
  //Desired Positions 
  geometry_msgs::Point des_pos_GF;
  geometry_msgs::Vector3 des_vel_GF;
  geometry_msgs::Vector3 des_acc_GF;
  geometry_msgs::Vector3 des_jerk_GF;
  
    // Desired user position 
  geometry_msgs::Point des_user_pos_GF;
  geometry_msgs::Vector3 des_user_vel_GF;
  geometry_msgs::Vector3 des_user_acc_GF;
 

  //Interactive Marker Desired Position 
  geometry_msgs::Point marker_position;

  //Interactive Marker from Unity
  geometry_msgs::Point marker_unity_position;
  
    float d_output = 0.0; //Damping output in manula driving case
  float gamma_output = 0.0; //Gamma output in manula droving case


  Eigen::Vector3f kx, kv;
  float des_yaw = 0.0;
  float des_yaw_dot = 0.0;
  float angle_rad_horizon = 0.0;
  float yaw_drone_initial = 0.0;
  int rrt_counter_case = 0;
  int counter_yaw_rate = 0;

  float max_vel_x = 0.6;
  float max_vel_y = 0.6;
  float max_vel_z = 0.6;  
  
   float final_angle = 0; 
  float old_time_stamp = 0.0;
  float dt = 0.2; 
  double dt_safe = 0.02; 
  vector<double> dt_vec; 

  
  int case_selected = 0; 
  int avg_stamps = 20;                                                                                                                      
  
    //FIltering the target positions 
  vector<double> target_pos_x_GF_vec; 
  vector<double> target_pos_y_GF_vec; 
  vector<double> target_pos_z_GF_vec;   

  vector<double> target_pos_x_GF_vec_rrt;
  vector<double> target_pos_y_GF_vec_rrt;
  vector<double> target_pos_z_GF_vec_rrt;

  vector<double> target_vel_x_GF_vec_rrt;
  vector<double> target_vel_y_GF_vec_rrt;
  vector<double> target_vel_z_GF_vec_rrt;


   //Filtering the target velocities
  vector<double> target_vel_x_GF_vec; 
  vector<double> target_vel_y_GF_vec; 
  vector<double> target_vel_z_GF_vec;                                                                                                         
  
    //Average the target accelration 
  vector<double> target_acc_x_GF_vec; 
  vector<double> target_acc_y_GF_vec; 
  vector<double> target_acc_z_GF_vec;                                                                                                         
  

  //Store the points from the rrt path
  vector<geometry_msgs::Point> rrt_path_points;    


  geometry_msgs::Vector3 target_vel_GF;
  geometry_msgs::Vector3 target_vel_GF_old;
  geometry_msgs::Vector3 target_acc_GF;
  geometry_msgs::Vector3 target_acc_GF_old;
  
    // User target velocities in trajectory case 2
  geometry_msgs::Vector3 target_user_vel_GF;
  geometry_msgs::Vector3 target_user_vel_GF_old;
  geometry_msgs::Vector3 target_user_acc_GF;
  geometry_msgs::Vector3 target_user_acc_GF_old;

 

   //Virtual Obstacles Vectors
  vector<geometry_msgs::Point> virtual_obstacles_vec; //each obtsacle is described by two points 
  vector<geometry_msgs::Point> virtual_obstacles_vec_1;
  vector<geometry_msgs::Point> virtual_obstacles_vec_2;
  int virtual_obstacles_number = 0;

  vector<double> gamma_vector;
  geometry_msgs::Vector3 force_obs_res;
  float force_obs_Magn = 0.0;
  vector<double> fx_obs_vec;
  vector<double> fy_obs_vec;
  int counter_f_filtered = 0;
  int max_counter_f_filt_value = 150; //3 sec

  geometry_msgs::Point final_goal;
  //Admittance control output
  geometry_msgs::Point comm_pos_GF;
  geometry_msgs::Vector3 comm_vel_GF;
  
  geometry_msgs::Pose quadrotor_pose;

   string topic_service_take_off_sim = "/quadrotor/mav_services/takeoff";
   string topic_service_take_off_voxl = "/voxl3/mav_services/takeoff";
   string topic_service_motors_sim = "/quadrotor/mav_services/motors";
   string topic_service_motors_voxl = "/voxl3/mav_services/motors";
   string topic_service_gotTo_sim = "/quadrotor/mav_services/goTo";
   string topic_service_gotTo_voxl = "/voxl3/mav_services/goTo";
   string topic_service_transition_sim = "/quadrotor/trackers_manager/transition";
   string topic_service_transition_voxl = "/voxl3/trackers_manager/transition";
   string topic_odom_sim = "/quadrotor/odom";
   string topic_odom_voxl = "/voxl3/odom";
   string topic_position_cmd_sim = "/quadrotor/position_cmd";
   string topic_position_cmd_voxl = "/voxl3/position_cmd";
    
  string topic_service_take_off; 
  string topic_service_motors;
  string topic_service_gotTo;
  string topic_service_transition;
  string topic_odom;
  string topic_position_cmd;


  string frame_id = "/simulator";
  string mesh_directory; 

  bool sim_mode = true;
  bool init_test = true;
  bool holo_voxl_test = false;
  bool rviz_real_test = false;
  bool allow_take_off= false;
     //RRT final goal reached fla 
  bool rrt_final_goal_reached = false;
  bool rrt_start_assistive_guidance = false;
  bool rrt_exit_assistive_guidance = false; //True whe triggered by the user inputy
  bool publish_rrt_final_goal_marker_flag  = false;
  
     //Increasing decreasing yaw 
  bool increasing_yaw = false;
  bool decreasing_yaw = false;

  bool under_unity_manipulation = false;
  bool start_motors;
  bool disable_tracker_flag = true;
  bool flagQuadSimOdom = false;
  bool flagMarkerPosition = false;
  bool flagCaseSelectedKeyboard = false;
   bool flagDroneUnderManipulationFlag = false;
  bool flagInteractiveMarkerPosition = false;
   bool flagRRTPath = false; 
   bool flagRRTStartAssistiveGuidance = false;
   bool flagRRTFinalGoal = false;
    bool flagRRTVirtualObstacles = false;
    bool flagRRTfinalgoal = false;
 bool flagObstacleForceResReceived = false;
  arpl_drone_voxl(bool sim_mode)
  {
    if (sim_mode == true)
    {
        topic_service_take_off = topic_service_take_off_sim;
        topic_service_motors  = topic_service_motors_sim;
        topic_service_gotTo = topic_service_gotTo_sim;
        topic_service_transition  = topic_service_transition_sim;
        topic_odom = topic_odom_sim;
        topic_position_cmd = topic_position_cmd_sim;
    }
    else
    {
        topic_service_take_off = topic_service_take_off_voxl;
        topic_service_motors  = topic_service_motors_voxl;
        topic_service_gotTo = topic_service_gotTo_voxl;
        topic_service_transition  = topic_service_transition_voxl;
        topic_odom = topic_odom_voxl;
        topic_position_cmd = topic_position_cmd_voxl;

    }


      //Services 
      take_off_srv = nh_.serviceClient<std_srvs::Trigger> (topic_service_take_off); ///voxl3/mav_services/takeoff
      motors_on_srv  = nh_.serviceClient<std_srvs::SetBool> (topic_service_motors); // /voxl3/mav_services/motors
      goTo_srv = nh_.serviceClient<mav_manager::Vec4> (topic_service_gotTo); // /voxl3/mav_services/goTo
      disable_tracker = nh_.serviceClient<trackers_msgs::Transition> (topic_service_transition); // /voxl3/trackers_manager/transition

	  
         
      //Subscribers
      //quad_sim_odom_sub =  nh_.subscribe("/voxl3/odom", 10, &arpl_drone_voxl::quad_sim_odom_callback, this);
      quad_sim_odom_sub =  nh_.subscribe(topic_odom, 10, &arpl_drone_voxl::quad_sim_odom_callback, this); // from Vicon   // /voxl3/odom

      interactiveMarker = nh_.subscribe("/basic_controls/feedback", 1, &arpl_drone_voxl::interactive_marker_feedback, this);
      Keyboard_input = nh_.subscribe("/keyboard_input/case_switcher", 1, &arpl_drone_voxl::Keyboard_input_callback, this);
      Keyboard_input_to_change_yaw =  nh_.subscribe("/keyboard_input/change_yaw", 1, &arpl_drone_voxl::Keyboard_change_yaw_callback, this);
      take_off_keyboard =  nh_.subscribe("/keyboard_input/take_off", 1, &arpl_drone_voxl::Keyboard_allow_take_off, this);
      under_teleoperation_unity = nh_.subscribe("/unity_to_ros/drone_under_teleoperation", 1, &arpl_drone_voxl::drone_under_unity_teleoperation_callback, this);
      interactive_marker_unity_position_world = nh_.subscribe("/unity_to_ros/interactive_marker_position", 1, &arpl_drone_voxl::interactive_marker_position_callback, this);
      rrt_path_2D =  nh_.subscribe("rrt/path", 1, &arpl_drone_voxl::rrt_local_path_callback, this);
      rrt_start_assistive_modality = nh_.subscribe("/rrt/start_assistive_guidance", 1, &arpl_drone_voxl::rrt_start_assistive_guidance_callback, this);
      rrt_goal_reached = nh_.subscribe("rrt/final_goal_reached", 1, &arpl_drone_voxl::rrt_final_goal_reached_callback, this);
      rrt_virtual_obstacles = nh_.subscribe("/rrt/virtual_obstacles_coo", 10, &arpl_drone_voxl::rrt_virtual_obstacles_callback, this);
      rrt_final_goal = nh_.subscribe("/rrt/final_goal", 10, &arpl_drone_voxl::rrt_final_goal_position_callback, this);
      obtscles_force_res = nh_.subscribe("/obstacles_force_field", 1, &arpl_drone_voxl::obstacle_force_res_callback, this);
      

      position_cmd_pub =  nh_.advertise<quadrotor_msgs::PositionCommand>(topic_position_cmd, 1); // /voxl3/position_cmd"
      line_to_follow_marker = nh_.advertise<visualization_msgs::Marker>("/visualization_marker/line_to_follow", 10);
      force_arrow_marker = nh_.advertise<visualization_msgs::Marker>("/visualization_marker/force_arrow", 10);
      force_obs_arrow_marker_hist = nh_.advertise< geometry_msgs::PoseArray>("/PoseArray/force_obs_arrow_hist", 10);
      virtual_obstacle_marker_1 = nh_.advertise<visualization_msgs::Marker>("/rrt/virtual_obstacle_marker_1", 10);
      virtual_obstacle_marker_2 = nh_.advertise<visualization_msgs::Marker>("/rrt/virtual_obstacle_marker_2", 10);
      virtual_obstacle_marker_3 = nh_.advertise<visualization_msgs::Marker>("/rrt/virtual_obstacle_marker_3", 10);
      path2D_to_unity = nh_.advertise<scene_understanding_pkg_msgs::RRTPathPoints2D>("/rrt/path2D_to_unity", 10);
      exit_from_rrt_search_mode = nh_.advertise<std_msgs::Bool>("/rrt/exit_search_mode", 10);
      QuadrotorMarker = nh_.advertise<visualization_msgs::Marker>("/tele_control/robot", 10);
      final_goal_marker = nh_.advertise<visualization_msgs::Marker>("/rrt/final_goal_marker", 10);
      interactive_marker_position = nh_.advertise<geometry_msgs::Point>("/interactive_marker_position", 10);
      rrt_inside_bypass_manouver = nh_.advertise<std_msgs::Bool>("/rrt/inside_bypass_manouver", 10);
      rrt_fixed_marker_for_manouver_publisher =  nh_.advertise<visualization_msgs::Marker>("/rrt/fxed_marker_for_manouver", 10);
      rrt_marker_for_manouver_in_unity =   nh_.advertise<geometry_msgs::Point>("/rrt/fxed_marker_for_manouver_to_unity", 10);
      publish_pose_history = nh_.advertise<nav_msgs::Path>("/visualization_marker/robot_path", 10);
  }

 ~arpl_drone_voxl()
  {
  
  }



double average(std::vector<double> const& v){
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<double>(v.size());
    return accumulate(v.begin(), v.end(), 0.0) / v.size();
}

void quad_sim_odom_callback(nav_msgs::Odometry msg)
{
    //Obtain Position of quadrotor 
   
    position_GF.x = msg.pose.pose.position.x;
    position_GF.y = msg.pose.pose.position.y;
    position_GF.z = msg.pose.pose.position.z;
    
  
    quat_orientation.x = msg.pose.pose.orientation.x; 
    quat_orientation.y = msg.pose.pose.orientation.y;
    quat_orientation.z = msg.pose.pose.orientation.z;
    quat_orientation.w = msg.pose.pose.orientation.w;


    //Obtain Velocities
    velocity_GF.x = msg.twist.twist.linear.x;
    velocity_GF.y = msg.twist.twist.linear.y;
    velocity_GF.z = msg.twist.twist.linear.z;
    
    ang_velocity_GF.x  = msg.twist.twist.angular.x;
    ang_velocity_GF.y  =  msg.twist.twist.angular.y;
    ang_velocity_GF.z  =  msg.twist.twist.angular.z;


//COnvert to rpy
    tf::Quaternion q(quat_orientation.x, quat_orientation.y,
       quat_orientation.z, quat_orientation.w);
    tf::Matrix3x3 m(q);

    double drone_roll, drone_pitch, drone_yaw;
    m.getRPY(drone_roll, drone_pitch, drone_yaw);
    rpy_orientation.x = drone_roll;
    rpy_orientation.y = drone_pitch;
    rpy_orientation.z = drone_yaw;
    
     std_msgs::Header h = msg.header;
    if (flagQuadSimOdom == false)
    {
       dt = h.stamp.nsec - old_time_stamp;
       old_time_stamp = msg.header.stamp.nsec;

       dt = dt/ 1000000000.0;

       if (dt < 0)
       {
         dt = dt_safe;
         
       }
       
       dt_vec.push_back(dt);
       auto const dt_aver = average(dt_vec);
       
       //Clean the vector 
       if (dt_vec.size() > 50)
       {
         dt_vec.erase(dt_vec.begin());
       }
       
        if (dt > 0)
        {
          dt_safe = dt;
        }
        
        dt = dt_aver;
    }
    
    // angular position
    flagQuadSimOdom = true;
}




void interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback pose)
{
    marker_position.x = pose.pose.position.x;
    marker_position.y = pose.pose.position.y;
    marker_position.z = pose.pose.position.z;
    under_unity_manipulation = true;
   flagMarkerPosition = true;
}


void drone_under_unity_teleoperation_callback(const std_msgs::Bool flag)
{
  //This flag is true when the drone is under manipulation
  
  under_unity_manipulation = flag.data;
   
   flagDroneUnderManipulationFlag = true;

}

void interactive_marker_position_callback(const geometry_msgs::Point position)
{

    marker_unity_position = position;
    flagInteractiveMarkerPosition = true;
}



void Keyboard_input_callback(const std_msgs::Int32 msg)
{
   case_selected = msg.data;

   if (case_selected == 1 ||case_selected == 2 || case_selected == 3 )
   {
    //Clear all the vectors
   target_pos_x_GF_vec.clear();
   target_pos_y_GF_vec.clear();
   target_pos_z_GF_vec.clear(); 

   target_pos_x_GF_vec_rrt.clear();
   target_pos_y_GF_vec_rrt.clear();
   target_pos_z_GF_vec_rrt.clear();
   
    target_vel_x_GF_vec_rrt.clear();
   target_vel_y_GF_vec_rrt.clear();
   target_vel_z_GF_vec_rrt.clear();
  
   //Filtering the target velocities
   target_vel_x_GF_vec.clear(); 
   target_vel_y_GF_vec.clear(); 
   target_vel_z_GF_vec.clear();                                                                                                         
  
    //Average the target accelration 
   target_acc_x_GF_vec.clear(); 
   target_acc_y_GF_vec.clear(); 
   target_acc_z_GF_vec.clear();  

   gamma_vector.clear();                                                                                                       
  
   flagCaseSelectedKeyboard = true;

   }
   
}




void Keyboard_change_yaw_callback(const std_msgs::Int32 msg)
{
  //Change Yaw
  if (msg.data == 0)
  {
    //Increasing Yaw of 10 degree
    increasing_yaw = true;
    decreasing_yaw = false;
  }
  else
  {
    increasing_yaw = false;
    decreasing_yaw = true;
  }
}



void Keyboard_allow_take_off(const std_msgs::Int32 msg)
{
 
    //Increasing Yaw of 10 degree
    allow_take_off = true;
   
}


float evaluate_distance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
  float a = pow(point2.x - point1.x, 2);
  float b = pow(point2.y - point1.y, 2);
  float distance = 0.0;
  return distance = sqrt(a + b);
}


// Obtain rrt local segment
void rrt_local_path_callback(const scene_understanding_pkg_msgs::RRTPathPoints2D points)
{
  geometry_msgs::Point old_final_waypoint;
  if (rrt_path_points.size() > 1)
  {
    old_final_waypoint =rrt_path_points[rrt_path_points.size() -1];
  }
  
  rrt_path_points.clear();
  int counter = 1;
  float distance_new = 0.0;
  for (int ii = 0; ii < points.point.size(); ii++)
  {
    counter = ii - 1;
    // EValuate the distance between two consecutive poinsts
    if (counter <= points.point.size())
    {
      distance_new = evaluate_distance(points.point[ii], points.point[counter]); // points.point[counter]
      if (distance_new == 0)
      {
        continue;
      }
    }
    //rrt_path_points.push_back(points.point[ii]);

    if (ii == 0 )
    {
      geometry_msgs::Point point; 
      point.x = des_pos_GF.x;
      point.y = des_pos_GF.y;  
      point.z = position_GF.z;

       rrt_path_points.push_back(point);
    }
    else
    {
     rrt_path_points.push_back(points.point[ii]);
    }
    

  }
  
  flagRRTPath = true;
}





void rrt_start_assistive_guidance_callback(const std_msgs::Bool flag)
{
    rrt_start_assistive_guidance = flag.data;
    flagRRTStartAssistiveGuidance = true;
}

// Obtain the flag goal reached from RRT
void rrt_final_goal_reached_callback(const std_msgs::Bool flag)
{
    rrt_final_goal_reached = flag.data;
    flagRRTFinalGoal = true;
}



void rrt_virtual_obstacles_callback(const scene_understanding_pkg_msgs::RRTObstaclesCoo obstacles)
{
  //Take the number of obstacles
  virtual_obstacles_number = obstacles.number;

      for (int i = 0; i < obstacles.point.size(); i++)
     {
       //Take obstacles data
       virtual_obstacles_vec.push_back(obstacles.point[i]);

     }
 
  flagRRTVirtualObstacles = true;
   
}


void rrt_final_goal_position_callback(const geometry_msgs::Point final_goal_msg)
{
    final_goal = final_goal_msg;
    flagRRTfinalgoal = true;
}



void obstacle_force_res_callback(const scene_understanding_pkg_msgs::ObstacleRepForce obstacles)
{

  force_obs_res.x = obstacles.Fx;
  force_obs_res.y = obstacles.Fy;
  force_obs_Magn = obstacles.F_magn;
  
  
  flagObstacleForceResReceived = true;
}


//Publisher 
void publish_position_cmd(quadrotor_msgs::PositionCommand msg)
{
   msg.header.frame_id = frame_id; //"/mocap_sr"; //I comandi del drone devono essere rispetto al frame mocap o simulator per quetsion di sicurezza
   position_cmd_pub.publish(msg);
}


void publish_path2D_to_unity(geometry_msgs::Point point, geometry_msgs::Point point_old)
{
   //Publish the segmet that the drone is following to unity 
   scene_understanding_pkg_msgs::RRTPathPoints2D path;
   path.point.push_back(point_old);
   path.point.push_back(point);
   path2D_to_unity.publish(path);
}




void publish_exit_from_rrt_search_mode(bool flag)
{
  std_msgs::Bool flag_;
  flag_.data = flag;
  exit_from_rrt_search_mode.publish(flag_);


}

//Services Call
bool take_off()
{
//Chiama il servizio nel quale la posizione attuale GPS del drone viene fissata come quella di home
  bool success = false;
  std_srvs::Trigger trigger_takeoff;
  try 
  {
     take_off_srv.call(trigger_takeoff);
     success = true;
  }
  catch (const std::exception& e) 
  {
     cout << "Impossible to Take Off " << endl;
    success = false;
  }
  
  return success;
}



bool motors_on()
{
//Chiama il servizio nel quale la posizione attuale GPS del drone viene fissata come quella di home
  bool success = false;
  std_srvs::SetBool trigger_motors_on;
  try 
  {
     trigger_motors_on.request.data = true;
     motors_on_srv.call(trigger_motors_on);
     success = true;
  }
  catch (const std::exception& e) 
  {
     cout << "Impossible to enable motors " << endl;
    success = false;
  }
  
  return success;
}


bool goTo()
{
    bool success = false;
    mav_manager::Vec4 Pose;
  try 
  {
      Pose.request.goal = {1.0, 0.0, 1.0, 0.0};
     goTo_srv.call(Pose);
     success = true;
  }
  catch (const std::exception& e) 
  {
     cout <<"Impossible to Reach Position " << endl;
    success = false;
  }
  
  return success;
}


bool disable_tracker_manager()
{
    bool success = false;
    trackers_msgs::Transition mode;
  try 
  {
     mode.request.tracker = "std_trackers/NullTracker";
     disable_tracker.call(mode);
     success = true;
  }
  catch (const std::exception& e) 
  {
     cout << "Impossible to  Disable Tracker" << endl;
    success = false;
  }
  
  return success;

}


void publish_voxl_tf()
    {
        //TF Publisher valuse 
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
    


   
    
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap_sr";
    odom_trans.child_frame_id = "voxl3";

    odom_trans.transform.translation.x = position_GF.x;
    odom_trans.transform.translation.y = position_GF.y;
    odom_trans.transform.translation.z = position_GF.z;
    odom_trans.transform.rotation = quat_orientation;
  
    //send the transform
    voxl_odom_broadcaster.sendTransform(odom_trans);
    
    }


//Functions 

void average_marker_postions_on_multiple_stamps()
{

  // AVeraging Position along X axis 
  target_pos_x_GF_vec.push_back(des_user_pos_GF.x);
  des_user_pos_GF.x= average(target_pos_x_GF_vec);
  //Clean the vector 
  if (target_pos_x_GF_vec.size() > avg_stamps)
  {
    target_pos_x_GF_vec.erase(target_pos_x_GF_vec.begin());
  } 

  // AVeraging Position along Y axis 
  target_pos_y_GF_vec.push_back(des_user_pos_GF.y);
  des_user_pos_GF.y = average(target_pos_y_GF_vec);
  //Clean the vector 
  if (target_pos_y_GF_vec.size() > avg_stamps)
  {
    target_pos_y_GF_vec.erase(target_pos_y_GF_vec.begin());
  } 


}

void average_des_velocities_rrt()
{
  
 float  avg_stamps_ = 15;
  float vel_init = target_vel_GF.x;
  // AVeraging Position along X axis 
  target_vel_x_GF_vec_rrt.push_back(target_vel_GF.x);
  target_vel_GF.x= average(target_vel_x_GF_vec_rrt);

  
  //Clean the vector 
  if (target_pos_x_GF_vec.size() > avg_stamps_)
  {
    target_vel_x_GF_vec_rrt.erase(target_vel_x_GF_vec_rrt.begin());
  } 

  // AVeraging Position along Y axis 
  target_pos_y_GF_vec_rrt.push_back(target_vel_GF.y);
  target_vel_GF.y = average(target_vel_y_GF_vec_rrt);
  //Clean the vector 
  if (target_vel_y_GF_vec_rrt.size() > avg_stamps_)
  {
    target_vel_y_GF_vec_rrt.erase(target_vel_y_GF_vec_rrt.begin());
  } 


}


void average_des_positions_rrt()
{

  float  avg_stamps_ = 20;
 
  // AVeraging Position along X axis 
  target_pos_x_GF_vec_rrt.push_back(des_pos_GF.x);
  des_pos_GF.x= average(target_pos_x_GF_vec_rrt);

  
  //Clean the vector 
  if (target_pos_x_GF_vec_rrt.size() > avg_stamps_)
  {
    target_pos_x_GF_vec_rrt.erase(target_pos_x_GF_vec_rrt.begin());
  } 

  // AVeraging Position along Y axis 
  target_pos_y_GF_vec_rrt.push_back(des_pos_GF.y);
  des_pos_GF.y = average(target_pos_y_GF_vec_rrt);
  //Clean the vector 
  if (target_pos_y_GF_vec_rrt.size() > avg_stamps_)
  {
    target_pos_y_GF_vec_rrt.erase(target_pos_y_GF_vec_rrt.begin());
  } 


}


float average_angle_gamma(double gamma)
{
  float  avg_stamps_ = 80;
 
  // AVeraging Position along X axis 
  gamma_vector.push_back(gamma);
  float gamma_= average(gamma_vector);

  
  //Clean the vector 
  if (gamma_vector.size() > avg_stamps_)
  {
    gamma_vector.erase(gamma_vector.begin());
  } 

 return gamma_;
  
}

void average_obs_force()
{
  
  if (counter_f_filtered < max_counter_f_filt_value)
  {
     float  avg_stamps_ = 20;
 
  // AVeraging Position along X axis 
  fx_obs_vec.push_back(force_obs_res.x);
  force_obs_res.x= average(fx_obs_vec);

  
  //Clean the vector 
  if (fx_obs_vec.size() > avg_stamps_)
  {
    fx_obs_vec.erase(fx_obs_vec.begin());
  } 

  // AVeraging Position along Y axis 
  fy_obs_vec.push_back(force_obs_res.y);
  force_obs_res.y = average(fy_obs_vec);
  //Clean the vector 
  if (fy_obs_vec.size() > avg_stamps_)
  {
    fy_obs_vec.erase(fy_obs_vec.begin());
  } 
counter_f_filtered = counter_f_filtered +1;

  }
 

  
}


void check_des_yaw()
{
 
   if (increasing_yaw)
  {
    if (counter_yaw_rate == 0)
    {
      angle_rad_horizon = 10*M_PI/180;
      yaw_drone_initial =  rpy_orientation.z;
      if (yaw_drone_initial > 170*M_PI/180 )
      {
        //Case when the final angle is bigger that 180
         float step  = M_PI - (yaw_drone_initial + angle_rad_horizon);
         final_angle = - M_PI - step;
      }
      else
      {
        final_angle = yaw_drone_initial + angle_rad_horizon;
      }
     
    }
    
    des_yaw =  rpy_orientation.z + 0.1;
    
    if (rpy_orientation.z > final_angle)
    {
       increasing_yaw = false;
       counter_yaw_rate = 0;
       
    }
    else
    {
         counter_yaw_rate = counter_yaw_rate + 1;
    }
   
  }

  if (decreasing_yaw)
  {
    if (counter_yaw_rate == 0)
    {
      angle_rad_horizon = 10*M_PI/180;
      yaw_drone_initial =  rpy_orientation.z;
      if (yaw_drone_initial < -170*M_PI/180 )
      {
        //Case when the final angle is bigger that 180
         float step  = -M_PI - (yaw_drone_initial - angle_rad_horizon);
         final_angle =  M_PI - step;
      }
      else
      {
        final_angle = yaw_drone_initial - angle_rad_horizon;
      }
     
    }

    
   des_yaw =  rpy_orientation.z - 0.1;
    if (rpy_orientation.z < final_angle)
    {
       decreasing_yaw = false;
       counter_yaw_rate = 0;
    }
    else
    {
         counter_yaw_rate = counter_yaw_rate + 1;
    }
   
  }

  cout<< " FINAL ANGLE: " << final_angle << endl;
}

void publish_rrt_inside_bypass_manouver(bool flag)
{
  std_msgs::Bool flag_inside;
  flag_inside.data = flag;
  rrt_inside_bypass_manouver.publish(flag_inside);
}


void publishQuadrotorMarker(const std::string &frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = position_GF;
  marker.pose.orientation = quat_orientation;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.2;
  marker.color.a = 0.7;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.mesh_resource = mesh_directory;
  QuadrotorMarker.publish(marker);
}




void publishRRTFinalGoalMarker(const std::string &frame_id, bool publish)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = final_goal.x;
  marker.pose.position.y = final_goal.y;
  marker.pose.position.z = final_goal.z;
  
  marker.pose.orientation = quat_orientation;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.2;
  if (publish == true)
  {
    marker.color.a = 0.7;
  }
  else
  {
     marker.color.a = 0.0;
  }
  
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  //marker.mesh_resource = mesh_directory;
  final_goal_marker.publish(marker);
}


void publish_bypass_marker_to_unity(geometry_msgs::Point point)
{
    rrt_marker_for_manouver_in_unity.publish(point);
}




 void republish_interactive_marker_position_for_rosbags(geometry_msgs::Point marker_position)
 {
    interactive_marker_position.publish(marker_position);
 }


 void publish_tf_transform_from_world_to_sim()
 {
  ros::Time current_time_tf, last_time_tf;
  current_time_tf = ros::Time::now();
  last_time_tf = ros::Time::now();
  
  geometry_msgs::Quaternion quat;
  quat.w = 1.0; 

   geometry_msgs::Vector3 zero_pos;
 
   
  geometry_msgs::TransformStamped world_to_map;
  geometry_msgs::TransformStamped map_to_mocap;
  geometry_msgs::TransformStamped mocap_to_mocap_sr;
  geometry_msgs::TransformStamped mocap_sr_to_simulator;


  

  world_to_map.header.stamp = current_time_tf;
  map_to_mocap.header.stamp= current_time_tf;
  mocap_to_mocap_sr.header.stamp = current_time_tf;
  mocap_sr_to_simulator.header.stamp = current_time_tf;
  
  
  world_to_map.header.frame_id = "world";
  world_to_map.child_frame_id = "map"; //drone31/cam1_link
  map_to_mocap.header.frame_id = "map";
  map_to_mocap.child_frame_id = "mocap";
  mocap_to_mocap_sr.header.frame_id = "mocap";
  mocap_to_mocap_sr.child_frame_id = "mocap_sr";
  mocap_sr_to_simulator.header.frame_id = "mocap_sr";
  mocap_sr_to_simulator.child_frame_id = "simulator";
  
  world_to_map.transform.translation = zero_pos;
  map_to_mocap.transform.translation = zero_pos;
  mocap_to_mocap_sr.transform.translation = zero_pos;
  mocap_sr_to_simulator.transform.translation = zero_pos;



   //Translation from base link to camera link (rigidly attached)
  world_to_map.transform.rotation = quat;
  map_to_mocap.transform.rotation = quat;
  mocap_to_mocap_sr.transform.rotation = quat;
  mocap_sr_to_simulator.transform.rotation = quat;

  world_to_map_tf.sendTransform(world_to_map);
  map_to_mocap_tf.sendTransform(map_to_mocap);
  mocap_to_mocap_sr_tf.sendTransform(mocap_to_mocap_sr);
  mocap_sr_to_simulator_tf.sendTransform(mocap_sr_to_simulator);
 }



/*
//Functions 
void saturate_velocities()
{
     //Saturation on x axis 
    float des_x_vel = des_vel_msg.linear.x;
    float des_y_vel = des_vel_msg.linear.y;
    float des_z_vel = des_vel_msg.linear.z;


    if (des_x_vel > max_vel_x)
    {
        des_vel_msg.linear.x = max_vel_x;
    }
    if (des_x_vel < -1*max_vel_x)
    {
        des_vel_msg.linear.x = -1*max_vel_x;
    }

    if (des_y_vel > max_vel_y)
    {
        des_vel_msg.linear.y = max_vel_y;
    }
    if (des_y_vel < -1*max_vel_y)
    {
        des_vel_msg.linear.y = -1*max_vel_y;
    }

    if (des_z_vel > max_vel_z)
    {
        des_vel_msg.linear.z = max_vel_z;
    }
    if (des_z_vel < -1*max_vel_z)
    {
        des_vel_msg.linear.z = -1*max_vel_z;
    }

}

*/
};


#endif
