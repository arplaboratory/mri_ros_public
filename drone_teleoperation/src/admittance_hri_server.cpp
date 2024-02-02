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
#include "drone_teleoperation/admittance_hri_server.h"
// #include "drone_teleoperation/arpl_drone_voxl.h"
// #include "drone_teleoperation/Mission_voxl.h"
// #include "drone_teleoperation/Admittance_Controller.h"
// #include "drone_teleoperation/KF.h"



namespace hri_admittance {

    HRIControlServer::HRIControlServer(const ros::NodeHandle& nh, 
            const ros::NodeHandle& nh_private,
            string path)
            : nh_(nh),
            nh_private_(nh_private),
            kx(Eigen::Vector3f(12.4, 12.4, 10.4)),
            kv(Eigen::Vector3f(4.8, 4.8, 6.0)),
            scenario(1),
            modality(1),
            haptic_feedback_flag(false),
            pub_path_history_every_nsec(2),
            unity_odom_rate(10),
            utils(nh, nh_private),
            IC_core(nh, nh_private, path),
            visualizer(nh, nh_private)
            
            // horizon_force_perception(1.5),
            // lamda(1.0),
            // Fs(5.0),
            // avg_stamp(10),
            // mesh_publish_period(3)
            //ros_visualization(nh, nh_private)
            {//in sec
      //transformer_(nh, nh_private) { // transformer e una classe che va inizializzata e definita (se necessario)
    //Publisher
    
    odom_to_unity_pub = nh_.advertise<nav_msgs::Odometry>("voxl/odom", 1);
    position_cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>("pos_cmd", 1);
    end_planning_mode = nh_.advertise<std_msgs::Bool>("/planner/exit_search_mode", 10);
    interactive_marker_position = nh_.advertise<geometry_msgs::Point>("/interactive_marker_position", 10);
    force_perceived = nh_.advertise<geometry_msgs::WrenchStamped>("/ext_wrench", 1);
   
    //Subscribers
    quad_odom = nh_.subscribe("odom", 10, &HRIControlServer::quad_odom_callback, this);
    rviz_interactiveMarker = nh_.subscribe("/basic_controls/feedback", 1, &HRIControlServer::interactive_marker_feedback, this);
    haptic_odom = nh_.subscribe("haptic_command", 10, &HRIControlServer::haptic_command_callback, this);
    Keyboard_input_various = nh_.subscribe("/rqt_input/case_switcher", 1, &HRIControlServer::Keyboard_input_callback, this);
    Keyboard_input_change_yaw = nh_.subscribe("/keyboard_input/change_yaw", 1, &HRIControlServer::Keyboard_input_yaw_callback, this);
    visualize_goals = nh_.subscribe("/rqt_input/start_FPVI_task", 1, &HRIControlServer::visualize_goals_callback, this);
    rotate_yaw_180 = nh_.subscribe("/rqt_input/rotate_180_yaw", 1, &HRIControlServer::rotate_yaw_callback, this);
    // under_teleoperation_unity =  nh_.subscribe("/unity_to_ros/drone_under_teleope  ration", 1, &HRIControlServer::drone_under_unity_teleoperation_callback, this);
    interaction_marker_pose_from_unity = nh_.subscribe("/unity_to_ros/interactive_marker_position", 1, &HRIControlServer::unity_marker_position_callback, this);
    obtscles_force_res = nh_.subscribe("/obstacles_force_field", 1, &HRIControlServer::obstacle_force_res_callback, this);
    
    // Services
    disable_tracker = nh_.serviceClient<trackers_msgs::Transition> ("tracker_transition");
    hovering_srv = nh_.serviceClient<std_srvs::Trigger> ("hovering");
    // //Parameters 
    // nh_private_.getParam("/tele_control_params/desired_txt_folder_name", folder_name, folder_name ); //Need to go in the class thta manage the log files 
    // nh_private_.getParam("/tele_control_params/holo_voxl_test",holo_voxl_test, holo_voxl_test);
    nh_private_.param("/tele_control_params/time_to_pub_path_history", pub_path_history_every_nsec, pub_path_history_every_nsec);  //Timer Publisher
    nh_private_.param("/tele_control_params/odom_rate_to_unity", unity_odom_rate, unity_odom_rate);  //Timer Publish
    nh_private_.param("/tele_control_params/scenario", scenario, scenario);
    nh_private_.param("/tele_control_params/modality", modality, modality);
    nh_private_.param("/tele_control_params/gains/pos/x", kx(0), kx(0));
    nh_private_.param("/tele_control_params/gains/pos/y", kx(1),kx(1) );
    nh_private_.param("/tele_control_params/gains/pos/z", kx(2),kx(2) );
  
    nh_private_.param("/tele_control_params/gains/vel/x",  kv(0), kv(0));
    nh_private_.param("/tele_control_params/gains/vel/y",  kv(1), kv(1));
    nh_private_.param("/tele_control_params/gains/vel/z",  kv(2), kv(2));
    nh_private_.param("/tele_control_params/haptic_feedback_in_use", haptic_feedback_flag, haptic_feedback_flag);
    
    IC_core.haptic_device = haptic_feedback_flag;
   


//  Set a timing function to publish the 

    if (pub_path_history_every_nsec > 0) {
    publish_f_obs_history_timer =
        nh_private_.createTimer(ros::Duration(pub_path_history_every_nsec),
                                &HRIControlServer::publish_f_obs_history, this);
  }
    
 unity_odom_rate = 1.0/unity_odom_rate;
    if (unity_odom_rate > 0) {
    publish_odom_to_unity_timer =
        nh_private_.createTimer(ros::Duration(unity_odom_rate),
                                &HRIControlServer::publish_odom_to_unity, this);
  }

     //RVIZ Marker Initialization in Origin 
    marker_position = Eigen::Vector3f(0.0,0.0,0.0);
    haptic_marker_position= Eigen::Vector3f(0.0,0.0,0.0);
    force_obs_vec = Eigen::Vector3f(0.0,0.0,0.0);
   force_obs_Magn = 0.0;
    
  }

  void HRIControlServer::quad_odom_callback(nav_msgs::Odometry msg)
{
    //Obtain Position of quadrotor 
    
    quad_position = Eigen::Vector3f(msg.pose.pose.position.x, msg.pose.pose.position.y,  msg.pose.pose.position.z);
    quad_quat_orientation =  Eigen::Vector4f(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);


    //Obtain Velocities
    quad_lin_vel = Eigen::Vector3f(msg.twist.twist.linear.x,msg.twist.twist.linear.y,  msg.twist.twist.linear.z);
    quad_ang_vel = Eigen::Vector3f(msg.twist.twist.angular.x,msg.twist.twist.angular.y,  msg.twist.twist.angular.z);
  

//COnvert to rpy
    tf::Quaternion q(quad_quat_orientation(0), quad_quat_orientation(1),
       quad_quat_orientation(2), quad_quat_orientation(3));
    tf::Matrix3x3 m(q);

    double r, p, y;
    m.getRPY(r, p, y);
    quad_euler_orientation = Eigen::Vector3f((float)r,(float)p,(float)y);
    
    std_msgs::Header h = msg.header;
    double time_stamp = double(h.stamp.sec) +  double(h.stamp.nsec)*1e-9;
    dt = time_stamp - t_stamp_old;
    //cout << "dt: " << dt << endl;
    if (dt < 0.012 && init_timer_odom == false)
    {
      //Jump this odom value
      //cout << "Jump Odom Value:" << endl;
      return;
    }

    t_stamp_old = time_stamp;
    //Check if the user want to change the yaw deone
    //des_yaw = utils.check_des_yaw(increasing_yaw, decreasing_yaw, quad_euler_orientation(2), des_yaw);

 
      
    //Check the frame to assign depending if the system is running in simulation or not
    frame_id = utils.check_frames(scenario); //utilities 

    odom_to_unity = msg;
    /*
    *Define State Machine: 
    Every time a new odom is received the entire admittance pipeline is evaluated.
    When the transition button is pressed the null tracker is called and the robot transitioned to the admittacne control
    */

    switch (state)
    {
      case 0:
      // FPVI Mode, Robot in Free Teleoperation 
      IC_core.free_admittance_interaction(&init, scenario, force_obs_vec, force_obs_Magn,  p_distance, p_distance_dot, (float)dt);
      break;

      case 1:
      // APVI Mode, Robot in Assisted Teleoperation with an active planner 
      IC_core.assisted_admittance_interaction(&init, scenario, force_obs_vec, force_obs_Magn,   p_distance, p_distance_dot, (float)dt);
      break;
    }
    
    if (IC_core.hovering && counter < 10){
        hovering();
        ROS_WARN("ROBOT OUTSIDE SAFETY BOUNDARIES. HOVERING. RESTART SESSION");
        counter = counter + 1;
        return;
    }
    
    //if the success to the tracker transition is true, let's send the transition command 
   
    if (tracker_transition_success)
    {  
       send_position_command();
       cout << "Sending Position Command" << endl;
    }
   
    //Check when to change the state 
    cout << "IC_core.APVI: " << IC_core.APVI << endl;
    if (IC_core.APVI)
    {
      state = 1;
    }
    else
    {
      state = 0;
    }
    
    des_yaw =  IC_core.yaw_value; 
    //Transfer to Admittance core the quad information 
    IC_core.get_quad_pose(quad_position, quad_euler_orientation);
    IC_core.get_quad_vel(quad_lin_vel);
    IC_core.get_rviz_marker_pose(marker_position);
    IC_core.get_unity_marker_pose(marker_unity_position);
    IC_core.get_haptic_marker_pose(haptic_marker_position); 
    
    
    //Publish the path History 
    visualizer.publish_path_history(quad_position, (double)dt);
    visualizer.publish_QuadrotorMarker(quad_position, quad_quat_orientation);

    //Send force to the haptic device 
    send_perceived_force();

    //PRINT TELEMETRY
     cout << "X_GF: " << quad_position(0) << " Y_GF: " <<  quad_position(1) << " Z_GF: " << quad_position(2) << endl;
    cout << "X_vel_GF: " << quad_lin_vel(0) << " Y_vel_GF: " << quad_lin_vel(1) << " Z_vel_GF: " << quad_lin_vel(2) << endl;
    cout << "X Des: " << IC_core.pos_comm(0) << " Y Des: " << IC_core.pos_comm(1) << " Z Des: " << IC_core.pos_comm(2) << endl;
    cout << "Yaw Rad: " << IC_core.yaw_value << endl;
    cout << "F Obst X: " << force_obs_vec(0) << " F Obst Y: " << force_obs_vec(1) << endl;
    
    cout << "Hz: " << 1/dt << endl;
    cout << "########################################################" << endl;
    under_unity_manipulation = false;
    increasing_yaw = false;
    decreasing_yaw = false;
    init_timer_odom = false;
    //usleep(1000000); //0.01 seconds

}

void HRIControlServer::unity_marker_position_callback(const geometry_msgs::Point position)
{
    marker_unity_position = Eigen::Vector3f(position.x, position.y, position.z);
     under_unity_manipulation = true;
}

void HRIControlServer::interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback pose)
{
    marker_position = Eigen::Vector3f(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    under_unity_manipulation = true;
}

void HRIControlServer::haptic_command_callback(geometry_msgs::PoseStamped msg)
{
      haptic_marker_position = Eigen::Vector3f(2*msg.pose.position.x, 2*msg.pose.position.y, 0);
}


void  HRIControlServer::Keyboard_input_yaw_callback(const std_msgs::Int32 msg)
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

void HRIControlServer::visualize_goals_callback(const std_msgs::Bool msg)
{
   IC_core.visualize_goals = msg.data;
  
}

void HRIControlServer::rotate_yaw_callback(const std_msgs::Bool msg)
{
   //This function made possible for the user to change the yaw of the drone and to explore the surroundings
   if (IC_core.change_yaw_active == false)
   {
      IC_core.change_yaw180 = msg.data;
   }
    
}

void HRIControlServer::obstacle_force_res_callback(const scene_understanding_pkg_msgs::ObstacleRepForce obstacles)
{
  force_obs_vec = Eigen::Vector3f(obstacles.Fx, obstacles.Fy, 0.0);
  force_obs_Magn = obstacles.F_magn;
  //Penetration distance of the closer obstacle inside the robot body horizon. To be sent to the haptic
  p_distance = Eigen::Vector2f(obstacles.pd[0], obstacles.pd[1]);
  p_distance_dot = Eigen::Vector2f(obstacles.pd_dot[0], obstacles.pd_dot[1]);
  

}

void HRIControlServer::Keyboard_input_callback(const std_msgs::Int32 msg)
{
   int case_selected = msg.data;
   
   if (case_selected == 1)
   {
     //Robot enters in hovering, switching from linear velocity tracker to null tracker with the admittance control
     //Call The null trackers
     disable_tracker_manager();
     IC_core.des_z = quad_position(2);
   }
   if (case_selected == 2)
   {
    if (state == 0){
        IC_core.leave_FPVI = true;
        IC_core.visualize_goals = false;
    }
    else{
      IC_core.leave_APVI = true;
      IC_core.leave_APVI_counter = true; //only to update counter 
    }
     
     IC_core.des_z = quad_position(2);
    
   }
  
//    if (case_selected == 1 ||case_selected == 2 || case_selected == 3 )
//    {
//     //Clear all the vectors
//    target_pos_x_GF_vec.clear();
//    target_pos_y_GF_vec.clear();
//    target_pos_z_GF_vec.clear(); 

//    target_pos_x_GF_vec_rrt.clear();
//    target_pos_y_GF_vec_rrt.clear();
//    target_pos_z_GF_vec_rrt.clear();
   
//     target_vel_x_GF_vec_rrt.clear();
//    target_vel_y_GF_vec_rrt.clear();
//    target_vel_z_GF_vec_rrt.clear();
  
//    //Filtering the target velocities
//    target_vel_x_GF_vec.clear(); 
//    target_vel_y_GF_vec.clear(); 
//    target_vel_z_GF_vec.clear();                                                                                                         
  
//     //Average the target accelration 
//    target_acc_x_GF_vec.clear(); 
//    target_acc_y_GF_vec.clear(); 
//    target_acc_z_GF_vec.clear();  

//    gamma_vector.clear();                                                                                                       
  
//    flagCaseSelectedKeyboard = true;

//    }
   
}

void HRIControlServer::publish_f_obs_history(const ros::TimerEvent& /*event*/) 
{
    visualizer.publish_f_obs_history(quad_position, force_obs_vec, force_obs_Magn, pub_path_history_every_nsec, dt);
   
}


void HRIControlServer::publish_odom_to_unity(const ros::TimerEvent& /*event*/)
{
    //Publish odom to holololens or unity at a reduced rate than vicon odom
    
    odom_to_unity_pub.publish(odom_to_unity);
}




bool HRIControlServer::disable_tracker_manager()
{
    tracker_transition_success = false;
    trackers_msgs::Transition mode;
  try 
  {
     mode.request.tracker = "std_trackers/NullTracker";
     disable_tracker.call(mode);
     tracker_transition_success = true;
     IC_core.null_tracker_trasition_success = true; //safety check before starting the admittance 
  }
  catch (const std::exception& e) 
  {
     cout << "Impossible to  Disable Tracker" << endl;
    tracker_transition_success = false;

  }
  counter = 0;
  return tracker_transition_success;

}

bool HRIControlServer::hovering()
{
  bool success = false;
  std_srvs::Trigger trigger_hovering;
  try 
  {
     hovering_srv.call(trigger_hovering);
     success = true;
  }
  catch (const std::exception& e) 
  {
     cout << "Impossible to Trigger Hovering " << endl;
    success = false;
  }
  
  return success;
}


void HRIControlServer::send_position_command()
{


     //Keep Constant Position and Altitude
     // From take off publish directly on the position command topic
   des_lin_vel = Eigen::Vector3f(0.0, 0.0,0.0 );
   quadrotor_msgs::PositionCommand pos_cmd;
   des_position = Eigen::Vector3f(0.0, 0.0,0.2 );
   pos_cmd.position.x = IC_core.pos_comm(0);
   pos_cmd.position.y = IC_core.pos_comm(1);
   pos_cmd.position.z = IC_core.pos_comm(2);

   pos_cmd.velocity.x = IC_core.vel_comm(0);
   pos_cmd.velocity.y = IC_core.vel_comm(1);
   pos_cmd.velocity.z = IC_core.vel_comm(2);

   pos_cmd.kx[0] = kx[0];
   pos_cmd.kx[1] = kx[1];
   pos_cmd.kx[2] = kx[2];
   pos_cmd.kv[0] = kv[0];
   pos_cmd.kv[1] = kv[1];
   pos_cmd.kv[2] = kv[2];
   pos_cmd.yaw =  des_yaw;
   publish_position_cmd(pos_cmd);

   
}

void HRIControlServer::send_perceived_force()
{
  geometry_msgs::WrenchStamped msg;
  geometry_msgs::Vector3 force_v;
  geometry_msgs::Vector3 torque_v;
  force_v.x = 2.0*IC_core.force_global(0);
  force_v.y = 2.0*IC_core.force_global(1);
  force_v.z = IC_core.sum_d(0); //Send the distance value x
  torque_v.x = IC_core.sum_d(1); //Send the distance value y
  torque_v.y = IC_core.sum_ddot(0); //Send the derivative of the distance value  x
  torque_v.z = IC_core.sum_ddot(1);

  msg.wrench.force = force_v;
  msg.wrench.torque = torque_v;
  force_perceived.publish(msg);
}


// Publisher 
void HRIControlServer::publish_position_cmd(quadrotor_msgs::PositionCommand msg)
{
   msg.header.frame_id = frame_id; //"/mocap_sr"; //I comandi del drone devono essere rispetto al frame mocap o simulator per quetsion di sicurezza
   position_cmd_pub.publish(msg);
}

}