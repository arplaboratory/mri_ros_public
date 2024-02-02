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
#include "drone_teleoperation/arpl_drone_voxl.h"
#include "drone_teleoperation/Mission_voxl.h"
#include "drone_teleoperation/Admittance_Controller.h"
#include "drone_teleoperation/KF.h"


using namespace std;
using namespace std::chrono;




void motors_on(arpl_drone_voxl *drone, Mission *mission)
{
   //Disable Tracker Manager for position control 
   drone->disable_tracker_manager();
   if (drone->start_motors == true)
   {
      mission->arm_motors_success = drone->motors_on();
      drone->start_motors = false;
      cout << "Motors On Success" << endl;
   }
   drone->disable_tracker_flag = false;

}

void take_off(arpl_drone_voxl *drone, Mission *mission)
{ 
   if (mission->take_off == true)
    {
	      mission->take_off_success = drone->take_off();
        mission->take_off = false;
	cout << "Take Off Success" << endl;
    }

}


void compute_target_velocities_and_accelerations(arpl_drone_voxl *drone, bool init, int trajectory_case)
{
  
  float dt = 0.03;
  
drone->target_user_vel_GF.x = (drone->des_user_pos_GF.x  - drone->des_user_pos_GF_old.x)/drone->dt;
    drone->target_user_vel_GF.y = (drone->des_user_pos_GF.y  - drone->des_user_pos_GF_old.y)/drone->dt;
    drone->target_user_vel_GF.z = (drone->des_user_pos_GF.z  - drone->des_user_pos_GF_old.z)/drone->dt;
    
     drone->target_user_acc_GF.x =  (drone->target_user_vel_GF.x  - drone->target_user_vel_GF_old.x)/drone->dt;
    drone->target_user_acc_GF.y =  (drone->target_user_vel_GF.y  - drone->target_user_vel_GF_old.y)/drone->dt;
    drone->target_user_acc_GF.z =  (drone->target_user_vel_GF.z  - drone->target_user_vel_GF_old.z)/drone->dt;
 
 if (trajectory_case == 2)
  {

    drone->target_vel_GF.x = (drone->des_pos_GF.x  - drone->des_pos_GF_old.x)/drone->dt;
  drone->target_vel_GF.y =  (drone->des_pos_GF.y  - drone->des_pos_GF_old.y)/drone->dt;
  drone->target_vel_GF.z =  (drone->des_pos_GF.z  - drone->des_pos_GF_old.z)/drone->dt;
  
  drone->target_acc_GF.x = (drone->target_vel_GF.x  - drone->target_vel_GF_old.x)/drone->dt;
  drone->target_acc_GF.y = (drone->target_vel_GF.y  - drone->target_vel_GF_old.y)/drone->dt;
  drone->target_acc_GF.z = (drone->target_vel_GF.z  - drone->target_vel_GF_old.z)/drone->dt;
  }
  else
  {
    //This is to avoid a big jump in the position when swithcing from case 1 to case 2.
    //In this the des_pos, which is the position on the line in case 2, is evaluated also in the case of free admittance. 
    // in this way no jump is detected in position because it isi initialized alredy where te drone and rrt start the task
    drone->des_pos_GF.x  = drone->des_user_pos_GF.x;
    drone->des_pos_GF.y = drone->des_user_pos_GF.y;
    drone->des_pos_GF.z  = drone->des_user_pos_GF.z;

     drone->target_vel_GF.x = (drone->des_user_pos_GF.x  - drone->des_user_pos_GF_old.x)/drone->dt;
  drone->target_vel_GF.y =  (drone->des_user_pos_GF.y  - drone->des_user_pos_GF_old.y)/drone->dt;
  drone->target_vel_GF.z =  (drone->des_user_pos_GF.z  - drone->des_user_pos_GF_old.z)/drone->dt;
  
  drone->target_acc_GF.x = (drone->target_user_vel_GF.x  - drone->target_user_vel_GF_old.x)/drone->dt;
  drone->target_acc_GF.y = (drone->target_user_vel_GF.y  - drone->target_user_vel_GF_old.y)/drone->dt;
  drone->target_acc_GF.z = (drone->target_user_vel_GF.z  - drone->target_user_vel_GF_old.z)/drone->dt;

  }

 
  
  if (init == true)
  {
    drone->target_vel_GF.x = 0.0;
    drone->target_vel_GF.y = 0.0;
    drone->target_vel_GF.z = 0.0;
    
    drone->target_acc_GF.x = 0.0;
    drone->target_acc_GF.y = 0.0;
    drone->target_acc_GF.z = 0.0;

    drone->target_user_vel_GF.x = 0.0;
    drone->target_user_vel_GF.y = 0.0;
    drone->target_user_vel_GF.z = 0.0;
    
    drone->target_user_acc_GF.x = 0.0;
    drone->target_user_acc_GF.y = 0.0;
    drone->target_user_acc_GF.z = 0.0;

  }
  
 

  // if ( drone->target_acc_GF.x > drone->target_acc_GF_old.x + 1)
  // {
  //     drone->target_acc_GF.x = drone->target_acc_GF_old.x;
  // }

  // if ( drone->target_acc_GF.y > drone->target_acc_GF_old.y + 1)
  // {
  //     drone->target_acc_GF.y = drone->target_acc_GF_old.y;
  // }

  // if ( drone->target_acc_GF.x < drone->target_acc_GF_old.x - 1)
  // {
  //     drone->target_acc_GF.x = drone->target_acc_GF_old.x;
  // }
  // if ( drone->target_acc_GF.y < drone->target_acc_GF_old.y - 1)
  // {
  //     drone->target_acc_GF.y = drone->target_acc_GF_old.y;
  // }



  
}

void trajectory_generation(arpl_drone_voxl *drone, Mission *mission, bool init)
{
  
  if (mission-> trajectory_case == 0)
  {
       // Generate desired Trajectory
      drone->des_pos_GF.x = 1.5*sin(0.5*mission->trajectory_time_stamp); 
      drone->des_pos_GF.y = 0; //sin(counter); //cos(counter);
      drone->des_pos_GF.z = 1.0; 
  }

  else if (mission-> trajectory_case == 1)
  {
    if (drone->rviz_real_test == true)
    {
      // The desired trajectory is defined by the marker 
      drone->des_user_pos_GF.x = drone->marker_position.x;  //drone->marker_unity_position.x; // drone->marker_position.x
      drone->des_user_pos_GF.y = drone->marker_position.y; //drone->marker_unity_position.y; //drone->marker_position.y
      drone->des_user_pos_GF.z = 0.9; //drone->marker_position.z; //drone->marker_unity_position.z;
     
     

    }
    else
    {
      	// The desired trajectory is defined by the marker 
      drone->des_user_pos_GF.x = drone->marker_unity_position.x;  //drone->marker_unity_position.x; // drone->marker_position.x
      drone->des_user_pos_GF.y = drone->marker_unity_position.y; //drone->marker_unity_position.y; //drone->marker_position.y
      drone->des_user_pos_GF.z = 0.8; //drone->marker_position.z; //drone->marker_unity_position.z;
    }

      drone->average_marker_postions_on_multiple_stamps(); 
   

  }
  else if (mission -> trajectory_case == 2)
  {
    //Drone Follow a predefined trajectory toward a goal. 
    // The user interact with the drone as an external force
    //The drone is compliant with the force depending on the mass and damoing parameters
     
    drone->des_pos_GF.x = mission->drone_desired_position_along_line.x; //0.5*sin(0.5*mission->trajectory_time_stamp); 
    drone->des_pos_GF.y = mission->drone_desired_position_along_line.y; //sin(counter); //cos(counter);
    drone->des_pos_GF.z = 1.0; 

    if (drone->rviz_real_test == true)
    {
    //User desired position during interaction 
    drone->des_user_pos_GF.x = drone->marker_position.x; //drone->marker_unity_position.x; 
    drone->des_user_pos_GF.y = drone->marker_position.y; //drone->marker_unity_position.y; 
    drone->des_user_pos_GF.z = 0.8; 
    }
    else
    {
       //User desired position during interaction 
    drone->des_user_pos_GF.x = drone->marker_unity_position.x; //drone->marker_unity_position.x; 
    drone->des_user_pos_GF.y = drone->marker_unity_position.y; //drone->marker_unity_position.y; 
    drone->des_user_pos_GF.z = 0.8; 
    }
    
    drone->average_des_positions_rrt();
    //drone->average_des_velocities_rrt();
   
    
    drone->average_marker_postions_on_multiple_stamps(); //Permits to average the des_pos_GF (in this case the position son the line )
   

  }
  else
  {
	  //Come Back to Hovering 
    drone->old_pos_GF_for_hovering = drone->position_GF;
    drone->old_pos_GF_for_hovering.z = 0.7;
	  mission->state = 1; 
  }

  // Evaluate desired drone velocity to follow the trajectory 
  compute_target_velocities_and_accelerations(drone, init, mission -> trajectory_case);


  float dt = 0.03;
  mission->trajectory_time_stamp = mission->trajectory_time_stamp + drone->dt;
  //cout << " mission->trajectory_time_stamp : " <<  mission->trajectory_time_stamp << endl;
}



void initialize_KF(KF *kf_x, KF *kf_y, KF *kf_user_x, KF *kf_user_y)
{
  // Initialize filters 
    // Initialize KF
         Eigen::Matrix2d P;
         Eigen::Matrix2d R;
         double sigma_x1 = 1.0;
         double sigma_x2 = 1.0;
         // Init covariance Matrix
         P << sigma_x1, 0,
             0, sigma_x2;
         // Init Measurement uncertanti Matrix : Uncertanty on position obs and vel obs
         R << 0.2, 0,
             0, 0.6;
         kf_x->pass_to_class_initialization_matrices(P, R);
         kf_y->pass_to_class_initialization_matrices(P, R);

         kf_user_x->pass_to_class_initialization_matrices(P, R);
         kf_user_y->pass_to_class_initialization_matrices(P, R);

     
}



void evaluate_background_processes(arpl_drone_voxl *drone, Mission *mission, 
KF *kf_x, KF *kf_y, KF *kf_user_x, KF *kf_user_y, Admittance_Controller * IC_x, Admittance_Controller *IC_y)
{
  // Filtering Position and velocity of the desired trajectory autonomously generated
  kf_x->calculate(drone->des_pos_GF.x, drone->target_vel_GF.x, drone->target_acc_GF.x, drone->dt);
  kf_x->state_est.x = kf_x->Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
  kf_x->state_est.y = kf_x->Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
  //cout << "[AUTONOMOUS TR TRACKING] [X AXIS] KF States: X: " << kf_x->state_est.x << " Y: " << kf_x->state_est.y << endl;

  kf_y->calculate(drone->des_pos_GF.y, drone->target_vel_GF.y, drone->target_acc_GF.y, drone->dt); // drone->dt);
  kf_y->state_est.x = kf_y->Obtain_KF_estimated_state()[0];                                        // Pos Filtered on X axis
  kf_y->state_est.y = kf_y->Obtain_KF_estimated_state()[1];  
  
 

  //Filtering X axis Position and Velocity

  
  kf_user_x->calculate(drone->des_user_pos_GF.x,drone->target_user_vel_GF.x, drone->target_user_acc_GF.x, drone->dt);
  kf_user_x->state_est.x = kf_user_x->Obtain_KF_estimated_state()[0]; //Pos Filtered on X axis
  kf_user_x->state_est.y = kf_user_x->Obtain_KF_estimated_state()[1]; //Vel Filtered on X axis
  cout << "[TR TRACKING] [X AXIS] KF States: X: " << kf_user_x->state_est.x << " Y: " <<  kf_user_x->state_est.y << endl;
  

  kf_user_y->calculate(drone->des_user_pos_GF.y, drone->target_user_vel_GF.y, drone->target_user_acc_GF.y,  drone->dt); //drone->dt);
  kf_user_y->state_est.x = kf_user_y->Obtain_KF_estimated_state()[0]; //Pos Filtered on X axis
  kf_user_y->state_est.y = kf_user_y->Obtain_KF_estimated_state()[1]; //Vel Filtered on X axis
  cout << "[TR TRACKING] [Y AXIS] KF States: X: " << kf_user_y->state_est.x << " Y: " <<  kf_user_y->state_est.y << endl;
  

  IC_x->calculate(kf_x->state_est.x, drone->position_GF.x,   kf_x->state_est.y , drone->velocity_GF.x, drone->force_obs_res.x, drone->dt); 
  IC_y->calculate(kf_y->state_est.x, drone->position_GF.y,  kf_y->state_est.y, drone->velocity_GF.y, drone->force_obs_res.y, drone->dt); 
  

   drone->comm_pos_GF.x = IC_x->obtain_commanded_position();
    drone->comm_vel_GF.x = IC_x->obtain_commanded_velocity();

    drone->comm_pos_GF.y = IC_y->obtain_commanded_position();
    drone->comm_vel_GF.y = IC_y->obtain_commanded_velocity();



}

bool init = true;
void hovering(arpl_drone_voxl *drone, Mission *mission, std::chrono::duration<double,std::milli> elapesed_time,  
KF *kf_x, KF *kf_y, KF *kf_user_x, KF *kf_user_y, Admittance_Controller * IC_x, Admittance_Controller *IC_y)
{
   
   
  
  //Check if the user want to cjange the yaw deone
  drone->check_des_yaw();

     //Keep Constant Position and Altitude
     // From take off publish directly on the position command topic
   drone->target_vel_GF.x = 0.0;
   drone->target_vel_GF.y = 0.0;
   drone->target_vel_GF.z = 0.0;
   quadrotor_msgs::PositionCommand pos_cmd;
   drone->des_pos_GF = drone->old_pos_GF_for_hovering;
   pos_cmd.position = drone->des_pos_GF;
   pos_cmd.velocity = drone->target_vel_GF;
   pos_cmd.kx[0] = drone->kx[0];
   pos_cmd.kx[1] = drone->kx[1];
   pos_cmd.kx[2] = drone->kx[2];
   pos_cmd.kv[0] = drone->kv[0];
   pos_cmd.kv[1] = drone->kv[1];
   pos_cmd.kv[2] = drone->kv[2];
   pos_cmd.yaw =  drone->des_yaw;
   drone->publish_position_cmd(pos_cmd);
   
   
  
  
   //Define the tike when to switch to the next state and start to follow the trajectory
   auto publish_period = 3000.0;
   //cout << "elapesed_time.count(): " << elapesed_time.count() << endl;
   //if (elapesed_time.count() > publish_period && mission->initial_hovering == true)

  //  if (drone->rviz_real_test == true) //sono nek caso solo simulazione o rviz in reale 
  //  {
  //    drone->under_unity_manipulation = true;
  //  }
   
   float distance_marker_drone = sqrt(pow( drone->des_user_pos_GF.x - drone->position_GF.x, 2) + pow( drone->des_user_pos_GF.y - drone->position_GF.y, 2));
  
      //Switch to state follow trajectory
    
      if (drone->under_unity_manipulation == true  && drone->rrt_start_assistive_guidance == false && distance_marker_drone < 0.10 || drone->case_selected == 2 || drone->init_test == true)
     {
         // Switch to state 2 related to the drone teleoperation
          init = false;
          drone->init_test = false;
         if (mission->previous_state == 0 )
         {
            initialize_KF(kf_x, kf_y, kf_user_x, kf_user_y);
             init = true;
         }
        
        
         mission->initial_hovering = false;
         mission->state = 2; //Manual guidance without rrt*
         mission->previous_state = 1;
         mission->trajectory_case = 1;
         
         drone->comm_pos_GF = drone->position_GF;
         drone->comm_vel_GF = drone->target_vel_GF;
         drone->case_selected = 0;
         drone->publish_rrt_final_goal_marker_flag = false;
         drone->rrt_counter_case = 0;
     }
     else if (drone->rrt_start_assistive_guidance == true && drone->rrt_path_points.size() > 0 &&  drone->rrt_exit_assistive_guidance == false)
     {

         if (mission->previous_state == 0 )
         {
            initialize_KF(kf_x, kf_y, kf_user_x, kf_user_y);
         }
         mission->initial_hovering = false;
         //init = false;
         mission->state = 3; //Switch to rrt* guidance oif some points are published 
         mission->previous_state = 1;
         mission->trajectory_case = 2;
          drone->publish_rrt_final_goal_marker_flag = true;
          drone->rrt_path_points.clear();
        mission->setpoint_x_vec.clear();
        mission->setpoint_y_vec.clear();
          init = true;
          mission->drone_desired_position_along_line.x = drone->position_GF.x;
           mission->drone_desired_position_along_line.y = drone->position_GF.y;
           drone->rrt_counter_case = 0;

     }
     else
     {
       mission->state = 1;
     }

   
    if (mission->previous_state == 3)
    {
      evaluate_background_processes(drone, mission,kf_x , kf_y, kf_user_x,kf_user_y, IC_x, IC_y);
    }


 cout << "[HOVERING STATE] " << endl;
}


void trajectory_following(arpl_drone_voxl *drone, Mission *mission, 
Admittance_Controller * IC_x, Admittance_Controller *IC_y, Admittance_Controller *IC_z, KF *kf_x, KF *kf_y, KF *kf_user_x, KF *kf_user_y)
{

   //Check if the yaw must be changed from user inputs 
drone->check_des_yaw();


	//Start To follow the desired trajectory
  trajectory_generation(drone, mission, init);

  geometry_msgs::Point init_comm_pos;
  geometry_msgs::Vector3 init_comm_vel;
  
  init_comm_pos.z = 0.8;
  // drone->comm_pos_GF = init_comm_pos;
  // drone->comm_vel_GF = init_comm_vel;
  
  // Filtering the position and velocity using KF
  double dt = 0.03;


  // Filtering Position and velocity of the desired trajectory autonomously generated
 
  kf_x->calculate(drone->des_pos_GF.x, drone->target_vel_GF.x, drone->target_acc_GF.x, drone->dt);
  kf_x->state_est.x = kf_x->Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
  kf_x->state_est.y = kf_x->Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
  cout << "[AUTONOMOUS TR TRACKING] [X AXIS] KF States: X: " << kf_x->state_est.x << " Y: " << kf_x->state_est.y << endl;
  
  kf_y->calculate(drone->des_pos_GF.y, drone->target_vel_GF.y, drone->target_acc_GF.y, drone->dt); // drone->dt);
  kf_y->state_est.x = kf_y->Obtain_KF_estimated_state()[0];                                        // Pos Filtered on X axis
  kf_y->state_est.y = kf_y->Obtain_KF_estimated_state()[1];  
  cout << "[AUTONOMOUS TR TRACKING] [Y AXIS] KF States: X: " << kf_y->state_est.x << " Y: " << kf_y->state_est.y << endl;
 

  //Filtering X axis Position and Velocity

  
  kf_user_x->calculate(drone->des_user_pos_GF.x,drone->target_user_vel_GF.x, drone->target_user_acc_GF.x, drone->dt);
  kf_user_x->state_est.x = kf_user_x->Obtain_KF_estimated_state()[0]; //Pos Filtered on X axis
  kf_user_x->state_est.y = kf_user_x->Obtain_KF_estimated_state()[1]; //Vel Filtered on X axis
  cout << "[TR TRACKING] [X AXIS] KF States: X: " << kf_user_x->state_est.x << " Y: " <<  kf_user_x->state_est.y << endl;
  

  kf_user_y->calculate(drone->des_user_pos_GF.y, drone->target_user_vel_GF.y, drone->target_user_acc_GF.y,  drone->dt); //drone->dt);
  kf_user_y->state_est.x = kf_user_y->Obtain_KF_estimated_state()[0]; //Pos Filtered on X axis
  kf_user_y->state_est.y = kf_user_y->Obtain_KF_estimated_state()[1]; //Vel Filtered on X axis
  cout << "[TR TRACKING] [Y AXIS] KF States: X: " << kf_user_y->state_est.x << " Y: " <<  kf_user_y->state_est.y << endl;

  //if mission previous state is the rrt, filtering the obstacle force to avoid big step in the admittance 
  if ( mission->previous_state == 3)
  {
    drone->average_obs_force();
  }

 IC_x->calculate(kf_x->state_est.x, drone->position_GF.x,   kf_x->state_est.y , drone->velocity_GF.x, drone->force_obs_res.x, drone->dt); 
 IC_y->calculate(kf_y->state_est.x, drone->position_GF.y,  kf_y->state_est.y, drone->velocity_GF.y, drone->force_obs_res.y, drone->dt); 
  
  // kf_x->state_est.x = 0.0;
  // kf_y->state_est.x = 0.0;
  // drone->d_output = 10;
  // IC_x->calculate_xc_from_planned_trajectory(kf_user_x->state_est.x, kf_x->state_est.x, drone->position_GF.x, kf_user_x->state_est.y, kf_x->state_est.y, drone->velocity_GF.x, (double)drone->d_output, drone->dt);
  // IC_y->calculate_xc_from_planned_trajectory(kf_user_y->state_est.x, kf_y->state_est.x, drone->position_GF.y, kf_user_y->state_est.y, kf_y->state_est.y, drone->velocity_GF.y, (double)drone->d_output, drone->dt);

  
  mission->F_x = IC_x->obtain_evaluated_force();
  mission->F_y = IC_y->obtain_evaluated_force();
  

  if (init == false && drone->rrt_counter_case  > 50)
  {
    drone->comm_pos_GF.x = IC_x->obtain_commanded_position();
    drone->comm_vel_GF.x = IC_x->obtain_commanded_velocity();

    drone->comm_pos_GF.y = IC_y->obtain_commanded_position();
    drone->comm_vel_GF.y = IC_y->obtain_commanded_velocity();

     drone->comm_pos_GF.z = 0.8;
     drone->comm_vel_GF.z = 0.0;
  }

  //When the unity_flag_drone_under manipulation become false 
  //The user release the interactive hologram marker, 
  // Implement here a safety check based on distance and time before come 
  // back to hovering staqte 
  
  //Evakluate the distanc ebetween the drone and the tarcking target when is not more under 
  // manipulation 
  float x_err_squared = pow(drone->marker_unity_position.x - drone->position_GF.x, 2);
  float y_err_squared = pow(drone->marker_unity_position.y - drone->position_GF.y, 2);
  float z_err_squared = pow(drone->marker_unity_position.z - drone->position_GF.z, 2);

  float distance_3D = sqrt(x_err_squared + y_err_squared + z_err_squared);

  // if (distance_3D < 0.05 && drone->under_unity_manipulation == false)
  // {
  //   //Switch to Hoverimng state to avoid 
  //   // not good behaviours 
  //   //drone->des_pos_GF = drone->old_pos_GF_for_hovering;
  //   drone->old_pos_GF_for_hovering =  drone->position_GF;
  //   init = true;
  //   mission->state = 1;
  // }
   
   
  //If the falg becomes true switch to rrt assistive guidance 
  if (drone->rrt_start_assistive_guidance == true  &&  drone->rrt_path_points.size() > 0)
  {
    drone->fx_obs_vec.clear();
    drone->fy_obs_vec.clear();
     drone->counter_f_filtered = 0;
    mission->state = 3;
    mission->previous_state = 2;
    mission->trajectory_case = 2;
    mission->init_new_des_projected_pose = false;
    drone->rrt_counter_case  = 0;
  }
 
  // if (drone->rviz_real_test == true) //sono nek caso solo simulazione o rviz in reale 
  //  {
  //    drone->under_unity_manipulation = true;
  //  }

   if (drone->case_selected == 1) // ||  drone->under_unity_manipulation == false)
   {
    
     mission->setpoint_x_vec.clear();
     mission->setpoint_y_vec.clear();
     mission->state = 1;
     mission-> trajectory_case = 1;
     mission->init_new_des_projected_pose = false;
     drone->old_pos_GF_for_hovering =  drone->position_GF;
     init = true;
    drone->case_selected = 0;
    drone->rrt_counter_case = 0;
    mission->previous_state = 2;
   }

  
      //Keep Constant Position and Altitude
   quadrotor_msgs::PositionCommand pos_cmd;
   
   pos_cmd.position = drone->comm_pos_GF;
   pos_cmd.velocity = drone->comm_vel_GF;
   pos_cmd.kx[0] = drone->kx[0];
   pos_cmd.kx[1] = drone->kx[1];
   pos_cmd.kx[2] = drone->kx[2];
   pos_cmd.kv[0] = drone->kv[0];
   pos_cmd.kv[1] = drone->kv[1];
   pos_cmd.kv[2] = drone->kv[2];
   pos_cmd.yaw =  drone->des_yaw;
   drone->publish_position_cmd(pos_cmd);

 init = false;
 cout <<"[FREE ADMITTANCE TELEOPERATION STATE]" << endl;
 drone->rrt_counter_case = drone->rrt_counter_case + 1;
}



void follow_des_trajectory_under_user_interactions_and_perturbations(arpl_drone_voxl *drone, Mission *mission,
                                                                     Admittance_Controller *IC_x, Admittance_Controller *IC_y, Admittance_Controller *IC_z, KF *kf_x, KF *kf_y, KF *kf_user_x, KF *kf_user_y)
{
  // Check if the yaw must be changed from user inputs
  drone->check_des_yaw();

  if (init == false && drone->rrt_path_points.size() > 0 )
  {


    // Evaluate drone distance from the current setpoint
    mission->evaluate_drone_distance_from_the_current_setpoint(drone);

    // Evaluate line equation between the drone and the goal position and
    // the point line distance between the commanded position and the line
    float point_line_distance = mission->evaluate_point_line_distance(drone);
    // cout << "Point Line Distance: " << point_line_distance << endl;

    // FInd the distance between the drone position and the projection of the commanded position XC on the drone-goal line
    float drone_des_pos_proj_distance = mission->evaluate_drone_des_pos_proj_distance(drone, point_line_distance);
    // cout << "Drone des pos projection distance: " << drone_des_pos_proj_distance << endl;

    // mission->evaluate_intersection_point_P(drone);
    // Start To follow the desired trajectory

    trajectory_generation(drone, mission, init);
  }
  else
  {
    drone->des_pos_GF.x = 0.0;
    drone->des_pos_GF.y = 0.0;
    drone->target_vel_GF.x = 0.0;
    drone->target_vel_GF.y = 0.0;
    drone->target_acc_GF.x = 0.0;
    drone->target_acc_GF.y = 0.0;
    mission->previous_state = 3;
    mission->state = 1;
  }

  // geometry_msgs::Point init_comm_pos;
  // geometry_msgs::Vector3 init_comm_vel;

  // drone->comm_pos_GF = init_comm_pos;
  // drone->comm_vel_GF = init_comm_vel;

  // Filtering the position and velocity using KF
  double dt = 0.03;

  // Filtering Position and velocity of the desired trajectory autonomously generated
  kf_x->calculate(drone->des_pos_GF.x, drone->target_vel_GF.x, drone->target_acc_GF.x, drone->dt);
  kf_x->state_est.x = kf_x->Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
  kf_x->state_est.y = kf_x->Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
  //cout << "[AUTONOMOUS TR TRACKING] [X AXIS] KF States: X: " << kf_x->state_est.x << " Y: " << kf_x->state_est.y << endl;

  kf_y->calculate(drone->des_pos_GF.y, drone->target_vel_GF.y, drone->target_acc_GF.y, drone->dt); // drone->dt);
  kf_y->state_est.x = kf_y->Obtain_KF_estimated_state()[0];                                        // Pos Filtered on X axis
  kf_y->state_est.y = kf_y->Obtain_KF_estimated_state()[1];                                        // Vel Filtered on X axis
 // cout << "[AUTONOMOUS TR TRACKING] [Y AXIS] KF States: X: " << kf_y->state_est.x << " Y: " << kf_y->state_est.y << endl;

  // Filtering user desired trajectory
  kf_user_x->calculate(drone->des_user_pos_GF.x, drone->target_user_vel_GF.x, drone->target_user_acc_GF.x, drone->dt);
  kf_user_x->state_est.x = kf_user_x->Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
  kf_user_x->state_est.y = kf_user_x->Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
  //cout << "[USER TR TRACKING] [X AXIS] KF States: X: " << kf_user_x->state_est.x << " Y: " << kf_user_x->state_est.y << endl;

  kf_user_y->calculate(drone->des_user_pos_GF.y, drone->target_user_vel_GF.y, drone->target_user_acc_GF.y, drone->dt); // drone->dt);
  kf_user_y->state_est.x = kf_user_y->Obtain_KF_estimated_state()[0];                                                  // Pos Filtered on X axis
  kf_user_y->state_est.y = kf_user_y->Obtain_KF_estimated_state()[1];                                                  // Vel Filtered on X axis
  //cout << "[USER TR TRACKING] [Y AXIS] KF States: X: " << kf_user_y->state_est.x << " Y: " << kf_user_y->state_est.y << endl;

  mission->evaluate_variable_damping_coefficient(drone);
  mission->evaluate_variable_damping_coefficient(drone);
  // drone->force_obs_res.x = 0;
  // drone->force_obs_res.y = 0;
  IC_x->calculate_xc_from_planned_trajectory(kf_user_x->state_est.x, kf_x->state_est.x, drone->position_GF.x, kf_user_x->state_est.y, kf_x->state_est.y, drone->velocity_GF.x, drone->force_obs_res.x, (double)drone->d_output, drone->dt);
  IC_y->calculate_xc_from_planned_trajectory(kf_user_y->state_est.x, kf_y->state_est.x, drone->position_GF.y, kf_user_y->state_est.y, kf_y->state_est.y, drone->velocity_GF.y, drone->force_obs_res.y, (double)drone->d_output, drone->dt);

  mission->F_x = IC_x->obtain_evaluated_force();
  mission->F_y = IC_y->obtain_evaluated_force();

  if (init == false &&   drone->rrt_counter_case > 50)
  {
    drone->comm_pos_GF.x = IC_x->obtain_commanded_position();
    drone->comm_vel_GF.x = IC_x->obtain_commanded_velocity();

    drone->comm_pos_GF.y = IC_y->obtain_commanded_position();
    drone->comm_vel_GF.y = IC_y->obtain_commanded_velocity();

    drone->comm_pos_GF.z = 0.8;
    drone->comm_vel_GF.z = 0.0;
  }

  geometry_msgs::Point zero_pos;
  zero_pos.x = 0.5;
  zero_pos.y = 0.5;
  zero_pos.z = 1.0;
  geometry_msgs::Vector3 zero_vel;
  // Keep Constant Position and Altitude
  quadrotor_msgs::PositionCommand pos_cmd;
  pos_cmd.position = drone->comm_pos_GF;
  pos_cmd.velocity = drone->comm_vel_GF;
  pos_cmd.kx[0] = drone->kx[0];
  pos_cmd.kx[1] = drone->kx[1];
  pos_cmd.kx[2] = drone->kx[2];
  pos_cmd.kv[0] = drone->kv[0];
  pos_cmd.kv[1] = drone->kv[1];
  pos_cmd.kv[2] = drone->kv[2];
  pos_cmd.yaw = drone->des_yaw;
  drone->publish_position_cmd(pos_cmd);
  init = false;
  
  // Switch case to Manual user operation whe the rrt* final goal is reached by the drone
  
  if (drone->rrt_final_goal_reached == true && drone->under_unity_manipulation == false && mission->counter_unity_manipulation_flag_is_flase > 10)
  {
    // Going to Manual contro
    drone->rrt_path_points.clear();
    mission->setpoint_x_vec.clear();
    mission->setpoint_y_vec.clear();
    mission->state = 2;
    mission->trajectory_case = 1;
    mission->init_new_des_projected_pose = false;
    init = false;
    drone->case_selected = 0;
    // publish the flag to the rrt to stop the search
    drone->rrt_start_assistive_guidance = false;
    drone->publish_rrt_final_goal_marker_flag = false;

    
  
    drone->under_unity_manipulation = false; 
    drone->old_pos_GF_for_hovering = drone->des_user_pos_GF;
    mission->previous_state = 3;
    drone->gamma_vector.clear();
    
    mission->initial_hovering = false;
        //  mission->previous_state = 1;

         
    drone->comm_pos_GF = drone->position_GF;
    drone->comm_vel_GF = drone->target_vel_GF;
    drone->case_selected = 0;
    drone->rrt_counter_case = 0;
  }
  
 
  // if (drone->rviz_real_test == true) //sono nek caso solo simulazione o rviz in reale 
  //  {
  //    drone->under_unity_manipulation = true;
  //  }
   

  if (drone->case_selected == 1) // || drone->under_unity_manipulation == false) 
  {
    // Going to Hovering case
    drone->rrt_path_points.clear();
    mission->setpoint_x_vec.clear();
    mission->setpoint_y_vec.clear();
    mission->state = 1;
    mission->trajectory_case = 1;
    mission->init_new_des_projected_pose = false;
    init = true;
    drone->case_selected = 0;
   // drone->rrt_exit_assistive_guidance = true;
   // drone->rrt_start_assistive_guidance = false;
    drone->old_pos_GF_for_hovering = drone->position_GF;
    drone->publish_rrt_final_goal_marker_flag = false;
    drone->rrt_counter_case = 60;
     mission->previous_state = 3;

  }

  

  //To send the signal exit to rrt if the user press the keyboard key
  if (drone->case_selected == 2 ||drone->case_selected == 1)
  {
    drone->rrt_exit_assistive_guidance  = true;
    drone->rrt_start_assistive_guidance = false;
    drone->rrt_counter_case = 60;
    init = false;
     mission->previous_state = 3;
   
  }

  //mission->init_new_des_projected_pose = false;
  drone->publish_rrt_final_goal_marker_flag = true;

  cout <<"[RRT ADMITTANCE TELEOPERATION STATE]" << endl;
  drone->rrt_counter_case = drone->rrt_counter_case + 1;
}

void switch_case_from_user_input(arpl_drone_voxl *drone, Mission *mission)
{
    //Case Switcher if selected by user 
if (drone->flagCaseSelectedKeyboard == true)
{
  if (drone->case_selected == 1)
  {
    drone->old_pos_GF_for_hovering = drone->position_GF;
    drone->old_pos_GF_for_hovering.z = 0.9;
    mission->state = drone->case_selected;
  }
  if (drone->case_selected == 2)
  {
      mission->trajectory_case = 0;
      mission->state = 2;
  }

  if (drone->case_selected == 3)
  {
      mission->trajectory_case = 1;
       mission->state = 2;
  }

}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_control_voxl");
  ros::NodeHandle nh;
 
 
  bool sim_mode = false;
  bool holo_voxl_test = false;
  nh.getParam("/tele_control_params/in_simulation", sim_mode);
 
  
  //Class Drone 
  arpl_drone_voxl drone = arpl_drone_voxl(sim_mode);
  Mission mission;
  KF kf_x; 
  KF kf_y;

  
  //User KF in second interaction case 
  KF kf_user_x;
  KF kf_user_y;


  string folder_name;
  nh.getParam("/tele_control_params/desired_txt_folder_name", folder_name );
  nh.getParam("/tele_control_params/trajectory_method", mission.trajectory_case);
  nh.getParam("/tele_control_params/n_average_stamps", drone.avg_stamps);
  nh.getParam("/tele_control_params/holo_voxl_test",holo_voxl_test);
  nh.getParam("/tele_control_params/visualize_force_history", mission.view_history);
  nh.getParam("/tele_control_params/time_to_pub_path_history", mission.time_to_pub);
  nh.getParam("/tele_control_params/clear_path_history_after_sec", mission.time_to_clear_old_hist);

 

 
 


  //Load Params 
  nh.param("/tele_control_params/gains/pos/x", drone.kx[0], 0.0f);
  nh.param("/tele_control_params/gains/pos/y", drone.kx[1], 0.0f);
  nh.param("/tele_control_params/gains/pos/z", drone.kx[2], 0.0f);

  nh.param("/tele_control_params/gains/vel/x", drone.kv[0], 0.0f);
  nh.param("/tele_control_params/gains/vel/y", drone.kv[1], 0.0f);
  nh.param("/tele_control_params/gains/vel/z", drone.kv[2], 0.0f);
  
  double dt_ = 0.02;
  double Ks_x, Ks_y, Ks_z = 0.5;
  double Kd_x, Kd_y, Kd_z = -0.3;
  double F_max = 10;
  double F_min = -10;
  double M = 1.7;
  double D_x, D_y, D_z = 10;
  double K_x, K_y, K_z = 2;
  double D_MIN = 1.0;
  double D_MAX = 30;
  
 

  
   // Admittance_Controller Filter Parameters  selected by user 

   nh.getParam("/tele_control_params/Ks_x", Ks_x );
   nh.getParam("/tele_control_params/Kd_X", Kd_x );
   nh.getParam("/tele_control_params/Ks_y", Ks_y );
   nh.getParam("/tele_control_params/Kd_Y", Kd_y );
   nh.getParam("/tele_control_params/Ks_z", Ks_z );
   nh.getParam("/tele_control_params/Kd_Z", Kd_z );

   nh.getParam("/tele_control_params/M", M );
   nh.getParam("/tele_control_params/D_x", D_x);
   nh.getParam("/tele_control_params/K_x", K_x);
  
   nh.getParam("/tele_control_params/D_y", D_y);
   nh.getParam("/tele_control_params/K_y", K_y);
   nh.getParam("/tele_control_params/K_y", K_y);
   nh.getParam("/tele_control_params/K_y", K_y);

   nh.getParam("/tele_control_params/D_z", D_z);
   nh.getParam("/tele_control_params/K_z", K_z);

   nh.getParam("/tele_control_params/D_MIN", mission.D_MIN);
   nh.getParam("/tele_control_params/D_MAX", mission.D_MAX);
   nh.getParam("/tele_control_params/damping_function", mission.damping_function);
  
   
   

  Admittance_Controller IC_x = Admittance_Controller(Ks_x, Kd_x, F_max, F_min, M, D_x, K_x);
  Admittance_Controller IC_y = Admittance_Controller(Ks_y, Kd_y, F_max, F_min, M, D_y, K_y);
  Admittance_Controller IC_z = Admittance_Controller(Ks_z, Kd_z, F_max, F_min, M, D_z, K_z);

  
  //Load Mesh visualization 
  drone.mesh_directory = "package://mesh_visualization/mesh/hummingbird.mesh";
  //Pass the variable to drone 
  drone.sim_mode = sim_mode; 
  drone.holo_voxl_test = holo_voxl_test; 

  if (drone.sim_mode == true && drone.holo_voxl_test == false)
  {
      drone.rviz_real_test  = true; //prendo input da interactive marker rviz
      drone.holo_voxl_test = false;
  }
  else if (drone.sim_mode == false && drone.holo_voxl_test == false)
  {
     drone.rviz_real_test  = true;
  }
  else
  {
     drone.rviz_real_test  = false;
  }
  
  
  //if sim_mode && holo_voxl_test are false the test is rviz in real world 


   



  



  //Declare class PID
  
  //Create Directory for print txt files:
   string stringpath = "/home/arpl/luca_ws/DATA/TELE_CONTROL/paper_experiments/iros/" + folder_name + "/";
   int status = mkdir(stringpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
   if (status != 0)
   {
     cout << "[TELE CONTROL] Impossible to create folder to store txt output files" << endl;
   }

  //Define all the OutFile for txt transcription 

  //Drone target position (Case 1: could be unity marker or Rviz Marker, Case 2: Desired position in the rrt line)
  std::ofstream outFile1(stringpath + "x_des_GF.txt");
  std::ofstream outFile2(stringpath + "y_des_GF.txt");
  std::ofstream outFile3(stringpath + "z_des_GF.txt");

  //Drone position in GF (Not Filtered)
  std::ofstream outFile4(stringpath + "x_GF.txt");
  std::ofstream outFile5(stringpath + "y_GF.txt");
  std::ofstream outFile6(stringpath + "z_GF.txt");

 //Drone target Velocity (Case 1: could be unity marker or Rviz Marker, Case 2: Desired position in the rrt line)
  std::ofstream outFile7(stringpath + "x_dot_des_target_GF.txt");
  std::ofstream outFile8(stringpath + "y_dot_des_target_GF.txt");
  std::ofstream outFile9(stringpath + "z_dot_des_GF.txt");

  //Drone velocity in GF 
  std::ofstream outFile10(stringpath + "x_dot_GF.txt");
  std::ofstream outFile11(stringpath + "y_dot_GF.txt");
  std::ofstream outFile12(stringpath + "z_dot_GF.txt");
  //Drone Yaw and Yaw des
  std::ofstream outFile13(stringpath + "yaw.txt");
  std::ofstream outFile14(stringpath + "yaw_des.txt");
  //Force on x axis
  std::ofstream outFile15(stringpath + "F_x.txt");
  //Commanded position and velocity on x axis (ADMITTANCE)
  std::ofstream outFile16(stringpath + "x_c.txt");
  std::ofstream outFile17(stringpath + "x_c_dot.txt");
  //Target acceleration ((Case 1: could be unity marker or Rviz Marker, Case 2: Desired position in the rrt line)
  std::ofstream outFile18(stringpath + "x_ddot_des_target_GF.txt");
  std::ofstream outFile19(stringpath + "y_ddot_des_target_GF.txt");
  //Commanded position and velocity on x axis (ADMITTANCE)
  std::ofstream outFile20(stringpath + "y_c.txt");
  std::ofstream outFile21(stringpath + "y_c_dot.txt");
  //Force on y axis
  std::ofstream outFile22(stringpath + "F_y.txt");
  //Des Position and Velocity estimated from KF  ((Case 1: could be unity marker or Rviz Marker, Case 2: Desired position in the rrt line)
  std::ofstream outFile23(stringpath + "KF_x_pos_est.txt");
  std::ofstream outFile24(stringpath + "KF_x_vel_est.txt");
  std::ofstream outFile25(stringpath + "KF_y_pos_est.txt");
  std::ofstream outFile26(stringpath + "KF_y_vel_est.txt");
  //Estimated position and vel of the unity or rviz marker (ONLY in RRT)
  std::ofstream outFile27(stringpath + "KF_x_user_case_3_pos_est.txt"); //User trajectory vel estimated
  std::ofstream outFile28(stringpath + "KF_y_user_case_3_pos_est.txt");
 
  std::ofstream outFile29(stringpath + "KF_x_user_case_3_vel_est.txt"); //User trajectory vel estimated
  std::ofstream outFile30(stringpath + "KF_y_user_case_3_vel_est.txt");
  // User marker position and Velocity not estimated from KF
  std::ofstream outFile31(stringpath + "user_target_pos_x_case_3.txt"); //User trajectory vel estimated
  std::ofstream outFile32(stringpath + "user_target_pos_y_case_3.txt");

  std::ofstream outFile33(stringpath + "user_target_vel_x_case_3.txt"); //User trajectory vel estimated
  std::ofstream outFile34(stringpath + "user_target_vel_y_case_3.txt");
  //Output of the damping and Gamma value in RRT 
  std::ofstream outFile35(stringpath + "damping_output.txt"); //User trajectory vel estimated
  std::ofstream outFile36(stringpath + "gamma_output.txt");
  std::ofstream outFile37(stringpath + "setpoint_start.txt");
  std::ofstream outFile38(stringpath + "setpoint_end.txt");
  std::ofstream outFile39(stringpath + "fx_obs.txt");
  std::ofstream outFile40(stringpath + "fy_obs.txt");
  std::ofstream outFile41(stringpath + "F_obs_magn.txt");
  



  //Arm Motors 
 auto start = high_resolution_clock::now();
auto time_checkPoint =  high_resolution_clock::now();
auto time_f_obs_published = high_resolution_clock::now();
auto publish_period = 2000000.0; //3 s
auto elapesed_time = duration_cast<microseconds>(start - time_checkPoint);
auto elapsed_time_f_obs_pose_pub = duration_cast<microseconds>(start - time_f_obs_published);

//Set setpoints alomg the line, where the line to follow change the slope  
mission.initial_position.x = 0.0;
mission.initial_position.y = 0.0;
mission.fill_mission_setpoints_vector(&drone);
mission.goal.x = 5.0;
mission.goal.y = 3.0;

int counter_exit_rrt_flag_publish = 0;
   
  drone.start_motors = true;
  drone.disable_tracker_flag = true;
  mission.take_off = true;
  mission.state = 0;
  
  quadrotor_msgs::PositionCommand pos_cmd;

  ros::Rate r(50);
  while (nh.ok())
  {
   

    if (drone.allow_take_off == false)
    {
      cout << "[PRESS S KEY TO ARM MOTORS AND TAKE OFF]" << endl;
       //PRINT TELEMETRY
    cout << "########################################################" << endl;
    cout << "X_GF: " << drone.position_GF.x << " Y_GF: " <<  drone.position_GF.y << " Z_GF: " << drone.position_GF.z << endl;
    cout << "X_vel_GF: " << drone.velocity_GF.x << " Y_vel_GF: " << drone.velocity_GF.y << " Z_vel_GF: " << drone.velocity_GF.z << endl;
    cout << "Des X_Pos_GF: " <<  drone.des_pos_GF.x << " Des Y_pos_GF: " <<  drone.des_pos_GF.y <<  endl;
    cout << "Yaw Rad: " << drone.rpy_orientation.z<< endl;
    
    // drone.des_pos_GF =  drone.position_GF;
    // pos_cmd.position = drone.des_pos_GF;
	  // pos_cmd.velocity = drone.des_vel_GF;
	  // pos_cmd.kx[0] = drone.kx[0];
	  // pos_cmd.kx[1] = drone.kx[1];
	  // pos_cmd.kx[2] = drone.kx[2];
	  // pos_cmd.kv[0] = drone.kv[0];
	  // pos_cmd.kv[1] = drone.kv[1];
	  // pos_cmd.kv[2] = drone.kv[2];
    // pos_cmd.yaw = drone.des_yaw;
    // drone.publish_position_cmd(pos_cmd);

    //ros::Duration(0.5).sleep();
      ros::spinOnce();
    //r.sleep();

      continue;
    }
   
    switch (mission.state)
    {
        //Take Off
	case 0:
  //When the user press s the robot is already taken off and the chnage to the null gtracker take effect
 


        //Place Here a mission.reset function if there are some counter to re update 
  motors_on(&drone,&mission);
  //sleep(2);

	if (elapesed_time.count()  > publish_period && mission.take_off == true && mission.arm_motors_success) 
	{
    take_off(&drone,&mission);
	   mission.take_off = false;
	   mission.arm_motors_success = false;
            
	  
	   //re initialize clock
	   time_checkPoint = high_resolution_clock::now();
	   publish_period = 4000000.0;
	}
 //mission.take_off_success = true;
	if (mission.take_off_success) 
	{
           
	   if (elapesed_time.count()  > publish_period && drone.position_GF.z > 0.75 )
	   {
	       //Go to Hovering State
	       //re initialize clock
	       time_checkPoint = high_resolution_clock::now();

          drone.old_pos_GF_for_hovering = drone.position_GF;
	  drone.old_rpy_orientation_for_hovering = drone.rpy_orientation;
    mission.state = 1;
    mission.previous_state = 0;
    mission.initial_hovering = true;
	     
	   }
      
	
	
     drone.des_pos_GF.z = 1.0;
	  pos_cmd.position = drone.des_pos_GF;
	  pos_cmd.velocity = drone.des_vel_GF;
	  pos_cmd.kx[0] = drone.kx[0];
	  pos_cmd.kx[1] = drone.kx[1];
	  pos_cmd.kx[2] = drone.kx[2];
	  pos_cmd.kv[0] = drone.kv[0];
	  pos_cmd.kv[1] = drone.kv[1];
	  pos_cmd.kv[2] = drone.kv[2];
    pos_cmd.yaw =  drone.rpy_orientation.z;
    drone.publish_position_cmd(pos_cmd);
  }
  
  //  drone.disable_tracker_manager();
  // drone.des_pos_GF =  drone.position_GF;    
	//   pos_cmd.position = drone.des_pos_GF;
	//   pos_cmd.velocity = drone.des_vel_GF;
	//   pos_cmd.kx[0] = drone.kx[0];
	//   pos_cmd.kx[1] = drone.kx[1];
	//   pos_cmd.kx[2] = drone.kx[2];
	//   pos_cmd.kv[0] = drone.kv[0];
	//   pos_cmd.kv[1] = drone.kv[1];
	//   pos_cmd.kv[2] = drone.kv[2];
  //   pos_cmd.yaw =  drone.rpy_orientation.z;

  //   drone.publish_position_cmd(pos_cmd);
  //   drone.old_pos_GF_for_hovering = drone.position_GF;
	//   drone.old_rpy_orientation_for_hovering = drone.rpy_orientation;
  //   mission.state = 1;
  //   mission.previous_state = 0;
  //   mission.initial_hovering = true;
  
   
  
    
  


  break;
  

  case 1:
	  hovering(&drone, &mission, elapesed_time, &kf_x, &kf_y, &kf_user_x, &kf_user_y, &IC_x, &IC_y);    
	break;

	case 2:
	  trajectory_following(&drone, &mission, &IC_x, &IC_y, &IC_z, &kf_x, &kf_y,  &kf_user_x, &kf_user_y);
	break;

  case 3:

     follow_des_trajectory_under_user_interactions_and_perturbations(&drone, &mission, &IC_x, &IC_y, &IC_z, &kf_x, &kf_y, &kf_user_x, &kf_user_y);  
  break;

  
}


   //Define drone publisehr commands topic depending if simulator or mocap
   if (drone.sim_mode == true)
   {
     drone.frame_id = "/simulator";
     drone.publish_tf_transform_from_world_to_sim();
     
   }
   else
   {
    drone.frame_id = "/mocap";
   }


   
   //switch_case_from_user_input(&drone, &mission);
    
   //Switch to Trajectory floowing using impedence if the flag is true
    if (drone.under_unity_manipulation == true && mission.state != 3)
    {
        mission.state = 2;
    } 
 


    //PRINT TELEMETRY
    cout << "########################################################" << endl;
    cout << "X_GF: " << drone.position_GF.x << " Y_GF: " <<  drone.position_GF.y << " Z_GF: " << drone.position_GF.z << endl;
    cout << "X_vel_GF: " << drone.velocity_GF.x << " Y_vel_GF: " << drone.velocity_GF.y << " Z_vel_GF: " << drone.velocity_GF.z << endl;
    cout << "Des X_Pos_GF: " <<  drone.des_pos_GF.x << " Des Y_pos_GF: " <<  drone.des_pos_GF.y <<  endl;
    cout << "Yaw Rad: " << drone.rpy_orientation.z<< endl;

 

  //Write OutFile
    outFile1 << drone.des_pos_GF.x << "\n";
    outFile2 << drone.des_pos_GF.y << "\n";
    outFile3 << drone.des_pos_GF.z << "\n";
    outFile4 << drone.position_GF.x << "\n";
    outFile5 << drone.position_GF.y << "\n";
    outFile6 << drone.position_GF.z << "\n";
    outFile7 << drone.target_vel_GF.x << "\n";
    outFile8 << drone.target_vel_GF.y << "\n";
    outFile9 << drone.target_vel_GF.z << "\n"; 
    outFile10 << drone.velocity_GF.x << "\n";
    outFile11 << drone.velocity_GF.y << "\n";
    outFile12 << drone.velocity_GF.z << "\n";
    outFile13 << drone.rpy_orientation.z << "\n";
    outFile14 <<  drone.des_yaw << "\n";
    outFile15 <<   mission.F_x  << "\n";
    outFile16 <<   drone.comm_pos_GF.x  << "\n";
    outFile17 <<   drone.comm_vel_GF.x  << "\n";
    outFile18 <<   drone.target_acc_GF.x  << "\n";
    outFile19 <<   drone.target_acc_GF.y   << "\n";
    outFile20 <<   drone.comm_pos_GF.y  << "\n";
    outFile21 <<   drone.comm_vel_GF.y  << "\n";
     outFile22 <<   mission.F_y  << "\n";
     outFile23 << kf_x.state_est.x << "\n";
     outFile24 << kf_x.state_est.y << "\n";
     outFile25 << kf_y.state_est.x << "\n";
     outFile26 << kf_y.state_est.y << "\n";
   //Only in case 3
    outFile27 << kf_user_x.state_est.x << "\n";
     outFile28 << kf_user_y.state_est.x << "\n";
     outFile29 << kf_user_x.state_est.y << "\n";
   outFile30 << kf_user_y.state_est.y << "\n";
   outFile31 << drone.des_user_pos_GF.x << "\n";
   outFile32 << drone.des_user_pos_GF.y << "\n";
   outFile33 << drone.target_user_vel_GF.x << "\n";
   outFile34 << drone.target_user_vel_GF.y << "\n";
   outFile35 << drone.d_output << "\n";
   outFile36 << drone.gamma_output << "\n";
   outFile37 << mission.setpoint_old.x << ", " << mission.setpoint_old.y << endl;
   outFile38 << mission.setpoint.x << ", " << mission.setpoint.y << endl;
   outFile39 << drone.force_obs_res.x << "\n";
   outFile40 << drone.force_obs_res.y << "\n";
   outFile41 << drone.force_obs_Magn << "\n";


   
   
   drone.des_pos_GF_old = drone.des_pos_GF;
   drone.target_vel_GF_old = drone.target_vel_GF;
   drone.target_acc_GF_old = drone.target_acc_GF;

   drone.des_user_pos_GF_old = drone.des_user_pos_GF;
   drone.target_user_vel_GF_old = drone.target_user_vel_GF;
   drone.target_user_acc_GF_old = drone.target_user_acc_GF;

   drone.flagQuadSimOdom = false;
   drone.flagMarkerPosition = false;
   drone.flagCaseSelectedKeyboard = false;
   drone.flagDroneUnderManipulationFlag = false;
   drone.flagInteractiveMarkerPosition = false;
   drone.flagRRTPath = false;  
   drone.flagRRTStartAssistiveGuidance = false;
   drone.flagRRTFinalGoal = false;
   drone.flagRRTVirtualObstacles = false;
   drone.flagObstacleForceResReceived = false;

   if (drone.rrt_exit_assistive_guidance == true && counter_exit_rrt_flag_publish < 100)
   {
    counter_exit_rrt_flag_publish = counter_exit_rrt_flag_publish + 1;
   }
   else
   {
    drone.rrt_exit_assistive_guidance = false;
    counter_exit_rrt_flag_publish = 0;
   }
   
   
   drone.publish_exit_from_rrt_search_mode( drone.rrt_exit_assistive_guidance );
  
  //  if (holo_voxl_test == true)
  //  {
  //      drone.publish_voxl_tf();
  //  }
  
   

   //necessario per determinare se il flag di manipolazione è falso in maniera consistetnte. 
   //la callback è piu lenta del loop while, quindi anche se il drone è in manipolazione potrebbe risulatare falso perche bon entra nella callback
   if (drone.under_unity_manipulation == false)
   {
       mission.counter_unity_manipulation_flag_is_flase =  mission.counter_unity_manipulation_flag_is_flase + 1;
   }
   else
   {
    mission.counter_unity_manipulation_flag_is_flase = 0;
   }

   
  
   string frame_id = "mocap_sr";  //mocap_sr is rigidly attached to
   mission.visualize_line_to_follow_in_rviz(&drone, frame_id);
   mission.visualize_virtual_obstacles_in_RVIZ(&drone, frame_id);
   mission.visualize_arrow_force_in_rviz(&drone, frame_id);
   mission.visualize_marker_bypass_manouver_rviz(&drone, frame_id);
   mission.visualize_path_and_history(&drone, frame_id, elapsed_time_f_obs_pose_pub);

   //Publish Mesh Resource 
   drone.publishQuadrotorMarker(frame_id);
   drone.publishRRTFinalGoalMarker(frame_id, drone.publish_rrt_final_goal_marker_flag);
   drone.publish_rrt_inside_bypass_manouver(mission.inside_manouver);
   drone.publish_bypass_marker_to_unity(mission.fixed_marker_for_manouver_to_unity);

   
   if (drone.rviz_real_test == true)
   {
      drone.republish_interactive_marker_position_for_rosbags(drone.marker_position);
   }
  
   
   drone.publish_rrt_final_goal_marker_flag = false;
   drone.under_unity_manipulation = false;

   if (mission.pose_array_f_obs_pub)
   {
     time_f_obs_published =  high_resolution_clock::now();
     mission.pose_array_f_obs_pub = false;
   }

   
   start = high_resolution_clock::now();
   elapesed_time = duration_cast<microseconds>(start - time_checkPoint);
   elapsed_time_f_obs_pose_pub = duration_cast<microseconds>(start - time_f_obs_published);

  
   ros::spinOnce();
   r.sleep();
}
}
