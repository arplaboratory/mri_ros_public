#include "drone_teleoperation/core/admittance_core.h"


#include <vector>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <numeric>

namespace hri_admittance {

  AdmittanceCore::AdmittanceCore(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private, string path)
                    : nh_(nh),
                    nh_private_(nh_private),
                    utils(nh, nh_private),
                    planner_utils(nh, nh_private),
                    visualizer(nh, nh_private),
                    log_files(path),
                    avg_stamps(20),
                    IC_x(5.5, 5.5,10, -10, 2.5, 50,10),
                    IC_y(5.5, 5.5,10, -10, 2.5, 50,10),
                    kf_x(),
                    kf_y(),
                    kf_user_x(),
                    kf_user_y()
                    {
                   
            //Admittance Filter parameters 
            double Ks_x, Ks_y, Kd_x, Kd_y, F_max, F_min;
            double M, D_x, D_y, K_x, K_y;
            double K_haptic_adm_x, K_haptic_adm_y, Kp_haptic_obst_x,Kp_haptic_obst_y;
            float Kd_haptic_obst_x, Kd_haptic_obst_y;
          
            Eigen::Vector3f P1;
            Eigen::Vector3f P2;
            nh_.getParam("/tele_control_params/force_gains/pos/x", Ks_x);
            nh_.getParam("/tele_control_params/force_gains/pos/y", Ks_y);
            nh_.getParam("/tele_control_params/force_gains/vel/x", Kd_x);
            nh_.getParam("/tele_control_params/force_gains/vel/y", Kd_y);
            nh_.getParam("/tele_control_params/force_gains/F_max", F_max);
            nh_.getParam("/tele_control_params/force_gains/F_min", F_min);
            nh_.getParam("/tele_control_params/admittance_gains/x_dir/M", M);
            nh_.getParam("/tele_control_params/admittance_gains/x_dir/D", D_x);
            nh_.getParam("/tele_control_params/admittance_gains/x_dir/K", K_x);
            nh_.getParam("/tele_control_params/admittance_gains/y_dir/D", D_y);
            nh_.getParam("/tele_control_params/admittance_gains/y_dir/K", K_y);
            nh_.getParam("/tele_control_params/n_average_stamps", avg_stamps);
            nh_.getParam("/tele_control_params/log_files_directory", path );
            nh_.getParam("/tele_control_params/desired_txt_folder_name", folder_name );
            //For task pupropse 
            nh_.getParam("/tele_control_params/intermediate_goals/P1/x", P1(0) );
            nh_.getParam("/tele_control_params/intermediate_goals/P1/y", P1(1) );
            nh_.getParam("/tele_control_params/intermediate_goals/P2/x", P2(0) );
            nh_.getParam("/tele_control_params/intermediate_goals/P2/y", P2(1) );
            nh_.getParam("/tele_control_params/intermediate_goals/random",random_intermediate_p);
            //Force parameters to amplify the force sent to the haptic devide 
            nh_.getParam("/tele_control_params/haptic_gains/admittance_force/x", K_haptic_adm_x);
            nh_.getParam("/tele_control_params/haptic_gains/admittance_force/y", K_haptic_adm_y);
            nh_.getParam("/tele_control_params/haptic_gains/Kp_obst_force/x", Kp_haptic_obst_x);
            nh_.getParam("/tele_control_params/haptic_gains/Kp_obst_force/y", Kp_haptic_obst_y );
            nh_.getParam("/tele_control_params/haptic_gains/Kd_obst_force/x", Kd_haptic_obst_x);
            nh_.getParam("/tele_control_params/haptic_gains/Kd_obst_force/y", Kd_haptic_obst_y);


            //Haptic gains vector composition 
            K_admittance = Eigen::Vector2f((float)K_haptic_adm_x, (float)K_haptic_adm_y);
            K_obstacles = Eigen::Vector2f((float)Kp_haptic_obst_x,(float) Kp_haptic_obst_y);
            Kd_ = Eigen::Vector2f((float)Kd_haptic_obst_x,(float) Kd_haptic_obst_y);
            //Admittance class Initialization
            IC_x = Admittance_Controller(Ks_x, Kd_x, F_max, F_min, M, D_x, K_x);
            IC_y = Admittance_Controller(Ks_y, Kd_y, F_max, F_min, M, D_y, K_y);

            
            marker_vel = Eigen::Vector3f(0.0 ,0.0,0.0);
            marker_acc = Eigen::Vector3f(0.0 ,0.0,0.0);
            marker_pos_old = Eigen::Vector3f(0.0 ,0.0,0.0);
            marker_vel_old = Eigen::Vector3f(0.0 ,0.0,0.0);

            //Fill the Waypoint Vector 
            intermediate_points.push_back(P1);
            intermediate_points.push_back(P2);
            intermediate_points_initial =intermediate_points; 

            }
    


    void AdmittanceCore::initialize_KF()
    {

         Eigen::Matrix2d P;
         Eigen::Matrix2d R;
         Eigen::Vector2d X_init;
         double sigma_x1 = 1.0;
         double sigma_x2 = 1.0;
         // Init covariance Matrix
         P << sigma_x1, 0,
             0, sigma_x2;
         // Init Measurement uncertanti Matrix : Uncertanty on position obs and vel obs
         R << 0.2, 0,
             0, 0.6;
          
          marker_pos = Eigen::Vector3f(0.0,0.0,0.0);
          marker_vel = Eigen::Vector3f(0.0,0.0,0.0);

        
          X_init <<  marker_pos(0), marker_vel(0) ;
          kf_x.pass_to_class_initialization_matrices(P, R, X_init);
          X_init <<  marker_pos(1), marker_vel(1) ;
          kf_y.pass_to_class_initialization_matrices(P, R, X_init);
          marker_proj_pos = Eigen::Vector3f(0.0,0.0,0.0);
          marker_proj_pos_old = Eigen::Vector3f(0.0,0.0,0.0);
          marker_proj_vel = Eigen::Vector3f(0.0,0.0,0.0);
          marker_proj_vel_old = Eigen::Vector3f(0.0,0.0,0.0);
          marker_proj_acc = Eigen::Vector3f(0.0,0.0,0.0);
      
         
      
         
         X_init <<  marker_pos(0), marker_vel(0);
         kf_user_x.pass_to_class_initialization_matrices(P, R, X_init);
         X_init <<  marker_pos(1), marker_vel(1);
         kf_user_y.pass_to_class_initialization_matrices(P, R, X_init);
        
         

    }

   void AdmittanceCore::average_obs_force(float *fx, float *fy, int avg_stamps_)
   {
   
     //float  avg_stamps_ = 40;
     // AVeraging Position along X axis 
     fx_obs_vec.push_back(*fx);
     *fx = utils.average(&fx_obs_vec);
   
     //Clean the vector 
     if (fx_obs_vec.size() > avg_stamps_)
     {
       fx_obs_vec.erase(fx_obs_vec.begin());
     } 
   
     // AVeraging Position along Y axis 
     fy_obs_vec.push_back(*fy);
     *fy = utils.average(&fy_obs_vec);
     //Clean the vector 
     if (fy_obs_vec.size() > avg_stamps_)
     {
       fy_obs_vec.erase(fy_obs_vec.begin());
     } 
   }

    void AdmittanceCore::smoothing_target_traj(int scenario)
    {
        if (scenario == 1 || scenario == 3) //Rviz Marker
        {
           //Define an if statement if the position is coming from rviz or the haptic marker 
           Eigen::Vector3f marker_odom;
           if (haptic_device)
           {
            marker_odom = haptic_marker_position;
           }
           else
           {
            marker_odom = rviz_marker_position;
           }

        
              // AVeraging Position along X axis 
              target_pos_x_vec.push_back(marker_odom(0));
              marker_pos(0) = utils.average(&target_pos_x_vec);
            
              //Averaging Position along Y axis
              target_pos_y_vec.push_back(marker_odom(1));
              marker_pos(1)= utils.average(&target_pos_y_vec);
              marker_pos(2) = des_z;
         
            if (APVI)
            {
      
              // AVeraging Projected marker position on the line l1 
            target_pos_x_l1_planner.push_back(planner_utils.marker_projected_position_on_line(0));
            marker_proj_pos(0) = utils.average(&target_pos_x_l1_planner);
            
            //Averaging Position along Y axis
            target_pos_y_l1_planner.push_back(planner_utils.marker_projected_position_on_line(1));
            marker_proj_pos(1)= utils.average(&target_pos_y_l1_planner);
            marker_proj_pos(2) = des_z;

          
            }
            else
            {
                            // AVeraging Projected marker position on the line l1 
            target_pos_x_l1_planner.push_back(marker_odom(0));
            marker_proj_pos(0) = utils.average(&target_pos_x_l1_planner);
            
            //Averaging Position along Y axis
            target_pos_y_l1_planner.push_back(marker_odom(1));
            marker_proj_pos(1)= utils.average(&target_pos_y_l1_planner);
            marker_proj_pos(2) = des_z;

            }
          
            //Publish the Force Aroow marker for visualization
            visualizer.publish_arrow_force_in_rviz(marker_odom, quad_pose);
            
        }
        else
        {
            //The Marker comes from Unity Hologram 
            target_pos_x_vec.push_back(unity_marker_position(0));
            marker_pos(0) = utils.average(&target_pos_x_vec);
            
            //Averaging Position along Y axis
            target_pos_y_vec.push_back(unity_marker_position(1));
            marker_pos(1)= utils.average(&target_pos_y_vec);
            marker_pos(2) = des_z;
            
            if (APVI)
            {
              // AVeraging Projected marker position on the line l1 
            target_pos_x_l1_planner.push_back(planner_utils.marker_projected_position_on_line(0));
            marker_proj_pos(0) = utils.average(&target_pos_x_l1_planner);
            
            //Averaging Position along Y axis
            target_pos_y_l1_planner.push_back(planner_utils.marker_projected_position_on_line(1));
            marker_proj_pos(1)= utils.average(&target_pos_y_l1_planner);
            marker_proj_pos(2) = des_z;
            }
             else
            {
                            // AVeraging Projected marker position on the line l1 
            target_pos_x_l1_planner.push_back(unity_marker_position(0));
            marker_proj_pos(0) = utils.average(&target_pos_x_l1_planner);
            
            //Averaging Position along Y axis
            target_pos_y_l1_planner.push_back(unity_marker_position(1));
            marker_proj_pos(1)= utils.average(&target_pos_y_l1_planner);
            marker_proj_pos(2) = des_z;

            }
          
             //Publish the Force Aroow marker for visualization
            visualizer.publish_arrow_force_in_rviz(unity_marker_position, quad_pose);

        }
        
        // //Safety check on the time stamps to avoid very risky maneuver or accelerations 
        // Eigen::Vector3f diff = marker_pos - marker_pos_old;
        // float distance_marker_setpoints = sqrt(pow(diff(0),2) + pow(diff(1),2));
        // cout << "Distance: " << distance_marker_setpoints << endl;
        // if (distance_marker_setpoints > 0.020 ||safety_flag == true) {
        //   if (safety_flag == false){
        //       //Save safety position at the first time it enters in the if 
        //       marker_pos_safety = marker_pos; 
        //   }
        //     safety_flag = true;
        //   //Evaluate target marker and drone distance 
        //     Eigen::Vector3f diff = marker_pos - quad_pose;
        //     float distance_marker_drone = sqrt(pow(diff(0),2) + pow(diff(1),2));
        //     if(distance_marker_drone < 0.10){
        //        Eigen::Vector3f marker_pos_int; 
        //       // //interplate the trajectory 
        //       trajectory_ended = planner_utils.line_tracker(Eigen::Vector3f quad_pos, Eigen::Vector3f goal_pos,  float dt, bool *init)
        //       marker_pos_int = planner_utils.interpolate_drone_position_with_user_interaction_marker(&safety_flag, marker_pos, marker_pos_safety);
        //       cout << "Interpolation Safety Trajectory: " << safety_flag << ", " << marker_pos_int<< endl;

        //       marker_pos = marker_pos_int;
        //     }
        //     else{
        //       marker_pos = marker_pos_safety;
        //     }
        // }

        //if (safety_flag)
        //Clean the vectors 
        if (target_pos_x_vec.size() > avg_stamps)
        {
          target_pos_x_vec.erase(target_pos_x_vec.begin());
        } 
        if (target_pos_y_vec.size() > avg_stamps)
        {
         target_pos_y_vec.erase(target_pos_y_vec.begin());
        }
        
        if (target_pos_x_l1_planner.size() > avg_stamps)
        {
          target_pos_x_l1_planner.erase(target_pos_x_l1_planner.begin());
        } 
        if (target_pos_y_l1_planner.size() > avg_stamps)
        {
         target_pos_y_l1_planner.erase(target_pos_y_l1_planner.begin());
        }
        
        //Clear The APVI Vectors when the signal to exit from the APVi is called 
        // if (leave_APVI || planner_utils.planner_final_goal_reached)
        // {
        //   target_pos_x_l1_planner.clear();
        //   target_pos_y_l1_planner.clear();
        // }
    }
    
    void AdmittanceCore::safety_interpolation_check(float dist_th, float dt)
    {

      //Check the various case
      //Case 1: Safety check distance between consecutive positions of the i teractive marker 
      Eigen::Vector3f diff = marker_pos - marker_pos_old;
      float distance_marker_setpoints = sqrt(pow(diff(0),2) + pow(diff(1),2)); 
      if (distance_marker_setpoints > 0.05) // can enter here only during the exection and not when the user take the control the first time. 
                                                                                        //This because the robot can be still on the ground and then the tracker generate a trajectpry starting with the orbot on the ground. After the take off the robot still trying to reach the first point generated by the line tracker still on the ground
      {
        safety_flag = true;
        go_to_hover = true;
        last_quad_position = quad_pose;

      }
      float vel_magn = sqrt(pow(quad_vel(0), 2) + pow(quad_vel(1), 2));

      if (vel_magn > 0.75){
        safety_flag = true;
      }
      //Case 2: Safety check on the null tracker transition
      if (null_tracker_trasition_success)
      {
        safety_flag = true;
        counter_safety = 0;
      }
      //Case 3: Safety check on transition from APVI and FPVI depending on the commanded position distance from the quadrotor

      float x_tr = IC_x.obtain_commanded_position();
      float y_tr = IC_y.obtain_commanded_position();
      float z_tr = quad_pose(2);
      Eigen::Vector3f tracker_goal_position = Eigen::Vector3f(x_tr, y_tr, z_tr);
      diff = quad_pose - tracker_goal_position;
      float distance_3D = sqrt(pow(diff(0), 2) +pow(diff(1),2) +  pow(diff(2),2)); 
      if (distance_3D > dist_th)
      {
        safety_flag = true;
      }
      //Safety check on the time stamps to avoid very risky maneuver or accelerations 

      if (safety_flag== true) {
        if (counter_safety == 0){
            utils.traj_track_start = true;
            null_tracker_trasition_success = false;
        }
        // //interplate the trajectory 
        utils.goal_pose_tracker = tracker_goal_position;
        utils.line_tracker(quad_pose, dt);
        
        pos_comm = utils.curr_des_pos;
        vel_comm = utils.curr_des_vel;
        acc_comm = utils.curr_des_acc;
        
        //Only if go _to_hover is true 
        if (go_to_hover)
        {
          pos_comm = last_quad_position;
          vel_comm = Eigen::Vector3f(0.0, 0.0, 0.0);
          acc_comm = Eigen::Vector3f(0.0, 0.0, 0.0);

          //increase go to hover counter before switching to trajectory tracking 
          if (counter_safety > 300)
              go_to_hover = false;
        }
        cout <<"[safety_interpolation_check] pos_comm: " << pos_comm << ", vel_comm: " << vel_comm << endl; 
        cout <<"[safety_interpolation_check]  Tracker Goal Position: " << tracker_goal_position << endl; 
        cout <<"[safety_interpolation_check] distance_3D: " << distance_3D << endl;
        counter_safety = counter_safety + 1;
        if (utils.trajectory_ended && distance_3D < 0.10){
            safety_flag = false;
            counter_safety = 0;
        }
        
      }
      return;
  }

    void AdmittanceCore::background_kf_user_update(float dt)
    {
       kf_user_x.calculate(marker_pos(0), marker_vel(0), marker_acc(0), dt);
       kf_user_x.state_est.x = kf_user_x.Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
       kf_user_x.state_est.y = kf_user_x.Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
       cout << "[BACKGROUND USER TR TRACKING] [X AXIS] KF States: X: " << kf_user_x.state_est.x << " Y: " << kf_user_x.state_est.y << endl;
     
       kf_user_y.calculate(marker_pos(1), marker_vel(1), marker_acc(1), dt);
       kf_user_y.state_est.x = kf_user_y.Obtain_KF_estimated_state()[0];                                                  // Pos Filtered on X axis
       kf_user_y.state_est.y = kf_user_y.Obtain_KF_estimated_state()[1];                                                  // Vel Filtered on X axis
       cout << "[BACKGROUND USER TR TRACKING] [Y AXIS] KF States: X: " << kf_user_y.state_est.x << " Y: " << kf_user_y.state_est.y << endl;

       //If not in apvi still tracking the variable with the actual marker position to avoid crazye oscillation 
       marker_proj_pos = marker_pos;
       marker_proj_vel = marker_vel;
       marker_proj_acc = marker_acc;

       marker_proj_pos_old = marker_pos_old;
       marker_proj_vel_old = marker_vel_old;
    }


    void AdmittanceCore::evaluate_target_position_derivatives(int scenario, float dt)
    {
        //Compute the (RVIZ UNITY) marker velocity in FPVI  from its position at tim t and at time t-1
      float x_dot = 0.0;
      float y_dot = 0.0;
      float z_dot = 0.0;
      float x_ddot = 0.0;
      float y_ddot = 0.0;
      float z_ddot = 0.0;
      
      x_dot = (marker_pos(0) - marker_pos_old(0))/dt;
      y_dot = (marker_pos(1) - marker_pos_old(1))/dt;
      z_dot = (marker_pos(2) - marker_pos_old(2))/dt;
      marker_vel = Eigen::Vector3f(x_dot,y_dot,z_dot);

      x_ddot =  (marker_vel(0) - marker_vel_old(0))/dt;
      y_ddot =  (marker_vel(1) - marker_vel_old(1))/dt;
      z_ddot =  (marker_vel(2) - marker_vel_old(2))/dt;
      marker_acc = Eigen::Vector3f(x_ddot,y_ddot,z_ddot);

      x_dot = (marker_proj_pos(0) - marker_proj_pos_old(0))/dt;
      y_dot = (marker_proj_pos(1) - marker_proj_pos_old(1))/dt;
      z_dot = (marker_proj_pos(2) - marker_proj_pos_old(2))/dt;
      marker_proj_vel = Eigen::Vector3f(x_dot,y_dot,z_dot);
     
      x_ddot =  (marker_proj_vel(0) - marker_proj_vel_old(0))/dt;
      y_ddot =  (marker_proj_vel(1) - marker_proj_vel_old(1))/dt;
      z_ddot =  (marker_proj_vel(2) - marker_proj_vel_old(2))/dt;
      marker_proj_acc = Eigen::Vector3f(x_ddot,y_ddot,z_ddot);
      
    }
    
    void  AdmittanceCore::free_admittance_interaction(bool *init, int scenario, Eigen::Vector3f f_obs_vec, float f_obs_magn, Eigen::Vector2f p_distance, Eigen::Vector2f p_distance_dot, float dt)
    {  
        
        if (*init == true)
        {
          // //Take the desired altitude
         
          // if (des_z < 0.2)
          //      des_z = 0.8;
          //Initialize KF for human target position estimation if this is the first interaction
           initialize_KF();
           if (counter_init > 100)
                *init = false;
           counter_init = counter_init+1;
           return;
        }
        
        cout << "[IC Core] change_yaw_active: " << change_yaw_active << endl;
        cout << "[IC Core] change_yaw180: " << change_yaw180 << endl;
        
        if (change_yaw180){
          if (des_yaw == 0.0){
              des_yaw = 0 + M_PI + 0.1 ;
              if (des_yaw > M_PI){
                float diff = des_yaw - M_PI;
               des_yaw = -M_PI + diff;
              }
          }
          else
          {
            des_yaw = 0.0;
          }
         
          
          change_yaw180 = false;
          change_yaw_active = true;
        }
         cout << "[IC Core] des_yaw: " << des_yaw << endl;
         yaw_value = utils.yaw_tracker(quad_orientation(2), des_yaw , &change_yaw_active, dt);
         



        APVI = false;
        smoothing_target_traj(scenario);
        safety_interpolation_check(0.20, dt);
        evaluate_target_position_derivatives(scenario, dt); 
        
        hovering = utils.check_safety_boundaries(quad_pose);

        // Filtering Position and velocity of the desired trajectory autonomously generated
        kf_x.calculate(marker_pos(0), marker_vel(0), marker_acc(0), dt);
        kf_x.state_est.x = kf_x.Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
        kf_x.state_est.y = kf_x.Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
        cout << "[IC Core] [X AXIS] KF States: X: " << kf_x.state_est.x << " Y: " << kf_x.state_est.y << endl;
       
       
        kf_y.calculate(marker_pos(1), marker_vel(1),  marker_acc(1), dt); 
        kf_y.state_est.x = kf_y.Obtain_KF_estimated_state()[0];                                        // Pos Filtered on X axis
        kf_y.state_est.y = kf_y.Obtain_KF_estimated_state()[1];  
        cout << "[IC core] [Y AXIS] KF States: X: " << kf_y.state_est.x << " Y: " << kf_y.state_est.y << endl;
        
        //evaluate background the kf user filter between one APVI session and the following 
        background_kf_user_update(dt);
        
        int avg_force = 40; 
        float f_obs_x =  f_obs_vec(0);
        float f_obs_y =  f_obs_vec(1);
        average_obs_force(&f_obs_x, &f_obs_y, avg_force); 
        f_obs_vec(0) = f_obs_x;
        f_obs_vec(1) = f_obs_y;
        f_obs_vec_ = f_obs_vec;
        cout << "Fx obs: " << f_obs_vec(0) << ", " << f_obs_vec(1) << endl;
        IC_x.calculate(kf_x.state_est.x, quad_pose(0),  kf_x.state_est.y , quad_vel(0), f_obs_vec(0), dt); 
        IC_y.calculate(kf_y.state_est.x, quad_pose(1),  kf_y.state_est.y,  quad_vel(1), f_obs_vec(1), dt); 
        
        F_admittance = Eigen::Vector3f(IC_x.obtain_evaluated_force(), IC_y.obtain_evaluated_force(), 0.0 );
        
        //altitude controller:
       float z_diff = des_z - quad_pose(2);
       float z_vel_des = 1 - (tanh(z_diff)/z_diff);
       if (z_diff <= 0)
            z_vel_des = -1*z_vel_des;
       if (z_diff<0.03 && z_diff > -0.03)
           z_vel_des = 0.0;
        marker_vel(2) = z_vel_des;

        //Obtain the commanded position and velocity to forward to the controller
      if (safety_flag == false)
        {
           pos_comm = Eigen::Vector3f(IC_x.obtain_commanded_position(),  IC_y.obtain_commanded_position(), des_z);
           vel_comm = Eigen::Vector3f(IC_x.obtain_commanded_velocity(),  IC_y.obtain_commanded_velocity(), z_vel_des);
           acc_comm = Eigen::Vector3f(0.0,  0.0,0.0);
        }
        
        
        if (planner_utils.planner_start_assistive_guidance && planner_utils.planner_path_points.size() > 0)
        {
          //Change To APVI Mode since some waypoints are received 
          APVI = true;
          planner_utils.planner_start_assistive_guidance = false;
          leave_APVI = false;
          intermediate_points = intermediate_points_initial;
          init_APVI_transition = true; //required to the next transition from APVI to FPVI
          change_scenario = true; //flag to update the APVI or FPVI start task counters 
          safety_d_th = 20;
        }
      
        //Visualize Goal 
        if (visualize_goals){
           if (random_intermediate_p && goal_selected == false){
              intermediate_points.clear();
              intermediate_points = planner_utils.generate_random_intermediate_points_location();
           }

           intermediate_points = visualizer.publish_intermediate_goals(intermediate_points, quad_pose, &FPVI_intermediate_target_counter);
           //Select the Final Goal Position to send to the visualizer
           if (goal_selected == false)
           {
              planner_utils.find_goal_position(APVI);
              goal_selected = true;
           }
           planner_utils.final_goal_position(2) = quad_pose(2);
           visualizer.publishRRTFinalGoalMarker(&visualize_goals, quad_pose, planner_utils.final_goal_position);
           
        }
        else
        {
          intermediate_points = intermediate_points_initial;
          goal_selected = false;
        }

         increasing_FPVI_counters();

        //Pass data to log file class 
        pass_data_to_log_file_class();
        
       //sum obstacles and interaction forces together 
      Eigen::Vector2f d_APVI = Eigen::Vector2f(0.0, 0.0);
      Eigen::Vector2f d_APVI_dot = Eigen::Vector2f(0.0, 0.0);
      sum_forces(p_distance, p_distance_dot, d_APVI, d_APVI_dot, dt);
       //publish haptic marker visualization 
     
      visualizer.publishHapticInteractionMarker(marker_pos,quad_vel, haptic_device);
      visualizer.publishVirtualObstacles(scenario);
        //Pass data to the class for the next interaction otherwise the quadrotor pose is shifted and the first waypoint ill be placed far away
       planner_utils.get_quad_pose(quad_pose,quad_orientation);
       planner_utils.get_des_pose( marker_pos); //forwardig the desired position defined from the marker
       planner_utils.get_commanded_pose( pos_comm);

        marker_pos_old = marker_pos;
        marker_vel_old = marker_vel;

        marker_proj_pos_old = marker_proj_pos;
        marker_proj_vel_old = marker_proj_vel;
       counter_APVI_updated = false;
       //To the writer 
      drone_line_distance_APVI = 0; //when not in APVI case this will be written in the log as zero, makes easy when the data will be analyzed to see where the transition is
      FPVI_final_goal_reached = 0;
    }

    void  AdmittanceCore::assisted_admittance_interaction(bool *init, int scenario, Eigen::Vector3f f_obs_vec, float f_obs_magn,Eigen::Vector2f  p_distance, Eigen::Vector2f  p_distance_dot, float dt)
    { 
       if (change_scenario)
       {
        APVI_start_task_counter = APVI_start_task_counter +1;
        change_scenario = false;
       }
        APVI = true;
       //Pass data to the class 
       planner_utils.get_quad_pose(quad_pose,quad_orientation);
       planner_utils.get_des_pose( marker_pos); //forwardig the desired position defined from the marker
       hovering = utils.check_safety_boundaries(quad_pose);
      /*
       This Function find the point P3, which is the perpendicular projection of the drone position on the line l1.
       The line l1 is the line between a sequence of two consecutive setpoints.
      */
       planner_utils.project_drone_position_on_segment();
       
      // FInd the distance between the drone position and the projection of the commanded position XC on the drone-goal line
      if (init_APVI_transition ||safety_flag==true ){
        planner_utils.project_des_position_on_segment();
      }
      else
      {
        //interpolate at each iteration the admittance desired position (the target projection on the line ) one step closer to the actual interactive marker transition 
        //to complete the transition to the modality and avoiding jump in the admittance 
        utils.final_position_for_transition =  marker_pos;
        utils.safety_admittance_desired_position_transition();
        planner_utils.marker_projected_position_on_line = utils.interpolation_updated_position;
      }
       
      //  //Evaluate the trajectories 

      smoothing_target_traj(scenario);
      if (leave_APVI==false && planner_utils.planner_final_goal_reached==false)
            safety_interpolation_check(safety_d_th, dt);
      evaluate_target_position_derivatives(scenario, dt); 

      // // Filtering Position and velocity of the marker position. 
        kf_x.calculate(marker_pos(0), marker_vel(0), marker_acc(0), dt);
        kf_x.state_est.x = kf_x.Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
        kf_x.state_est.y = kf_x.Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
        cout << "[AUTONOMOUS TR TRACKING] [X AXIS] KF States: X: " << kf_x.state_est.x << " Y: " << kf_x.state_est.y << endl;
       
       
        kf_y.calculate(marker_pos(1), marker_vel(1),  marker_acc(1), dt); 
        kf_y.state_est.x = kf_y.Obtain_KF_estimated_state()[0];                                        // Pos Filtered on X axis
        kf_y.state_est.y = kf_y.Obtain_KF_estimated_state()[1];  
        cout << "[AUTONOMOUS TR TRACKING] [Y AXIS] KF States: X: " << kf_y.state_est.x << " Y: " << kf_y.state_est.y << endl;

        // FFiltering Position and velocity of the marker position projected on the line (actual input to the admittance controller) 
     
       kf_user_x.calculate(marker_proj_pos(0), marker_proj_vel(0), marker_proj_acc(0), dt);
       kf_user_x.state_est.x = kf_user_x.Obtain_KF_estimated_state()[0]; // Pos Filtered on X axis
       kf_user_x.state_est.y = kf_user_x.Obtain_KF_estimated_state()[1]; // Vel Filtered on X axis
       cout << "[USER TR TRACKING] [X AXIS] KF States: X: " << kf_user_x.state_est.x << " Y: " << kf_user_x.state_est.y << endl;
     
       kf_user_y.calculate(marker_proj_pos(1), marker_proj_vel(1), marker_proj_acc(1), dt);
       kf_user_y.state_est.x = kf_user_y.Obtain_KF_estimated_state()[0];                                                  // Pos Filtered on X axis
       kf_user_y.state_est.y = kf_user_y.Obtain_KF_estimated_state()[1];                                                  // Vel Filtered on X axis
       cout << "[USER TR TRACKING] [Y AXIS] KF States: X: " << kf_user_y.state_est.x << " Y: " << kf_user_y.state_est.y << endl;

      //  // Evaluate the constrained admittance control 
      float gain_fx, gain_fy;
      int avg_force = 40;
      safety_d_th = 20;
      if (counter_slope < 1500)
      {
        slope = (1514 - counter_slope )/1500;
        counter_slope = counter_slope + 1;
        avg_force = 100;
        //distance threshold for the safety i terpolation and line tracker activation during the transition 
        safety_d_th = 0.10;
      }
       float f_obs_x =  f_obs_vec(0);
       float f_obs_y =  f_obs_vec(1);
       average_obs_force(&f_obs_x, &f_obs_y, avg_force); 
      //  f_obs_vec(0) = f_obs_x;
      //  f_obs_vec(1) = f_obs_y;
       f_obs_vec_ = Eigen::Vector3f(f_obs_x, f_obs_y, 0.0);
       planner_utils.evaluate_variable_damping_coefficient();
    
       IC_x.calculate_xc_from_planned_trajectory(kf_x.state_est.x, kf_user_x.state_est.x, quad_pose(0), 
                                             kf_x.state_est.y, kf_user_x.state_est.y, quad_vel(0), 
                                             f_obs_x, (double)planner_utils.d_output, dt);
       IC_y.calculate_xc_from_planned_trajectory(kf_y.state_est.x, kf_user_y.state_est.x, quad_pose(1), 
                                             kf_y.state_est.y, kf_user_y.state_est.y,  quad_vel(1), 
                                               f_obs_y, (double)planner_utils.d_output, dt);
       
       F_admittance = Eigen::Vector3f(IC_x.obtain_evaluated_force(), IC_y.obtain_evaluated_force(), 0.0 );
       
       //altitude controller:
      float z_diff = des_z - quad_pose(2);
       float z_vel_des = 1 - (tanh(z_diff)/z_diff);
       if (z_diff <= 0)
            z_vel_des = -1*z_vel_des;
       if (z_diff<0.03 && z_diff > -0.03)
           z_vel_des = 0.0;
      marker_vel(2) = z_vel_des;

      if (leave_APVI || planner_utils.planner_final_goal_reached) //check if to add also under unity manipulation 
      { 
       
        if (planner_utils.planner_final_goal_reached && counter_APVI_updated==false)
        {
            APVI_target_goal_counter = APVI_target_goal_counter + 1;
            counter_APVI_updated = true;
        }

        if (init_APVI_transition)
        {
          //Save the current commanded position and the marker projected position on the line which define the starting point of the line tracker 
          des_pos_for_transition =  Eigen::Vector3f(IC_x.obtain_commanded_position(),  IC_y.obtain_commanded_position(), des_z);
          des_vel_for_transition =  Eigen::Vector3f(0.0, 0.0, 0.0);
          des_acc_for_transition = Eigen::Vector3f(0.0,  0.0,0.0);

          //Pass the starting position to the safety admittance desired interpolation 
          utils.start_pos_for_transition = marker_proj_pos;
          utils.final_position_for_transition =  marker_pos;
        }
        
        //While the tracker is transition, keep the robot hovering on the latest evaluated position from the admittance 
       pos_comm = des_pos_for_transition;
       vel_comm = des_vel_for_transition;
       acc_comm = des_acc_for_transition;
       init_APVI_transition = false;

        if (utils.enable_transition){
          planner_utils.planner_final_goal_reached = false;
          planner_utils.publish_end_APVI_mode();
          intermediate_points = intermediate_points_initial;
          leave_APVI = false;
          goal_selected = false;
          APVI = false;
          init_APVI_transition = true;
          utils.enable_transition = false;
          counter_slope = 0;
 
          //Place a tracker ffrom the projected position on the line to the user tracker. Exiting from the APVI mode can cause crashes
          
        }

        //bool for counter in case leave APVI is pressed 
        if (leave_APVI_counter)
        {
          APVI_exit_task_counter = APVI_exit_task_counter +1;
          leave_APVI_counter = false;
        }
        cout << "planner_utils.planner_final_goal_reached: " << planner_utils.planner_final_goal_reached << endl;
      }
      else if(safety_flag==false)
      {


       //Obtain the commanded position and velocity to forward to the controller
       pos_comm = Eigen::Vector3f(IC_x.obtain_commanded_position(),  IC_y.obtain_commanded_position(), des_z);
       vel_comm = Eigen::Vector3f(IC_x.obtain_commanded_velocity(),  IC_y.obtain_commanded_velocity(), z_vel_des);
       acc_comm = Eigen::Vector3f(0.0,  0.0,0.0);

      }


       //TO define all the switching case scenarios depending on the external input or if the goal has been reached 


      // //Exit if the user decide to leave the planner 
      
       planner_utils.publish_marker_projection_to_Unity(marker_proj_pos);
      
      //Pass data to log file class 
      pass_data_to_log_file_class();
      planner_utils.get_commanded_pose( pos_comm);
      //intermediate_points = visualizer.publish_intermediate_goals(intermediate_points, quad_pose);
      if (goal_selected == false){
          planner_utils.find_goal_position(APVI);
          goal_selected = true;
      }
      
      //Evaluate APVI Distances along x and y to send to the user 
      float x_apvi = planner_utils.d_output*(quad_pose(0) - kf_x.state_est.x );
      float y_apvi = planner_utils.d_output*(quad_pose(1) - kf_y.state_est.x );
      Eigen::Vector2f d_APVI = Eigen::Vector2f(x_apvi, y_apvi);
      Eigen::Vector2f d_APVI_dot = (d_APVI - d_APVI_old)/dt;
      sum_forces(p_distance, p_distance_dot, d_APVI, d_APVI_dot, dt);
      //publish marker visualization 
      visualizer.publishHapticInteractionMarker(marker_pos,quad_vel, haptic_device);
      visualizer.publishVirtualObstacles(scenario);
      planner_utils.final_goal_position(2) = quad_pose(2);
      visualizer.publishRRTFinalGoalMarker(&visualize_goals,quad_pose, planner_utils.final_goal_position);
      marker_pos_old = marker_pos;
      marker_vel_old = marker_vel;
      marker_proj_pos_old = marker_proj_pos;
      marker_proj_vel_old = marker_proj_vel;
     
      drone_line_distance_APVI = planner_utils.obtain_drone_line_distance();
      d_APVI_old = d_APVI;
     
    } 

    void AdmittanceCore::increasing_FPVI_counters()
    {
       //Increase FPVI Start counter 
       if (visualize_goals && increasing_FPVI_start_counter)
       {
           FPVI_start_task_counter = FPVI_start_task_counter +1;
           increasing_FPVI_start_counter = false;
       }
       else
       {
        if (visualize_goals == false)
        {
          increasing_FPVI_start_counter = true;
          counter_updated = false;
        }
       }

       if (leave_FPVI)
       {
        FPVI_exit_task_counter = FPVI_exit_task_counter + 1;
        leave_FPVI = false;
       }

       //Increase FPVI Goal Reach exit counter 
      if (visualize_goals)
      {
        float dx = planner_utils.final_goal_position(0) - quad_pose(0);
        float dy = planner_utils.final_goal_position(1) - quad_pose(1);
        float distance = sqrt(pow(dx,2) + pow(dy,2));
        if (distance < 0.2 && counter_updated==false)
        {
          FPVI_target_goal_counter = FPVI_target_goal_counter + 1;
          counter_updated = true;
          FPVI_final_goal_reached = 1;
          FPVI_int_goal1_reached = 0;
          FPVI_int_goal2_reached = 0;
        }

      }

      //Check if the intermediate goal in FPVI has been reached 
      float drone_int_goal_distance = 0;
      float dx = intermediate_points_initial[0](0) - quad_pose(0);
      float dy = intermediate_points_initial[0](1) - quad_pose(1);
      float distance_int1 = sqrt(pow(dx,2) + pow(dy,2));
      dx = intermediate_points_initial[1](0) - quad_pose(0);
      dy = intermediate_points_initial[1](1)  - quad_pose(1);
      float distance_int2 = sqrt(pow(dx,2) + pow(dy,2));
      
      if (distance_int1 < 0.3)
      {
         FPVI_int_goal1_reached = 1;
      }

      if (distance_int2 < 0.3)
      {
         FPVI_int_goal2_reached = 1;
      }
     
    }

   
    void AdmittanceCore::sum_forces(Eigen::Vector2f pd, Eigen::Vector2f pd_dot, Eigen::Vector2f d_APVI,  Eigen::Vector2f d_APVI_dot, float dt)
    {
        //change the value of the force depending the direction of the vector 
        //To verify if this conditions needs to be verified outside the APVI task
        // if (marker_pos(0) < quad_pose(0))
        // {
        //   F_admittance(0) = -1*F_admittance(0);
        // }
        // if (marker_pos(1) < quad_pose(1))
        // {
        //   F_admittance(1) = -1*F_admittance(1);
        // }
        
        // if (APVI)
        // {
        //    F_admittance(0) = -1* F_admittance(0);
        //    F_admittance(1) = -1* F_admittance(1);
        // }
        //Evaluate obstacle force numerical derivative 
        Eigen::Vector2f f_obs_vec_dot = Eigen::Vector2f(0.0, 0.0);
        Eigen::Vector2f f_obs_vec_xy =  Eigen::Vector2f(f_obs_vec_(0), f_obs_vec_(1));
        f_obs_vec_dot = (f_obs_vec_xy - f_obs_vec_old)/dt;
        Eigen::Matrix2f K_p_obst;
         K_p_obst << K_obstacles(0), 0.0,
                      0.0, K_obstacles(1);
        Eigen::Matrix2f K_d_obst;
        K_d_obst  <<  Kd_(0), 0.0,
                      0.0, Kd_(1);                    
        Eigen::Vector2f f_obst = K_p_obst*f_obs_vec_xy + K_d_obst*f_obs_vec_dot;
        //x and y positive the force is inverted druing the apvi task 
        float x_sum = 0.0;
        float y_sum = 0.0;

        //Check if the drone is outside the bordes to chage the dircetion of the force sent to the haptic device 
        F_admittance = utils.check_force_direction_outside_borders(F_admittance, quad_pose, marker_pos);

       
        if (APVI)
        {
          x_sum = F_admittance(0) + f_obs_vec_(0); //(K_obstacles(0)/3)*f_obs_vec_(0);
          y_sum = F_admittance(1) + f_obs_vec_(1); //(K_obstacles(1)/3)*f_obs_vec_(1);
        }
        else
        {
           x_sum = K_admittance(0)*F_admittance(0) + f_obst(0); //K_obstacles(0)*f_obs_vec_(0);
           y_sum = K_admittance(1)*F_admittance(1) + f_obst(1); //K_obstacles(1)*f_obs_vec_(1);
        }
        

        
        
        //Sum of the position related to the robot distance from the obstacle and the robot distance from the Path.
        //The sum of these two components give the final direction and Manitude of the Force vector generated by the computed distances (proportional)
        float x_sum_d = pd(0) + d_APVI(0); 
        float y_sum_d = pd(1) + d_APVI(1); 
        sum_d =  Eigen::Vector2f(x_sum_d, y_sum_d);

        //Sum of the velocities 
        float x_sum_d_dot = pd_dot(0) + d_APVI_dot(0); 
        float y_sum_d_dot = pd_dot(1) + d_APVI_dot(1); 
        sum_ddot =  Eigen::Vector2f(x_sum_d_dot, y_sum_d_dot);
       
        force_global = Eigen::Vector3f(x_sum, y_sum, 0.0);
        cout<< "x_sum: " << x_sum << "y_sum: " << y_sum << endl;


        f_obs_vec_old = f_obs_vec_xy;
    }


    void AdmittanceCore::pass_data_to_log_file_class()
    {
        log_files.get_des_pose(marker_pos);
        log_files.get_des_pose_projected(marker_proj_pos);
        log_files.get_quad_pose(quad_pose, quad_orientation);
        log_files.get_target_vel(marker_vel);
        log_files.get_quad_vel(quad_vel);
        log_files.get_Admittance_Force(F_admittance);
        log_files.get_commanded_position(pos_comm);
        log_files.get_commanded_velocity(vel_comm);
        log_files.get_commanded_kfx_state(kf_x.state_est);
        log_files.get_commanded_kf_userx_state(kf_user_x.state_est);
        log_files.get_commanded_kfy_state(kf_y.state_est);
        log_files.get_commanded_kf_usery_state(kf_user_y.state_est);
        log_files.get_target_user_projected_vel(marker_proj_vel);
        log_files.get_damping(planner_utils.d_output);
        log_files.get_obstacle_force_vector(f_obs_vec_);
        log_files.get_task_counters(FPVI_intermediate_target_counter, FPVI_target_goal_counter,APVI_target_goal_counter, FPVI_start_task_counter, APVI_start_task_counter, APVI_exit_task_counter, FPVI_exit_task_counter);
        log_files.get_drone_line_distance(drone_line_distance_APVI);
        log_files.get_target_position_FPVI(intermediate_points_initial[0], intermediate_points_initial[1], planner_utils.final_goal_position, FPVI_int_goal1_reached, FPVI_int_goal2_reached, FPVI_final_goal_reached);

        //Write files
        log_files.writing();



    }

    
    void AdmittanceCore::get_rviz_marker_pose(Eigen::Vector3f marker_position_)
    {
        rviz_marker_position = marker_position_;
    }

  

    void  AdmittanceCore::get_unity_marker_pose(Eigen::Vector3f marker_position_)
    {
      unity_marker_position = marker_position_;
    }

    void  AdmittanceCore::get_haptic_marker_pose(Eigen::Vector3f marker_position_)
    {
      haptic_marker_position = marker_position_;
    }


    void AdmittanceCore::get_quad_pose(Eigen::Vector3f quad_pose_, Eigen::Vector3f quad_orientation_)
    {
       quad_pose = quad_pose_;
       quad_orientation = quad_orientation_;
    }
    void AdmittanceCore::get_quad_vel(Eigen::Vector3f quad_vel_)
    {
       quad_vel = quad_vel_;
    }
    
 

}