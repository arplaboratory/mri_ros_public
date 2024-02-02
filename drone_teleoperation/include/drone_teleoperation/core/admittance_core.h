#ifndef CORE_H_
#define CORE_H_

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <numeric>
#include <unistd.h>
#include <math.h>
#include "drone_teleoperation/core/Admittance_Controller.h"
#include "drone_teleoperation/utils/admittance_utils.h"
#include "drone_teleoperation/core/KF.h"
#include "drone_teleoperation/utils/planner_utils.h"
#include "drone_teleoperation/utils/ros_visualization.h"
#include "drone_teleoperation/utils/log_files_writing.h"

using namespace std;
namespace hri_admittance {
class AdmittanceCore
{
    public:
    AdmittanceCore(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private, string path);
    virtual ~AdmittanceCore(){}

    void free_admittance_interaction(bool *init, int scenario, Eigen::Vector3f f_obs_vec, float f_obs_magn, Eigen::Vector2f p_d, Eigen::Vector2f p_d_dot,float dt);
    void assisted_admittance_interaction(bool *init, int scenario, Eigen::Vector3f f_obs_vec, float f_obs_magn, Eigen::Vector2f p_d, Eigen::Vector2f p_d_dot, float dt);
    void background_kf_user_update(float dt);
    void evaluate_target_position_derivatives(int scenario, float dt);
    void smoothing_target_traj(int scenario);
    void average_obs_force(float *fx, float *fy, int avg_stamps);
    void initialize_KF();
    void safety_interpolation_check(float th, float dt);
    void pass_data_to_log_file_class();
    void increasing_FPVI_counters();
    void sum_forces( Eigen::Vector2f p_distance,  Eigen::Vector2f p_distance_dot,  Eigen::Vector2f d_APVI,  Eigen::Vector2f d_APVI_dot, float dt);
    
    void get_rviz_marker_pose(Eigen::Vector3f marker_position_);
    void get_unity_marker_pose(Eigen::Vector3f marker_position_);
    void get_haptic_marker_pose(Eigen::Vector3f marker_position_);
    void get_quad_pose(Eigen::Vector3f quad_pose_, Eigen::Vector3f quad_orientation_);
    void get_quad_vel(Eigen::Vector3f quad_vel_);


    
    //Admittance Controller commanded output traj
    Eigen::Vector3f pos_comm;
    Eigen::Vector3f vel_comm;
    Eigen::Vector3f acc_comm;
    float des_z = 0.95;
    float des_yaw = 0.0;
    float yaw_value = 0.0;

    //For Haptic 
    Eigen::Vector3f force_global;
     Eigen::Vector2f sum_d;
     Eigen::Vector2f sum_ddot;

    int FPVI_intermediate_target_counter = 0;
    int FPVI_target_goal_counter = 0;
    int APVI_target_goal_counter = 0;
    int APVI_start_task_counter = 0;
    int FPVI_start_task_counter = 0;
    int APVI_exit_task_counter = 0;
    int FPVI_exit_task_counter = 0;

    

    bool APVI = false; // InFPVI the case is zeros
    bool leave_APVI = false;
    bool leave_APVI_counter = false;
    bool leave_FPVI = false;
    bool hovering = false;
    bool visualize_goals = false;
    bool null_tracker_trasition_success = false;
    bool increasing_FPVI_start_counter=true;
    bool counter_updated=false;
    bool counter_APVI_updated = false;
    bool change_scenario = false;
    bool haptic_device = false;
    bool change_yaw180 = false;
    bool change_yaw_active = false;

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    AdmittanceUtils utils;
    Admittance_Controller IC_x;
    Admittance_Controller IC_y;
    LOGFilesWriting log_files;
    KF kf_x;
    KF kf_y;
    KF kf_user_x;
    KF kf_user_y;

    PlannerUtils planner_utils;
    ROSVisualization visualizer;
    
    //Quad Params
    Eigen::Vector3f quad_pose;
    Eigen::Vector3f quad_orientation;
    Eigen::Vector3f quad_vel;

     //Controller Params
    int avg_stamps = 20; 
    string path;
    string folder_name;
    
    float slope = 1;
    int counter_slope = 0;
    //Utils 
    vector<float> target_pos_x_vec;
    vector<float> target_pos_y_vec;
    vector<float> target_pos_x_l1_planner;
    vector<float> target_pos_y_l1_planner;
    vector<Eigen::Vector3f> intermediate_points;
    vector<Eigen::Vector3f> intermediate_points_initial;

    Eigen::Vector3f rviz_marker_position;
    Eigen::Vector3f unity_marker_position;
    Eigen::Vector3f haptic_marker_position;
    Eigen::Vector3f marker_pos; //This is the Rviz or Unity Holo marker position 
    Eigen::Vector3f marker_pos_old;
    Eigen::Vector3f marker_vel;
    Eigen::Vector3f marker_vel_old;
    Eigen::Vector3f marker_acc;
    Eigen::Vector3f marker_pos_safety;

    Eigen::Vector3f marker_proj_pos;
    Eigen::Vector3f marker_proj_pos_old;
    Eigen::Vector3f marker_proj_vel;
    Eigen::Vector3f marker_proj_vel_old;
    Eigen::Vector3f marker_proj_acc;
    Eigen::Vector3f f_obs_vec_;


    Eigen::Vector3f des_pos_for_transition;
    Eigen::Vector3f des_vel_for_transition;
    Eigen::Vector3f des_acc_for_transition;

    Eigen::Vector3f last_quad_position;
    Eigen::Vector2f d_APVI_old;

    //dmittance Force 
    Eigen::Vector3f F_admittance;
    vector<float> fx_obs_vec;
    vector<float> fy_obs_vec;

    //Haptic feedback force amplification gains 
    Eigen::Vector2f K_admittance;
    Eigen::Vector2f K_obstacles;
    Eigen::Vector2f f_obs_vec_old;
    Eigen::Vector2f Kd_;
    
    //saving variables 
    int FPVI_final_goal_reached = 0;
    int FPVI_int_goal1_reached = 0;
    int FPVI_int_goal2_reached = 0;
    float drone_line_distance_APVI = 0;
    int counter_init = 0;
    int counter_safety = 0;
    bool go_to_hover = false;
    float safety_d_th = 0.10;
    bool safety_flag=false;
    bool safety_flag_case_1 = false;
    bool safety_flag_case_2 = false;
    bool goal_selected=false;
    bool init_APVI_transition = false;
    bool random_intermediate_p = false;

    
   

};
}

#endif