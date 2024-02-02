#ifndef LOG_FILES_H_
#define LOG_FILES_H_
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <unistd.h>
#include <numeric>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Vector3Stamped.h>


/*
This class manages the tf transformations and the utilities functions used to manage geometrical relationship.
*/
#define C_PI (double)3.141592653589793
using namespace std;
namespace hri_admittance {
  
class LOGFilesWriting
{
    public:
    LOGFilesWriting( string path);
    virtual ~LOGFilesWriting(){}
    

    void get_des_pose(Eigen::Vector3f marker_pos);
    void get_des_pose_projected(Eigen::Vector3f marker_proj_pos);
    void get_quad_pose(Eigen::Vector3f quad_pose, Eigen::Vector3f quad_orientation);
    void get_target_vel(Eigen::Vector3f marker_vel);
    void get_quad_vel(Eigen::Vector3f quad_vel);
    void get_Admittance_Force(Eigen::Vector3f F_admittance);
    void get_commanded_position(Eigen::Vector3f pos_comm);
    void get_commanded_velocity(Eigen::Vector3f vel_comm);
    void get_commanded_kfx_state(geometry_msgs::Vector3 kf_x_state);
    void get_commanded_kf_userx_state(geometry_msgs::Vector3 kf_user_x_state);
    void get_commanded_kfy_state(geometry_msgs::Vector3 kf_y_state);
    void get_commanded_kf_usery_state(geometry_msgs::Vector3 kf_user_y_state);
    void get_target_user_projected_vel(Eigen::Vector3f marker_proj_vel);
    void get_damping(float d);
    void get_obstacle_force_vector(Eigen::Vector3f f_vec);
    void get_task_counters(int FPVI_int_goals, int FPVI_final_goal, int APVI_final_goal, int FPVI_start, int APVI_start, int APVI_end, int FPVI_end);
    void get_drone_line_distance(float distance);
    void get_target_position_FPVI(Eigen::Vector3f FPVI_int_goal1_pos, Eigen::Vector3f FPVI_int_goal2_pos,  Eigen::Vector3f FPVI_final_goal_pos, int FPVI_int_goal1_reached, int FPVI_int_goal2_reached, int FPVI_final_goal_reached);
    void writing();

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    string folder_name;
    string stringpath;
    std:: ofstream ofs;
    std::ofstream ofs1;
    std::ofstream ofs2;
    std::ofstream ofs3;
    std::ofstream ofs4;
    std::ofstream ofs5;
    std::ofstream ofs6;
    std::ofstream ofs7;
    std::ofstream ofs8;
    std::ofstream ofs9;
    std::ofstream ofs10;
    std::ofstream ofs11;
    std::ofstream ofs12;
    std::ofstream ofs13;
    std::ofstream ofs14;
    std::ofstream ofs15;
    std::ofstream ofs16;
    std::ofstream ofs17;
    std::ofstream ofs18;
    std::ofstream ofs19;
    std::ofstream ofs20;
    std::ofstream ofs21;
    std::ofstream ofs22;
    std::ofstream ofs23;
    std::ofstream ofs24;
    std::ofstream ofs25;
    std::ofstream ofs26;
    std::ofstream ofs27;
    std::ofstream ofs28;
    std::ofstream ofs29; 
    std::ofstream ofs30;
    std::ofstream ofs31; 
    std::ofstream ofs32;
    std::ofstream ofs33; 
    std::ofstream ofs34;
    std::ofstream ofs35; 
    std::ofstream ofs36;
    std::ofstream ofs37;
    std::ofstream ofs38;
    std::ofstream ofs39;
    std::ofstream ofs40;
    std::ofstream ofs41;
    std::ofstream ofs42;
    std::ofstream ofs43;
    std::ofstream ofs44;

    Eigen::Vector3f marker_pos;
    Eigen::Vector3f marker_proj_pos;
    Eigen::Vector3f quad_pose;
    Eigen::Vector3f quad_orientation;
    Eigen::Vector3f marker_vel;
    Eigen::Vector3f quad_vel;
    Eigen::Vector3f F_admittance;
    Eigen::Vector3f pos_comm;
    Eigen::Vector3f vel_comm;
    geometry_msgs::Vector3 kf_x_state;
    geometry_msgs::Vector3 kf_user_x_state;
    geometry_msgs::Vector3 kf_y_state;
    geometry_msgs::Vector3 kf_user_y_state;
    Eigen::Vector3f marker_proj_vel;
    Eigen::Vector3f f_vec;
    float d;
    float distance_; 

    //counter related variables 
    int FPVI_int_goals_ = 0; 
    int FPVI_final_goal_ = 0; 
    int APVI_final_goal_ = 0; 
    int FPVI_start_ = 0;
    int APVI_start_ = 0;
    int FPVI_end_ = 0;
    int APVI_end_ = 0;

    //target related variables 
    float FPVI_int_goal1_pos_x_ = 0;
    float FPVI_int_goal1_pos_y_ = 0;
    float FPVI_int_goal2_pos_x_ = 0;
    float FPVI_int_goal2_pos_y_ = 0;
    float FPVI_final_goal_pos_x_ = 0;
    float FPVI_final_goal_pos_y_ = 0;
    int FPVI_int_goal1_reached_ = 0; //1 if reached, define the order how the number change from zero to one
    int FPVI_int_goal2_reached_ = 0;
    int FPVI_final_goal_reached_ = 0;
      

};

}

#endif