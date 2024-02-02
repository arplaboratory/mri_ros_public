#ifndef PLANNER_UTILS_H_
#define PLANNER_UTILS_H_
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <tf/transform_broadcaster.h> 
#include <random>

#include <scene_understanding_pkg_msgs/RRTPathPoints2D.h>
#include <scene_understanding_pkg_msgs/RRTObstaclesCoo.h>
#include "drone_teleoperation/utils/ros_visualization.h"

/*
This class manages the tf transformations and the utilities functions used to manage geometrical relationship.
*/
#define C_PI (double)3.141592653589793
using namespace std;
namespace hri_admittance {
  
class PlannerUtils
{
    public:
    PlannerUtils(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~PlannerUtils(){}
    //Planner related callbacks 
    void planner_local_path_callback(const scene_understanding_pkg_msgs::RRTPathPoints2D points);
    void planner_start_assistive_guidance_callback(const std_msgs::Bool flag);
    void planner_final_goal_reached_callback(const std_msgs::Bool flag);
    void planner_final_goal_position_callbak(const geometry_msgs::Point final_goal_msg);
    
    
    //Planner Publishers
    void publish_path2D_to_unity();
    void publish_end_APVI_mode();
    void publish_marker_projection_to_Unity(Eigen::Vector3f point);
    void publish_pause_planner(bool flag);
    
    void project_drone_position_on_segment();
    void project_des_position_on_segment();
    void evaluate_variable_damping_coefficient();
    void fill_mission_setpoints_array();
    float average_angle_gamma(double gamma);
    void find_goal_position(bool APVI);
    void clear_arrays();
    Eigen::Vector3f evaluate_projection_on_line(Eigen::Vector3f agent, float angle, float y_agent_on_line, float d2, float intercept, float pl_d, bool drone_case);
    Eigen::Vector3f interpolate_projection_marker_on_line(bool * flag_in_int,  Eigen::Vector3f proj);
    Eigen::Vector3f interpolate_drone_position_with_user_interaction_marker(bool * flag, Eigen::Vector3f marker_pose, Eigen::Vector3f actual_proj);
    vector<Eigen::Vector3f> generate_random_intermediate_points_location();
    
    void get_quad_pose(Eigen::Vector3f quad_pose_, Eigen::Vector3f quad_orientation_);
    void get_des_pose(Eigen::Vector3f des_pose_);
    void get_commanded_pose(Eigen::Vector3f xc_pose_);
    float obtain_drone_line_distance();
    // float check_des_yaw(bool *increasing_yaw, bool *decreasing_yaw, float *yaw, float d_yaw);
    // double average(std::vector<float>  *v);
    // std::string check_frames(int check_frames);
    
    //Store the points from the rrt path
    vector<Eigen::Vector3f> planner_path_points;  
    Eigen::Vector3f marker_projected_position_on_line; //final evaluated desired position for the drone on the line
    Eigen::Vector3f last_marker_projected_position_on_line;
    Eigen::Vector3f final_goal_position;
    float d_output = 0.0; //variable damping output
    float gamma_output;
    bool planner_start_assistive_guidance;
    bool planner_final_goal_reached;


    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    //Add Subscriber and Publisher declaration
    ros::Subscriber planner_path_2D;
    ros::Subscriber enable_assistive_mode;
    ros::Subscriber planner_goal_reached;
    ros::Subscriber planner_final_goal;

    ros::Publisher path2D_to_unity;
    ros::Publisher end_APVI_mode;
    ros::Publisher planner_marker_projection_unity;
    ros::Publisher planner_paused;

    std::random_device rd;
    // Functions declaration
    
    ROSVisualization visualizer;

    //Quad Params
    Eigen::Vector3f quad_pose;
    Eigen::Vector3f quad_orientation;
    Eigen::Vector3f quad_vel;

    //Des Pose
    Eigen::Vector3f des_pose;
    Eigen::Vector3f xc_pose;
    
    //variable Damping Params
    double D_MIN = 1.0;
    double D_MAX = 30;
    string damping_function;

    //Variables 
    int setpoint_index_counter = 0;
    int step = 0;
    int scenario =1;
    float point_line_distance;

    //Line Paraeters 
    float m_l1, intercept;
    float y_drone_on_line, y_marker_on_line;
    //float y_drone_projection_on_line_, y_marker_projection_on_line;
    float drone_marker_distance;
    float distance_marker_to_next_setpoint;
    float distance_marker_to_prev_setpoint;
    float step_sum_int;
    
    //drone line distance 
    float d_l_distance;
    vector<float> setpoint_x_vec;
    vector<float> setpoint_y_vec;
    vector<double> gamma_vector;
    //Save the value of the previously received rrt horizon gial point
    Eigen::Vector3f old_horizon_goal;
    Eigen::Vector3f setpoint;
    Eigen::Vector3f setpoint_old;
    Eigen::Vector3f drone_projected_position_along_the_line;
    Eigen::Vector3f marker_position_proj_on_line;
    
    Eigen::Vector3f P1_final_goal_position;
   
    
    //vector<Eigen::Vector3f> planner_setpoints;
    
    bool keep_des_pos_fixed;
    bool inside_manouver;
    bool under_unity_manipulation;
    bool path_received = false;


};

}
#endif


