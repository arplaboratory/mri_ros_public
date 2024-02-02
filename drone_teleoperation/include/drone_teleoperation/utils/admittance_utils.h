#ifndef UTILS_H_
#define UTILS_H_
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <tf/transform_broadcaster.h> 
#include "drone_teleoperation/utils/ros_visualization.h"

/*
This class manages the tf transformations and the utilities functions used to manage geometrical relationship.
*/
using namespace std;
namespace hri_admittance {
  
class AdmittanceUtils
{
    public:
    AdmittanceUtils(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~AdmittanceUtils(){}

    void publish_tf_transform_from_world_to_sim();
    float check_des_yaw(bool increasing_yaw, bool decreasing_yaw, float yaw, float d_yaw);
    bool check_safety_boundaries(Eigen::Vector3f quad_pos);
    float yaw_tracker(float yaw, float yaw_des, bool * yaw_flag, float dt);
    double average(std::vector<float>  *v);
    void line_tracker(Eigen::Vector3f quad_pos,  float dt);
    void safety_admittance_desired_position_transition();
    Eigen::Vector3f check_force_direction_outside_borders(Eigen::Vector3f F_admittance, Eigen::Vector3f  quad_pos,Eigen::Vector3f marker_pos);

    std::string check_frames(int check_frames);
   
    Eigen::Vector3f curr_des_pos;
    Eigen::Vector3f curr_des_vel;
    Eigen::Vector3f curr_des_acc;
    Eigen::Vector3f goal_pose_tracker;

    Eigen::Vector3f start_pos_for_transition;
    Eigen::Vector3f final_position_for_transition;
    Eigen::Vector3f interpolation_updated_position;


    bool traj_track_start=false;
    bool trajectory_ended = false;
    bool tracker_in_action = false;
    bool enable_transition = false;
 

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    tf::TransformBroadcaster voxl_odom_broadcaster; 
    tf::TransformBroadcaster world_to_map_tf;
    tf::TransformBroadcaster map_to_mocap_tf;
    tf::TransformBroadcaster mocap_to_mocap_sr_tf;
    tf::TransformBroadcaster mocap_sr_to_simulator_tf;
    
    ROSVisualization visualizer;

    //chack yaw function var
    int counter_yaw_rate = 0;
    float yaw_drone_initial = 0.0;
    float final_angle = 0.0;
    float sum_x = 0.0;
    float sum_y = 0.0;
    int counter_transition =0;

    //Yaw tracker 
    bool yaw_traj_track_start = false;
    bool yaw_traj_track_end = false;
    float yaw_des_init;
    float yaw_des_tracker;
    float yaw_init;
    bool increase = false;
    float yaw_offset = 0.0;

    float current_traj_duration_, current_traj_length_;
    Eigen::Vector2f P1, P2;
    Eigen::Vector3f pos_;
    Eigen::Vector3f start_;
    bool goal_reached_;
    
    

};

}
#endif


