#ifndef ROS_VISUALIZATION_H_
#define ROS_VISUALIZATION_H_
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <tf/transform_broadcaster.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <scene_understanding_pkg_msgs/RRTPathPoints2D.h>
#include <scene_understanding_pkg_msgs/RRTObstaclesCoo.h>
#include <scene_understanding_pkg_msgs/IntermediatePointsFPVI.h>

/*
This class manages the tf transformations and the utilities functions used to manage geometrical relationship.
*/
#define C_PI (double)3.141592653589793
using namespace std;
namespace hri_admittance {
  
class ROSVisualization
{
    public:
    ROSVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ROSVisualization(){}
    void publish_marker_proj_on_the_line(Eigen::Vector3f proj_position);
    void publish_des_pos_marker_proj_on_the_line(Eigen::Vector3f proj_position);
    void publish_planner_line_to_follow_in_rviz(vector<Eigen::Vector3f> setpoints_vec);
    void publishHapticInteractionMarker(Eigen::Vector3f marker_pose,Eigen::Vector3f quad_vel, bool haptic);
    void publish_safety_boundaries_visualization(Eigen::Vector2f P1,Eigen::Vector2f P2 );
    void publish_arrow_force_in_rviz(Eigen::Vector3f quad_pose, Eigen::Vector3f user_marker);
    vector<Eigen::Vector3f>  publish_intermediate_goals(vector<Eigen::Vector3f> points, Eigen::Vector3f  quad_pose, int * FPVI_intermediate_target_counter);
    void publish_path_history(Eigen::Vector3f quad_pose, double dt);
    void publish_f_obs_history(Eigen::Vector3f quad_pose, Eigen::Vector3f force_obs_res, float force_obs_Magn, float time_to_pub, double dt);
    void publish_QuadrotorMarker(Eigen::Vector3f quad_pose, Eigen::Vector4f quad_orientation);
    void publishRRTFinalGoalMarker(bool * visualize_points, Eigen::Vector3f quad_pose, Eigen::Vector3f goal_position);
    void euler_to_quat(Eigen::Vector4f *quat, float roll, float pitch, float yaw);
    void publishVirtualObstacles(int scenario);



    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher planner_drone_on_line_projection; 
    ros::Publisher planner_des_pose_on_line_projection;
    ros::Publisher line_to_follow_marker;
    ros::Publisher safety_boundaries_publisher;
    ros::Publisher force_arrow_marker;
    ros::Publisher publish_pose_history;
    ros::Publisher force_obs_arrow_marker_hist;
    ros::Publisher QuadrotorMarker;
    ros::Publisher final_goal_marker;
    ros::Publisher intermediate_points;
    ros::Publisher intermediate_points_to_unity;
    ros::Publisher final_goal_publish_to_unity;
    ros::Publisher haptic_device_marker_pub;
    ros::Publisher virtual_obstacles_marker_pub;
    
    nav_msgs::Path msg_history;
    geometry_msgs::PoseArray f_obs_poses;
    int time_to_clear_old_hist;
    int msg_counter_history = 0;
    int counter_clear_path;
    int max_counter_value;
    int n_obstacle = 3;
    Eigen::Vector4f obst_1;
    Eigen::Vector4f obst_2;
    Eigen::Vector4f obst_3;
    vector<Eigen::Vector4f> obst_vector;

    string frame_id;
    string mesh_directory; 
    bool view_history;
};
}
#endif