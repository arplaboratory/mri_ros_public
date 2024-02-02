#ifndef _PERCEPTION_SERVER_H
#define _PERCEPTION_SERVER_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include "scene_understanding_pkg_msgs/MeshVertexPosition.h"
#include "scene_understanding_pkg_msgs/ObstacleRepForce.h"
#include <scene_understanding_pkg_msgs/RRTObstaclesCoo.h>


#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>

// Image conversion from RosMsgs to OpenCV
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_msgs/Vertices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/filter.h>
#include <shape_msgs/Mesh.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <voxblox_msgs/Mesh.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <voxblox/mesh/mesh_utils.h>
#include "scene_understanding_pkg_msgs/waypointArray.h"
#include "scene_understanding_pkg_msgs/waypointMsg.h"

#include "perception_utils.h"


using namespace std;
namespace perception {
namespace enc = sensor_msgs::image_encodings;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  

class PerceptionServer
{
  public:
  PerceptionServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~PerceptionServer() {}
  
  //Subscriber Callback Declaration 
  void getServerConfigFromRosParam(const ros::NodeHandle& nh_private); //obtain RosParam 
  void voxblox_mesh_callback(const voxblox_msgs::Mesh::ConstPtr &msg);
  void voxblox_surface_cloud_callback(const sensor_msgs::PointCloud2 &input);
  void quadrotor_pose_callback(const nav_msgs::Odometry &msg);
  void cloud_in_callback(const sensor_msgs::PointCloud2 &input);
  void from_unity_start_stop_mapping_callback(const std_msgs::Bool &msg);
  void mocap_scene_root_in_unity_frame_callback(const geometry_msgs::PoseStamped &msg);
  void holo_position_callback(const geometry_msgs::Pose &msg);
  void virtual_obs_callback(const scene_understanding_pkg_msgs::RRTObstaclesCoo &msg);
  void from_unity_reset_map_callback(const std_msgs::Bool &msg);
  void planner_start_assistive_guidance_callback(const std_msgs::Bool flag);
  void evaluate_obstacle_force_field();
  




  protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber rs_RGB;
  ros::Subscriber cloud_in;
  ros::Subscriber quadrotor_pose;
  //From Unity
  ros::Subscriber start_stop_mapping;
  ros::Subscriber restart_mapping;

  // voxblox sub
  ros::Subscriber voxblox_mesh;
  ros::Subscriber voxblox_pc_surface;

  ros::Subscriber mocap_scene_root_in_unity_world;
  ros::Subscriber holoPose;
  ros::Subscriber virtual_obstacles;
  ros::Subscriber enable_assistive_mode;

  ros::Publisher triangles_mesh;
  ros::Publisher marker_pub_force_arrow_rviz;
  //Publish Mesh arrays to unity
  ros::Publisher mesh_to_unity;
  ros::Publisher cloud_in_rep; //rep pc points depending if the user wants to start or stop tye map
  ros::Publisher pub_occupied_pc_to_unity;
  ros::Publisher mesh_pointcloud_to_unity;
  ros::Publisher holo_position_rviz;
  ros::Publisher obst_force;


  ros::ServiceClient clear_map_service;
  tf::TransformBroadcaster voxl3_odom_broadcaster;
  tf::TransformBroadcaster from_mocap_to_world;
  tf::TransformBroadcaster mocap_to_mocap_sr_broadcaster;
  tf::TransformBroadcaster mocap_to_sim_broadcaster;
  tf::TransformBroadcaster unity_mocap_to_mocap_sr_broadcaster;

    /**
    Manage the Ros transformation between frames
   */

 
  PerceptionUtils perception_utils;
  //Timers 
  ros::Timer publish_mesh_timer_;
  ros::Timer publish_occupied_pc_to_unity;

  //Functions 
  void initialize_visualization_messages();
  void publish_hololens_pose_in_RVIZ();
    /// Incremental update.
  void publishMesh(const ros::TimerEvent& event);
  void publishOccupiedPC_to_unity(const ros::TimerEvent& event);
  void publish_mesh_to_unity(visualization_msgs::Marker triangles);
  void publish_obs_force_array_marker(float theta);
  void publish_obstacle_force_resultant(float fx, float fy, float Fm);
  void publish_mocap_sim_tf();

   //Variables 
   bool in_simulation;
   bool horizon_force_perception;
   float lamda;
   float Fs;
   int avg_stamp;
   int mesh_publish_period;

   //Mapping
   bool start_mapping = true;
   bool APVI = false;

   visualization_msgs::Marker triangles, obs_magn_force_b_frame;
   vector<geometry_msgs::Point> voxblox_mesh_vertices;
   //Voxblox Pc Surface pointcloud 
   pcl::PointCloud<pcl::PointXYZ> pc_surface_;
   pcl::PointCloud<pcl::PointXYZ> pc_to_unity;
   pcl::PointCloud<pcl::PointXYZ>  pc_to_unity_new;
   string frame_pc;

   //Unity MSR Frame 
   geometry_msgs::Point mocap_sr_unity_world_position;
   geometry_msgs::Vector3 mocap_sr_euler_angles_unity_world;
   geometry_msgs::Quaternion mocap_sr_orientation_unity_world;
   
    //Hololens position respect world frame 
    geometry_msgs::Point hololens_position;
    geometry_msgs::Quaternion hololens_orientation;


   //Robot params 
    geometry_msgs::Point quad_position;
    geometry_msgs::Point quad_euler_orientation;
    geometry_msgs::Quaternion quad_quat_orientation;

   //Obstacle Force Field variables 
    vector<float> fx_vec;
    vector<float> fy_vec; 
    vector<float> distance_vec;
    float theta_res = 0.0;
    float F_magn = 0.0;
    float theta_min_distance_old = 0.0;
    float theta_min_distance_ = 0.0;
    vector<geometry_msgs::Point> virtual_obstacles_vec;

   string mesh_vis_msgs_frame = "map";
   string force_vis_msgs_frame = "mocap"; 

};

}

#endif