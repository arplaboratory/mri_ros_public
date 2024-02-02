#include "drone_teleoperation/utils/ros_visualization.h"
#include <vector>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <numeric>

namespace hri_admittance {

  ROSVisualization::ROSVisualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
                            : view_history(false),
                            counter_clear_path(0),
                            max_counter_value(2),
                            time_to_clear_old_hist(30)
    
    
                       {
    float obs1_x, obs1_y, obs1_sx, obs1_sy; 
    float obs2_x, obs2_y, obs2_sx, obs2_sy; 
    float obs3_x, obs3_y, obs3_sx, obs3_sy; 

    nh_.getParam("/tele_control_params/visualization_frame_id", frame_id);
    nh_.getParam("/tele_control_params/visualize_force_history", view_history);
    nh_.getParam("/tele_control_params/clear_path_history_after_sec", time_to_clear_old_hist);
    nh_.getParam("/tele_control_params/obstacles/n_obstacle", n_obstacle);
    nh_.getParam("/tele_control_params/obstacles/obst_1_pos_and_dim/x", obs1_x);
    nh_.getParam("/tele_control_params/obstacles/obst_1_pos_and_dim/y", obs1_y);
    nh_.getParam("/tele_control_params/obstacles/obst_1_pos_and_dim/sx", obs1_sx);
    nh_.getParam("/tele_control_params/obstacles/obst_1_pos_and_dim/sy", obs1_sy);
    nh_.getParam("/tele_control_params/obstacles/obst_2_pos_and_dim/x", obs2_x);
    nh_.getParam("/tele_control_params/obstacles/obst_2_pos_and_dim/y", obs2_y);
    nh_.getParam("/tele_control_params/obstacles/obst_2_pos_and_dim/sx", obs2_sx);
    nh_.getParam("/tele_control_params/obstacles/obst_2_pos_and_dim/sy", obs2_sy);
    nh_.getParam("/tele_control_params/obstacles/obst_3_pos_and_dim/x", obs3_x);
    nh_.getParam("/tele_control_params/obstacles/obst_3_pos_and_dim/y", obs3_y);
    nh_.getParam("/tele_control_params/obstacles/obst_3_pos_and_dim/sx", obs3_sx);
    nh_.getParam("/tele_control_params/obstacles/obst_3_pos_and_dim/sy", obs3_sy);

    obst_1 = Eigen::Vector4f(obs1_x, obs1_y, obs1_sx, obs1_sy);
    obst_2 = Eigen::Vector4f(obs2_x, obs2_y, obs2_sx, obs2_sy);
    obst_3 = Eigen::Vector4f(obs3_x, obs3_y, obs3_sx, obs3_sy);
    obst_vector.push_back(obst_1);
    obst_vector.push_back(obst_2);
    obst_vector.push_back(obst_3);

    //Publisher 
    planner_drone_on_line_projection = nh_.advertise<visualization_msgs::Marker>("/rrt/marker_drone_line_proj", 2);
    planner_des_pose_on_line_projection = nh_.advertise<visualization_msgs::Marker>("/rrt/marker_des_pos_line_proj", 2);
    line_to_follow_marker = nh_.advertise<visualization_msgs::Marker>("/rrt/line_to_follow", 2);
    safety_boundaries_publisher =  nh_.advertise<visualization_msgs::Marker>("/safety_boundaries", 2);
    force_arrow_marker = nh_.advertise<visualization_msgs::Marker>("/visualization_marker/force_arrow", 2);
    publish_pose_history = nh_.advertise<nav_msgs::Path>("/visualization_marker/robot_path", 2);
    force_obs_arrow_marker_hist = nh_.advertise< geometry_msgs::PoseArray>("/PoseArray/force_obs_arrow_hist", 2);
    QuadrotorMarker = nh_.advertise<visualization_msgs::Marker>("/tele_control/robot", 2);
    final_goal_marker = nh_.advertise<visualization_msgs::Marker>("/rrt/final_goal_marker", 2);
    intermediate_points = nh_.advertise<visualization_msgs::Marker>("/path/intermediate_points", 2);
    intermediate_points_to_unity = nh_.advertise<scene_understanding_pkg_msgs::IntermediatePointsFPVI>("/to_unity/FPVI_intermediate_goals", 2);
    final_goal_publish_to_unity = nh_.advertise<geometry_msgs::Point>("/to_unity/FPVI_final_goal", 10);
    haptic_device_marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker/haptic_device_marker", 2);
    virtual_obstacles_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker/virtual_obstacles_marker", 2);
    
    mesh_directory = "package://mesh_visualization/mesh/hummingbird.mesh";
    }

  void ROSVisualization::publish_marker_proj_on_the_line(Eigen::Vector3f proj_position)
  {
    visualization_msgs::Marker projected_marker;
  
    projected_marker.header.frame_id = frame_id;
    projected_marker.header.stamp = ros::Time::now();
    projected_marker.ns = "drone_projected_marker";
    projected_marker.action  = visualization_msgs::Marker::ADD;
    projected_marker.pose.orientation.w = 1.0;
    projected_marker.id = 0; 
    projected_marker.type = visualization_msgs::Marker::SPHERE;
    projected_marker.lifetime = ros::Duration(0.2); 
    
    //obstacles.points.push_back(p);
    projected_marker.pose.position.x = proj_position(0);
    projected_marker.pose.position.y = proj_position(1);
    projected_marker.pose.position.z = proj_position(2);
    projected_marker.pose.orientation.x = 0.0;
    projected_marker.pose.orientation.y = 0.0;
    projected_marker.pose.orientation.z = 0.0;
    projected_marker.pose.orientation.w = 1.0;
    

    //projected_marker_to_unity = projected_marker.pose.position;
    //obstacles.scale.push_back(p);
    projected_marker.scale.x = 0.12;
    projected_marker.scale.y = 0.12;
    projected_marker.scale.z = 0.12;

    projected_marker.color.r = 1.0;
    projected_marker.color.g = 0.0;
    projected_marker.color.b = 0.0;
    projected_marker.color.a = 0.8;
    // if (keep_des_pos_fixed == true)
    // {
    //   projected_marker.color.a = 1.0;
    // }
    // else
    // {
    //    projected_marker.color.a = 0.05;
    // }
    planner_drone_on_line_projection.publish(projected_marker);

  }

  void ROSVisualization::publish_des_pos_marker_proj_on_the_line(Eigen::Vector3f proj_position)
  {
    visualization_msgs::Marker projected_marker;
  
    projected_marker.header.frame_id = frame_id;
    projected_marker.header.stamp = ros::Time::now();
    projected_marker.ns = "interactive_marker_projected_marker";
    projected_marker.action  = visualization_msgs::Marker::ADD;
    projected_marker.pose.orientation.w = 1.0;
    projected_marker.id = 0; 
    projected_marker.type = visualization_msgs::Marker::SPHERE;
    projected_marker.lifetime = ros::Duration(0.2);
    
    //obstacles.points.push_back(p);
    projected_marker.pose.position.x = proj_position(0);
    projected_marker.pose.position.y = proj_position(1);
    projected_marker.pose.position.z = proj_position(2);
    projected_marker.pose.orientation.x = 0.0;
    projected_marker.pose.orientation.y = 0.0;
    projected_marker.pose.orientation.z = 0.0;
    projected_marker.pose.orientation.w = 1.0;
    

    //projected_marker_to_unity = projected_marker.pose.position;
    //obstacles.scale.push_back(p);
    projected_marker.scale.x = 0.12;
    projected_marker.scale.y = 0.12;
    projected_marker.scale.z = 0.12;

    projected_marker.color.r = 1.0;
    projected_marker.color.g = 0.647;
    projected_marker.color.b = 0.0;
    projected_marker.color.a = 0.8;
    // if (keep_des_pos_fixed == true)
    // {
    //   projected_marker.color.a = 1.0;
    // }
    // else
    // {
    //    projected_marker.color.a = 0.05;
    // }
    planner_des_pose_on_line_projection.publish(projected_marker);

  }

  void ROSVisualization::publish_safety_boundaries_visualization(Eigen::Vector2f P1,Eigen::Vector2f P2 )
  {
    visualization_msgs::Marker safety_boundaries, point;
  safety_boundaries.header.frame_id = frame_id;
  safety_boundaries.header.stamp = ros::Time::now();
  safety_boundaries.ns = "safety_boundaries";
  safety_boundaries.action =  visualization_msgs::Marker::ADD;
  safety_boundaries.pose.orientation.w = 1.0;

  safety_boundaries.id = 1;
  safety_boundaries.type = visualization_msgs::Marker::LINE_STRIP;


  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  safety_boundaries.scale.x = 0.3;
 
  // Line strip is red
  safety_boundaries.color.r = 1.0;
  safety_boundaries.color.a = 1.0;
 
  safety_boundaries.lifetime = ros::Duration(0.2); 
   // Create the vertices for the points and lines
   geometry_msgs::Point P1_;
   P1_.x = P1(0);
   P1_.y = P1(1);

   geometry_msgs::Point P2_;
   P2_.x = P2(0);
   P2_.y = P2(1);

   geometry_msgs::Point P3_;
   P3_.x = P1(0);
   P3_.y = P2(1); 

   geometry_msgs::Point P4_;
   P4_.x = P2(0);
   P4_.y = P1(1); 

   //The sequence will be P1-P3, P3-P2, P2-P4, P4-P1
   safety_boundaries.points.push_back(P1_);
   safety_boundaries.points.push_back(P3_);
   safety_boundaries.points.push_back(P2_);
   safety_boundaries.points.push_back(P4_);
   safety_boundaries.points.push_back(P1_);

   safety_boundaries_publisher.publish(safety_boundaries);

  }

 vector<Eigen::Vector3f> ROSVisualization::publish_intermediate_goals( vector<Eigen::Vector3f> points, 
                                                  Eigen::Vector3f  quad_pose, int * FPVI_intermediate_target_counter){
   
    //Visualize intermediate goals only. The final goal is called by another function 
    geometry_msgs::Quaternion quat_orientation;
    quat_orientation.w = 1.0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
    marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
    marker.ns = ros::this_node::getName();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);
    
    //Check if the robot reaches one of the waypoint to make them disappear 
    vector<int> indexes;
    for (int i = 0; i < points.size(); i ++){
      Eigen::Vector3f point_ = points[i];
      Eigen::Vector3f diff = quad_pose - point_;
      float dist = sqrt(pow(diff(0),2) + pow(diff(1),2));
      if (dist < 0.15)
      {
        indexes.push_back(i);
        *FPVI_intermediate_target_counter = *FPVI_intermediate_target_counter +1;
       
      }
    }

    //Check which indexes to delete from the points array. If the vector of indexes is not empty
    //it means that one of the intermediate goals needs to be eliminated because reached by the robot

    for (int ii = 0; ii < indexes.size(); ii++)
    {
      points.erase(points.begin() + indexes[ii]);
    }
    
    scene_understanding_pkg_msgs::IntermediatePointsFPVI intermediate_points_v;
    for (int i = 0; i < points.size(); i ++)
    {
      geometry_msgs::Point waypoint;
      waypoint.x = points[i](0);
      waypoint.y = points[i](1);
      waypoint.z = quad_pose(2);
       marker.points.push_back(waypoint);
      intermediate_points_v.points.push_back(waypoint);

    }

   
    
    marker.pose.orientation = quat_orientation;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.2;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    intermediate_points.publish(marker);
    intermediate_points_to_unity.publish(intermediate_points_v);

    return points;
}
//Visualize line to follow in RVIZ
void ROSVisualization::publish_planner_line_to_follow_in_rviz(vector<Eigen::Vector3f> setpoints_vec)
{
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp  = ros::Time::now();
  points.ns = line_strip.ns = "line_to_follow";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.05;
 
  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  points.lifetime = ros::Duration(0.2); 
  line_strip.lifetime = ros::Duration(0.2); 
   // Create the vertices for the points and lines
   for (uint32_t i = 0; i < setpoints_vec.size(); ++i)
   {

     
    // if (i == 0)
    // {
    //   x = initial_position.x;
    //   y = initial_position.y;
    // }
    // else
    // {
    //   x = setpoint_x_vec[i];
    //   y = setpoint_y_vec[i];
    // }
    
    
     geometry_msgs::Point p;
     p.x = setpoints_vec[i](0);
     p.y = setpoints_vec[i](1);
     p.z = setpoints_vec[i](2);

      points.points.push_back(p);
      line_strip.points.push_back(p);
}
      line_to_follow_marker.publish(points);
      line_to_follow_marker.publish(line_strip);
     
}

void ROSVisualization::publish_arrow_force_in_rviz(Eigen::Vector3f user_marker, Eigen::Vector3f quad_pose)
  {
    visualization_msgs::Marker force_arrow;
    force_arrow.header.frame_id = frame_id;
    force_arrow.header.stamp = ros::Time::now();
    force_arrow.ns = "force_arrow";
    force_arrow.action = visualization_msgs::Marker::ADD;
    force_arrow.pose.orientation.w = 1.0;
    force_arrow.lifetime = ros::Duration(0.2);

    force_arrow.id = 0;
    force_arrow.type = visualization_msgs::Marker::ARROW;
    // POINTS markers use x and y scale for width/height respectively
    force_arrow.scale.x = 0.05;
    force_arrow.scale.y = 0.05;
   // Points are green
    force_arrow.color.r = 1.0f;
    force_arrow.color.a = 1.0;
    float x = 0.0;
    float y = 0.0;

    geometry_msgs::Point drone_position;
    geometry_msgs::Point user_position;
    drone_position.x = quad_pose(0);
    drone_position.y = quad_pose(1);
    drone_position.z = quad_pose(2);
    
    user_position.x = user_marker(0);
    user_position.y = user_marker(1);
    user_position.z = user_marker(2);

    force_arrow.points.push_back(drone_position);
    force_arrow.points.push_back(user_position);
    
    force_arrow_marker.publish(force_arrow);
    force_arrow.points.clear();
  }


// Adding the virtual obstacles visualization marker later


//Visualize path history and force arrow lists
void ROSVisualization::publish_path_history(Eigen::Vector3f quad_pose, double dt)
{
  //initialize the nav_msgs 
  geometry_msgs::Quaternion rot;
  rot.w = 1.0;
  
  msg_history.header.frame_id = frame_id;  
  msg_history.header.stamp = ros::Time::now();
  
  geometry_msgs::PoseStamped ps;
  geometry_msgs::Pose pose;
  pose.position.x = quad_pose(0);
  pose.position.y = quad_pose(1);
  pose.position.z = quad_pose(2);
  pose.orientation = rot;
  
  ps.pose = pose;
  ps.header.frame_id =  frame_id;
  double time = (double) msg_counter_history *dt;
  ps.header.stamp = ros::Time(time);
  msg_history.poses.push_back(ps);

  if (view_history)
  {
     publish_pose_history.publish(msg_history);
  }
  else
  {
      msg_history.poses.clear();
      msg_counter_history = 0;
  }


 if (counter_clear_path >= (max_counter_value) && view_history)
 {
    msg_history.poses.erase( msg_history.poses.begin(), msg_history.poses.begin() + ((msg_counter_history)/2));
    f_obs_poses.poses.erase(f_obs_poses.poses.begin(), f_obs_poses.poses.begin() + counter_clear_path);
    counter_clear_path = 0;
    msg_counter_history = 0;
 }

  msg_counter_history = msg_counter_history+1; // update in the force arrow history publisher with the counter_clear_path
}

void ROSVisualization::publish_QuadrotorMarker(Eigen::Vector3f quad_pose, Eigen::Vector4f quad_orientation)
{
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = quad_pose(0);
  marker.pose.position.y = quad_pose(1);
  marker.pose.position.z = quad_pose(2);
  marker.pose.orientation.x = quad_orientation(0);
  marker.pose.orientation.y = quad_orientation(1);
  marker.pose.orientation.z = quad_orientation(2);
  marker.pose.orientation.w = quad_orientation(3);
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


void ROSVisualization::publishRRTFinalGoalMarker(bool * visualize_waypoints,Eigen::Vector3f quad_pose, Eigen::Vector3f goal_position)
{
    //initialize the nav_msgs 
  geometry_msgs::Quaternion quat_orientation;
  quat_orientation.w = 1.0;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.2);
  marker.pose.position.x = goal_position(0);
  marker.pose.position.y = goal_position(1);
  marker.pose.position.z = goal_position(2);
  
  marker.pose.orientation = quat_orientation;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.2;
  marker.color.a = 0.7;
 
  
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  //Check if the flag visualize_waypoints is true. 
  //If true we are in the FPVI case and we want to make the waypoint disappear if the drone reach the final goal 
  //to conclude the task
  if (*visualize_waypoints){
    //check the distance of the robot from the final goal 
     Eigen::Vector3f diff = quad_pose - goal_position;
     float dist = sqrt(pow(diff(0),2) + pow(diff(1),2));
     if (dist < 0.15)
     {
        *visualize_waypoints = false;
        
     }
  }
  geometry_msgs::Point final_goal;
  final_goal.x = goal_position(0);
  final_goal.y = goal_position(1);
  final_goal.z = goal_position(2);

  //marker.mesh_resource = mesh_directory;
  final_goal_marker.publish(marker);
  final_goal_publish_to_unity.publish(final_goal);
}


void ROSVisualization::publishHapticInteractionMarker(Eigen::Vector3f marker_pose, Eigen::Vector3f quad_vel, bool haptic)
{
    //initialize the nav_msgs 
  geometry_msgs::Quaternion quat_orientation;
  quat_orientation.w = 1.0;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = ros::this_node::getName();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.2);
  marker.pose.position.x = marker_pose(0);
  marker.pose.position.y = marker_pose(1);
  marker.pose.position.z = marker_pose(2);
  
  marker.pose.orientation = quat_orientation;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;
  marker.color.a = 0.6;
 
  
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

   //change color depending on the magnitude of the drone velocity 
  float magn = sqrt(pow(quad_vel(0),2) + pow(quad_vel(1),2));
  
 
  if (magn >= 0 && magn < 0.15)
  {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.7;
  }
  else if(magn >= 0.15 && magn < 0.30){
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;
  }
  else
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;
  }


  //marker.mesh_resource = mesh_directory;
  if (haptic)
       haptic_device_marker_pub.publish(marker);

}


void ROSVisualization::publishVirtualObstacles(int scenario)
{
  //Define number of Obstacles
    visualization_msgs::MarkerArray markers;
    for (int i=0; i < n_obstacle; i++)
    {
      Eigen::Vector4f obst = obst_vector[i];
       geometry_msgs::Quaternion quat_orientation;
    quat_orientation.w = 1.0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id; //(new_frame_id == "") ? frame_id : new_frame_id;
    marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
    marker.ns = ros::this_node::getName();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);
    marker.pose.position.x = obst(0);  // 1.3; 
     marker.pose.position.y = obst(1);  //-0.5;
    marker.pose.position.z = 0.0; //marker_pose(2);
  
    marker.pose.orientation = quat_orientation;
    marker.scale.x = obst(2);
    marker.scale.y =  obst(3);
    marker.scale.z = 0.1;
    marker.color.a = 0.8;
 
  
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    markers.markers.push_back(marker);
    }

   
    if (scenario == 3 || scenario ==4)
        virtual_obstacles_marker_pub.publish(markers);
}
void ROSVisualization::publish_f_obs_history(Eigen::Vector3f quad_pose, Eigen::Vector3f force_obs_res, float force_obs_Magn, float time_to_pub, double dt)
{
    geometry_msgs::Quaternion rot;
    rot.w = 1.0;
    
    f_obs_poses.header.frame_id = frame_id;  
    f_obs_poses.header.stamp = ros::Time::now();
    //find angle for orientation 
    float yaw__ = 0.0;
    if (force_obs_Magn > 0)
    {
       yaw__ = acos(force_obs_res(0) / force_obs_Magn);
       if (isnan(yaw__) || isinf(yaw__))
       {
          yaw__ = M_PI;
       }
       if (force_obs_res(1) <= 0)
       {
         yaw__ = -1*yaw__;
       }
    }

    Eigen::Vector4f quat; 
    float roll = 0.0;
    float pitch = 0.0;
    euler_to_quat(&quat, roll, pitch, yaw__);
    
    rot.x = quat(0);
    rot.y = quat(1);
    rot.z = quat(2);
    rot.w = quat(3);

    geometry_msgs::Pose pose;
    pose.position.x = quad_pose(0);
    pose.position.y = quad_pose(1);
    pose.position.z = quad_pose(2);

    pose.orientation = rot;
    f_obs_poses.poses.push_back(pose);
  
    force_obs_arrow_marker_hist.publish(f_obs_poses);
    
    //set max counter value 
    max_counter_value = time_to_clear_old_hist/time_to_pub;
     
    counter_clear_path = counter_clear_path + 1;
   
}

void ROSVisualization::euler_to_quat(Eigen::Vector4f *quat, float roll, float pitch, float yaw_)
{
  float qx, qy, qz, qw;
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw_/2) - cos(roll/2) * sin(pitch/2) * sin(yaw_/2);
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw_/2) + sin(roll/2) * cos(pitch/2) * sin(yaw_/2);
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw_/2) - sin(roll/2) * sin(pitch/2) * cos(yaw_/2);
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw_/2) + sin(roll/2) * sin(pitch/2) * sin(yaw_/2);

  *quat << qx, qy, qz, qw;
}


}