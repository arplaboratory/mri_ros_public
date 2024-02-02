#include "drone_teleoperation/utils/admittance_utils.h"
#include "drone_teleoperation/utils/planner_utils.h"
#include <vector>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <numeric>

namespace hri_admittance {

  PlannerUtils::PlannerUtils(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
                             : nh_(nh),
                               nh_private_(nh_private),
                               visualizer(nh, nh_private),
                               keep_des_pos_fixed(false),
                               inside_manouver(false),
                               D_MIN(0), 
                               D_MAX(30),
                               planner_final_goal_reached(false) {
    
    float P1_final_goal_x, P1_final_goal_y;
    
    nh_.getParam("/tele_control_params/admittance_gains/D_MIN", D_MIN);
    nh_.getParam("/tele_control_params/admittance_gains/D_MAX", D_MAX);
    nh_.getParam("/tele_control_params/admittance_gains/damping_func", damping_function);
    nh_.getParam("/tele_control_params/scenario", scenario);
    if (scenario==1 || scenario==2){
       nh_.getParam("/tele_control_params/final_goal_sim/P1/x", P1_final_goal_x);
       nh_.getParam("/tele_control_params/final_goal_sim/P1/y", P1_final_goal_y);
    }
    else{
       nh_.getParam("/tele_control_params/final_goal/P1/x", P1_final_goal_x);
       nh_.getParam("/tele_control_params/final_goal/P1/y", P1_final_goal_y);
    }
    


    
    //Subscriber Declaration 
    planner_path_2D =  nh_.subscribe("rrt/path", 1, &PlannerUtils::planner_local_path_callback, this);
    enable_assistive_mode = nh_.subscribe("/rrt/start_assistive_guidance", 1, &PlannerUtils::planner_start_assistive_guidance_callback, this);
    planner_goal_reached = nh_.subscribe("/rrt/final_goal_reached", 1, &PlannerUtils::planner_final_goal_reached_callback, this);
    // planner_final_goal = nh_.subscribe("/rrt/final_goal", 10, &PlannerUtils::planner_final_goal_position_callbak, this);

    //Publisher 
    path2D_to_unity = nh_.advertise<scene_understanding_pkg_msgs::RRTPathPoints2D>("/rrt/path2D_to_unity", 10);
    end_APVI_mode = nh_.advertise<std_msgs::Bool>("/rrt/exit_search_mode", 10);
    planner_marker_projection_unity =   nh_.advertise<geometry_msgs::Point>("/rrt/fxed_marker_for_manouver_to_unity", 10);
    planner_paused = nh_.advertise<std_msgs::Bool>("/rrt/planner_paused", 10);

    P1_final_goal_position = Eigen::Vector3f(P1_final_goal_x, P1_final_goal_y, 0.8);


    }


float evaluate_distance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
  float a = pow(point2.x - point1.x, 2);
  float b = pow(point2.y - point1.y, 2);
  float distance = 0.0;
  return distance = sqrt(a + b);
}


void PlannerUtils::planner_local_path_callback(const scene_understanding_pkg_msgs::RRTPathPoints2D points)
{

    
  planner_path_points.clear();
  int counter = 1;
  float distance_new = 0.0;
  
  //Iterate acroos each couple of waypoints to evaluate their distance.
  for (int ii = 0; ii < points.point.size(); ii++)
  {
    counter = ii - 1;
    if (counter <= points.point.size())
    {
      distance_new = evaluate_distance(points.point[ii], points.point[counter]); // points.point[counter]
      if (distance_new == 0)
      {
        continue;
      }
    }
  
    //Start the planner from the robot actual position 
    if (ii == 0 )
    {
      planner_path_points.push_back(xc_pose); //quad_pose
      
    }
    else
    {
      Eigen::Vector3f points_ = Eigen::Vector3f(points.point[ii].x, points.point[ii].y, 0.9);
      planner_path_points.push_back(points_);
    }

    cout << "[Planner Points] planner_path_points[ii]: " << planner_path_points[ii] << " ii: " << ii << endl;
  }
 path_received =  true;
}

void PlannerUtils::planner_start_assistive_guidance_callback(const std_msgs::Bool flag)
{
    planner_start_assistive_guidance = flag.data;
    if (flag.data)
        clear_arrays();
    
 
}

void PlannerUtils::planner_final_goal_reached_callback(const std_msgs::Bool flag)
{
    planner_final_goal_reached = flag.data;
    path_received = false;
    inside_manouver = false;
    
}

// void PlannerUtils::planner_final_goal_position_callbak(const geometry_msgs::Point final_goal_msg)
// {
//     final_goal_position = Eigen::Vector3f(final_goal_msg.x, final_goal_msg.y, final_goal_msg.z);
// }

void PlannerUtils::publish_path2D_to_unity()
{
   //Publish the segmet that the drone is following to unity 
   scene_understanding_pkg_msgs::RRTPathPoints2D path;
   geometry_msgs::Point point_old;
   geometry_msgs::Point point;
   
   //Publish the entire set of points in the horizon to unity.
   for (int i = 0; i < planner_path_points.size(); i++)
   {
     point.x = planner_path_points[i](0);
     point.y = planner_path_points[i](1);
     point.z = planner_path_points[i](2);
     path.point.push_back(point);
   }
   
   //Add the projected position of the drone on the line 
   path.drone_proj_position.x = drone_projected_position_along_the_line(0);
   path.drone_proj_position.y = drone_projected_position_along_the_line(1);
   path.drone_proj_position.z = drone_projected_position_along_the_line(2);
   
   path.marker_proj_position.x = marker_projected_position_on_line(0);
   path.marker_proj_position.y = marker_projected_position_on_line(1);
   path.marker_proj_position.z = marker_projected_position_on_line(2);
   //Publish the actual considered segment to unity
   
   point_old.x = setpoint_old(0);
   point_old.y = setpoint_old(1);
   point_old.z = setpoint_old(2);
   
   point.x = setpoint(0);
   point.y = setpoint(1);
   point.z = setpoint(2);

   path.actual_segment.push_back(point_old);
   path.actual_segment.push_back(point);
   path2D_to_unity.publish(path);
}

void PlannerUtils::publish_end_APVI_mode()
{
  std_msgs::Bool flag_;
  flag_.data = true;
  end_APVI_mode.publish(flag_);
  path_received = false;
  inside_manouver = false;
}

void PlannerUtils::publish_pause_planner(bool flag)
{
  std_msgs::Bool flag_;
  flag_.data = flag;
  planner_paused.publish(flag_);
}

void PlannerUtils::publish_marker_projection_to_Unity(Eigen::Vector3f point)
{   
    geometry_msgs::Point point_;
    point_.x = point(0);
    point_.y = point(1);
    point_.z = point(2);

    planner_marker_projection_unity.publish(point_);
}


void PlannerUtils::fill_mission_setpoints_array()
{
  bool init = false;
  //Define Initialization phase if no points are received 
  if (planner_path_points.size() == 0)
  {
    //Waiting to receive some waypoints 
    init = true;
  }

  if (planner_path_points.size() > 0)
  {
    if (init == true)
    {
      //First Iteration: Points are received and the init flag is still true
      setpoint_index_counter = 0;
      old_horizon_goal = planner_path_points[planner_path_points.size() - 1]; //Take the coordinates of the old horizon goal

    }
    else
    {
      // If it is not the first iteration, check if the planner is updated looking of the coordinates of the horizon goal 
      // setpoint has been changed. 
      Eigen::Vector3f goal_vec_diff = planner_path_points[planner_path_points.size() - 1] - old_horizon_goal;
      if (goal_vec_diff.sum() != 0) //if the difference is different than zero the two setpoints are different
      {
        setpoint_index_counter = 0;
        step = planner_path_points.size() - 1;
        old_horizon_goal = planner_path_points[planner_path_points.size() - 1]; // save the new last orizon goal
        
      }
    }
  }

}

void PlannerUtils::project_drone_position_on_segment()
{
   fill_mission_setpoints_array();
  //  Eigen::Vector3f point1 =  Eigen::Vector3f(0.0, 0.0, 0.0);
  //  Eigen::Vector3f point2 =  Eigen::Vector3f(1.0, 1.0, 0.0);
  //  planner_path_points.push_back(point1);
  //  planner_path_points.push_back(point2);
   step = 1; 
   if (planner_path_points.size() == 0)
   {
     cout << "[Planner Manager] No waypoints received! Exit." << endl;
     return;
   }

   //Initialize the two setpoints that define the line that the robot has to follow 
   setpoint     = planner_path_points[setpoint_index_counter + step];
   setpoint_old = planner_path_points[setpoint_index_counter];

   //Find the line to follow slope 
   m_l1 = (setpoint(1) - setpoint_old(1)) / (setpoint(0) - setpoint_old(0));
   //Find the line to follow intercept 
   float b_l1 = setpoint(1) - m_l1 * setpoint(0);;
   //Evaluate the point-line perpendicular distance between the robot position and the line to follow 
   Eigen::Vector3f setpoints_diff = setpoint_old - setpoint;
   Eigen::Vector3f setpoint_quad_diff = setpoint - quad_pose;
   float d = sqrt(pow(setpoints_diff(0), 2) + pow(setpoints_diff(1), 2));
   point_line_distance = abs(setpoints_diff(0) * setpoint_quad_diff(1) - setpoint_quad_diff(0) * setpoints_diff(1)) / (d);
   cout << "point_line_distance: " << point_line_distance << endl;
   
   //Find the Projection of the drone position on the line between the two setpoints 
   float gamma = atan(m_l1); //angle between the line and the W x axis
   intercept = setpoint(1) - m_l1 * setpoint(0); // Intersection between the line and the Y World axis
   y_drone_on_line = m_l1 * quad_pose(0) + intercept; // Y value of the robot position projected on the line l1
   
   /* Evaluate the distance between the marker position and the point:
   A triangle is defined between:
   P1: the robot position P1
   P2: a point P2(x_robot, y_drone_on_line) which is sliding on the line 
   P3:  a point P3(x_int, y_int) where x_int y_int are the intersection point between the line and the perpendicular distacne 
       from  the line point_line_distance 
   */
   float d1 = abs(quad_pose(1) - y_drone_on_line); //distance along Y axis betwene the robot y pos and its projection on line.
   //Define the angle alfa on P2 between P1 and P3. (The angle on P1 is gamma)
   float alfa = (C_PI / 2) - gamma;
   // FInd the projection d2 on the line of the segment P2-P1
    float d2 = abs(d1 * cos(alfa));
   // Find the intersection coordinates of P3 between l1 and the line through P1 and P3.
   bool drone_case = true;
   Eigen::Vector3f proj = evaluate_projection_on_line(quad_pose,gamma, y_drone_on_line, d2, intercept, point_line_distance, drone_case);
   drone_projected_position_along_the_line = proj;

 
    //EValuate distance between the point P3 and the next stepoint to reach  
    Eigen::Vector3f P3_setpoint_diff = setpoint - drone_projected_position_along_the_line;
    float distance_from_setpoint = sqrt(pow(P3_setpoint_diff(0), 2) + pow(P3_setpoint_diff(1), 2));
    cout << "[PLANNER MANAGER] Distance P3 (drone proj on l1) to next Setpoint: " << distance_from_setpoint << endl;
    //If the distance is under a certain threshold, then the iterator is increased and the next segment is considered
    if (distance_from_setpoint < 0.05 && planner_path_points.size() > setpoint_index_counter + step)
    {
      setpoint_index_counter = setpoint_index_counter + 1;
    }
    
    publish_path2D_to_unity();
    visualizer.publish_marker_proj_on_the_line(drone_projected_position_along_the_line);
    visualizer.publish_planner_line_to_follow_in_rviz(planner_path_points);
    //save drone line distance 
    
}


void PlannerUtils::project_des_position_on_segment()
{
    //EValuate the point line distance between the marker position and the line 
    Eigen::Vector3f setpoints_diff = setpoint_old - setpoint;
    Eigen::Vector3f setpoint_des_pos_diff = setpoint - des_pose;
    Eigen::Vector3f old_setpoint_des_pos_diff = setpoint_old - des_pose;
    Eigen::Vector3f drone_marker_des_proj_line_diff = quad_pose - marker_position_proj_on_line;
    Eigen::Vector3f drone_marker_diff = des_pose - quad_pose;

    float d = sqrt(pow(setpoints_diff(0), 2) + pow(setpoints_diff(1), 2));
    float point_line_distance = abs(setpoints_diff(0) * setpoint_des_pos_diff(1) - setpoint_des_pos_diff(0) * setpoints_diff(1)) / (d);
    
    //EValuate the y projection of the marker on the line given its x coordinate 
    float gamma = atan(m_l1);
    y_marker_on_line = m_l1 * des_pose(0) + intercept;

    //Same steps done as in the previous function for the drone position
    float d1 = abs(des_pose(1) - y_marker_on_line);
    float alfa = (C_PI / 2) - gamma; //gamma is the angle between the line and the x axis and is the same between the three points considered before (guarda ipad)
    float d2 = abs(d1 * cos(alfa));
    
    d_l_distance = d1;
    // [SAFETY MECHANISM] If the marker is closer to the previous setpoint, the desired distance on the line is the fixed in the last computed
    distance_marker_to_next_setpoint = sqrt(pow(setpoint_des_pos_diff(0), 2) + pow(y_marker_on_line - setpoint(1), 2));
    distance_marker_to_prev_setpoint = sqrt(pow(old_setpoint_des_pos_diff(0), 2) + pow(old_setpoint_des_pos_diff(1), 2));
    float drone_distance_to_next_setpoint = sqrt(pow( setpoint_des_pos_diff(0), 2) + pow(setpoint_des_pos_diff(1), 2));
    //float drone_distance_from_desired_position_on_line =  sqrt(pow( drone->position_GF.x - marker_projected_position_on_line.x, 2) + pow(drone->position_GF.y -  marker_projected_position_on_line.y, 2));
    float distance_interaction_marker_projection_during_bypass = sqrt(pow( drone_marker_des_proj_line_diff(0), 2) + pow(drone_marker_des_proj_line_diff(1), 2));
    drone_marker_distance = sqrt(pow( drone_marker_diff(0), 2) + pow(drone_marker_diff(1), 2));
    
    //[Safety Mechanish] If the distance between the marker and the drone increase mroe than 0.30cm, the projected desired position on the line stop to slide until the marker
    // become closer enough to the drone. This increase the safety of very dangerous movement that can make the robot starting to oscillate.
    keep_des_pos_fixed = false;
    if (drone_marker_distance > 0.3)  
    {
      keep_des_pos_fixed = true;
      inside_manouver = true;
      publish_pause_planner(keep_des_pos_fixed);
      last_marker_projected_position_on_line = marker_projected_position_on_line;
    }
    else
    {
      publish_pause_planner(keep_des_pos_fixed);
    }
    
   bool drone_case = false;
   Eigen::Vector3f proj = evaluate_projection_on_line(des_pose,gamma, y_marker_on_line, d2, intercept, point_line_distance, drone_case);


   if (inside_manouver == false)
   {
      marker_projected_position_on_line = proj;
      step_sum_int = 0; 
   }
   else
   {
     if (inside_manouver && drone_marker_distance <= 0.3){
        bool flag_in_int = true;
        Eigen::Vector3f proj_int = interpolate_projection_marker_on_line(&flag_in_int, proj);
        marker_projected_position_on_line = proj_int; 
        inside_manouver = flag_in_int;  //true until the interpolation is finished
     }
   }
   
    cout << "[PLANNER MANAGER] Des Position on LINE X: " <<    marker_projected_position_on_line(0)  << endl;
    cout << "[PLANNER MANAGER] Des Position on LINE Y: " <<    marker_projected_position_on_line(1) << endl; 
    visualizer.publish_des_pos_marker_proj_on_the_line(marker_projected_position_on_line);
    // planner_path_points.clear();
}

Eigen::Vector3f PlannerUtils::interpolate_projection_marker_on_line(bool * flag, Eigen::Vector3f actual_proj)
{
  /*
    This function provide a sequence of interpolated projected marker desired position on the line. 
    This function is used when the user is coming back from a manouver outside the planner. 
    Sinc the condition to activate the inside manouver flag is that the distance between the projected drone and te projected marker 
    is bigger than 0.3 m, when the user is coming back from the manouver, in the worst case a 0.6m step in the desired position can be forwarded as input to 
    the controller causing oscillation. 
    This function provide a tool to interpolate the distance between the two positions
  */
   Eigen::Vector3f setp1 = last_marker_projected_position_on_line;
   Eigen::Vector3f setp2 = actual_proj;
   //Define the line passing through the two points
   m_l1 = (setp1(1) - setp2(1)) / (setp1(0) - setp2(0));
   float gamma = atan(m_l1); //angle between the line and the W x axis
   float intercept_line = setp1(1) - m_l1 * setp1(0);
   
   Eigen::Vector3f new_proj = Eigen::Vector3f(0.0, 0.0, 0.0);
   float step = 0.01; //
   //Define the incremental or decremental step
   if (setp2(0) > setp1(0)){
      float x_new_proj = setp1(0) + step_sum_int;
      float y_new_proj = m_l1 *x_new_proj +  intercept_line;
      new_proj = Eigen::Vector3f(x_new_proj, y_new_proj, setp1(2));
      
   }
   else
   {
      float x_new_proj = setp1(0) - step_sum_int;
      float y_new_proj = m_l1 *x_new_proj +  intercept_line;
      new_proj = Eigen::Vector3f(x_new_proj, y_new_proj, setp1(2));

   }
    //Evaluate distance between the the actual marker projection and the new marker projection 
    Eigen::Vector3f distance_diff =  setp2 - new_proj;
    float distance = sqrt(pow(distance_diff(0), 2) + pow(distance_diff(1), 2));
    if (distance < 0.02){
      *flag = false;
      step_sum_int = 0;
    }
    
    step_sum_int = step_sum_int + step;
    return new_proj;
}

Eigen::Vector3f PlannerUtils::interpolate_drone_position_with_user_interaction_marker(bool * flag,Eigen::Vector3f marker_pose, Eigen::Vector3f actual_proj)
{
  /*
    This function provide a sequence of interpolated projected marker desired position on the line. 
    This function is used when the user is coming back from a manouver outside the planner. 
    Sinc the condition to activate the inside manouver flag is that the distance between the projected drone and te projected marker 
    is bigger than 0.3 m, when the user is coming back from the manouver, in the worst case a 0.6m step in the desired position can be forwarded as input to 
    the controller causing oscillation. 
    This function provide a tool to interpolate the distance between the two positions
  */
   Eigen::Vector3f setp1 = marker_pose;
   Eigen::Vector3f setp2 = actual_proj;
   //Define the line passing through the two points
   m_l1 = (setp1(1) - setp2(1)) / (setp1(0) - setp2(0));
   float gamma = atan(m_l1); //angle between the line and the W x axis
   float intercept_line = setp1(1) - m_l1 * setp1(0);
   
   Eigen::Vector3f new_proj = Eigen::Vector3f(0.0, 0.0, 0.0);
   float step = 0.01; //
   //Define the incremental or decremental step
   if (setp1(0) > setp2(0)){
      float x_new_proj = setp2(0) + step_sum_int;
      float y_new_proj = m_l1 *x_new_proj +  intercept_line;
      new_proj = Eigen::Vector3f(x_new_proj, y_new_proj, setp2(2));
      
   }
   else
   {
      float x_new_proj = setp2(0) - step_sum_int;
      float y_new_proj = m_l1 *x_new_proj +  intercept_line;
      new_proj = Eigen::Vector3f(x_new_proj, y_new_proj, setp1(2));

   }
    //Evaluate distance between the the actual marker projection and the new marker projection 
    Eigen::Vector3f distance_diff =  setp1 - new_proj;
    float distance = sqrt(pow(distance_diff(0), 2) + pow(distance_diff(1), 2));
    if (distance < 0.02){
      *flag = false;
      step_sum_int = 0;
    }
    
    step_sum_int = step_sum_int + step;
    return new_proj;
}


void PlannerUtils::evaluate_variable_damping_coefficient()
{
  
  Eigen::Vector3f drone_marker_proj_diff = marker_projected_position_on_line - drone_projected_position_along_the_line;
  float d_proj_drone = sqrt(pow(drone_marker_proj_diff(1), 2) + pow(drone_marker_proj_diff(0), 2));

  //Evaluate the angle that identify the direction of the force respect the direction of the line to follow l2, passing throug the drone position. 
  float gamma = acos(d_proj_drone/drone_marker_distance);
  // Avoid NAN
  if (gamma != gamma )
  {  
    gamma = C_PI;
  }

  // //Check also the difference between the coordinates of 
  if (distance_marker_to_prev_setpoint < distance_marker_to_next_setpoint)
  {
    gamma = C_PI/2 +  (C_PI/2 -gamma);
  }

  //filter gamma over a history of observation 
  gamma = average_angle_gamma(gamma);

  //Here change the value of damping implemnting the functiion respect the gamma value 
  // Implementing the linear, exponential and sqrt function.

  float d_min = (float)D_MIN; 
  float d_max = (float)D_MAX;
  
  
 
  if (damping_function == "linear")
  {
     float m_linear = (d_max - d_min)/(C_PI - 0);
     d_output = m_linear * gamma;
     cout << "[PLANNER MANAGER] d_output_linear : " << d_output << " gamma: " << gamma << endl;
  }
  else if (damping_function == "exponential")
  {
      //Exponential 
      float a = pow(d_max, 1/C_PI); //Find the basis of the exponnetial function 
      d_output = pow(a, gamma);
      cout << "[PLANNER MANAGER] d_output_exp : " << d_output << " gamma: " << gamma << endl;
  }
  else
  {
     //Sqrt 
     float a_sqrt = (d_max - d_min)/(sqrt(C_PI - 0.0));
     d_output = a_sqrt * sqrt(gamma - 0.0) + d_min;
     cout << "[PLANNER MANAGER] d_output_sqrt : " << d_output << " gamma: " << gamma << endl;

  }

  // if (keep_des_pos_fixed == true)
  // {
  //  drone->d_output = 20; //d_max
  // cout << "[MISSION] POSITION FIXED FOR BYPSASS MANEUVER: X: " <<  drone_desired_position_along_line.x << " Y: " << drone_desired_position_along_line.y << endl;
  // }
 

  // Only for debug and plot 
  gamma_output = gamma;



}


Eigen::Vector3f PlannerUtils::evaluate_projection_on_line(Eigen::Vector3f agent, float angle, float y_agent_on_line, float d2, float intercept, float pl_d, bool drone_case)
{
  float x_projection_value =0.0;
  float y_marker_projection_on_line = 0.0;
  float y_projection_value = 0.0;

  float x_final_proj = 0.0;
  float y_final_proj = 0.0;
  Eigen::Vector3f projection;

  if (m_l1 > 0 && agent(1) > y_agent_on_line)
    {
      x_projection_value = d2 * cos(angle) + agent(0);
      y_projection_value = m_l1 * x_projection_value + intercept;
    }
    else if (m_l1 > 0 && agent(1) < y_agent_on_line)
    {
      x_projection_value = agent(0) - d2 * cos(angle);
      y_projection_value = m_l1 * x_projection_value + intercept;
    }
    else if (m_l1 < 0 && agent(1) > y_agent_on_line)
    {
      x_projection_value = agent(0) - d2 * cos(angle);
      y_projection_value = m_l1 * x_projection_value + intercept;
    }
    else
    {
      x_projection_value = d2 * cos(angle) + agent(0);
      y_projection_value = m_l1 * x_projection_value + intercept;
    }
    
    if (drone_case)
    {
      if ((angle < 0.05 && angle > -0.05) || pl_d == 0)
      {
         y_agent_on_line =  intercept;
         y_projection_value = y_agent_on_line;
         // cout << "X drone proj value on the line " << drone->position_GF.x << endl;
         // cout << "Y drone proj value on the line " << y_drone_projection_on_line << endl;
         projection = Eigen::Vector3f(agent(0), y_projection_value,des_pose(2));
  
      }
      else
      {
        projection = Eigen::Vector3f(x_projection_value, y_projection_value,des_pose(2));
      }
    }
    else
    {
      projection = Eigen::Vector3f(x_projection_value, y_projection_value, des_pose(2));
    }
    return projection;
}

double average_(std::vector<double> const& v){
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<double>(v.size());
    return accumulate(v.begin(), v.end(), 0.0) / v.size();
}

float PlannerUtils::average_angle_gamma(double gamma)
{
  float  avg_stamps_ = 80;
  // AVeraging Position along X axis 
  gamma_vector.push_back(gamma);
  float gamma_= average_(gamma_vector);
  //Clean the vector 
  if (gamma_vector.size() > avg_stamps_)
  {
    gamma_vector.erase(gamma_vector.begin());
  } 

 return gamma_;
  
}

vector<Eigen::Vector3f> PlannerUtils::generate_random_intermediate_points_location()
{
    //The obstacle is supposed to be in a region from x = [0.3, 1.5] y = [-0.4, 0.4]
    vector<Eigen::Vector3f> int_points;
    Eigen::Vector3f P= Eigen::Vector3f(0.0, 0.0, 0.0);
    
    int max_counter = 1000;
    float distance = 0.0;
    for (int j = 0; j < 2; j++){
      for (int i = 0; i < max_counter; i++){
       std::mt19937 gen(rd());
       std::uniform_real_distribution<double> disx(-1.2, 0.9);
       double x_random = disx(gen);
       std::uniform_real_distribution<double> disy(-1.0, 0.6);
       double y_random = disy(gen);
       if (j > 0)
       {
        //Evaluate the distance of the second intermediate target from the previous one 
        float x_prev = int_points[0](0);
        float y_prev = int_points[0](1);
        distance = sqrt(pow(x_random - x_prev, 2) + (pow(y_random - y_prev, 2)));
       }
      if (j == 0 || distance > 0.4){
          
            P = Eigen::Vector3f((float)x_random, (float)y_random, 0.0);
            int_points.push_back(P);
            break;
      }

    }
     
    }
    
   return int_points;
   
}
void PlannerUtils::find_goal_position(bool APVI)
{
/*
  This function provide the coordinates for the final goal in APVI and FPVI both in simulation and real scenario. 
  In Simulation, both in APVI (managed by the rrt planner) and FPVI, the coordinates of y_target will be y_target > 0 if x_quad < 0 and 
  y_target < 0 if x_quad>0 when the modality is called. This because the map in simulation is different from the real one 

  In the real scenario, on the APVI side the rrt planner provides the parametrs 
  On the FPVI side the same procedure is kept, x_target is flipped depending by the position of the quad and some randomness on 
  the final y_target location is added. Moreover the intermediates target can appear completely randmo.
*/
  if (APVI){
      //Random number generator 
    //int scenario = 1;
    float P1_final_goal_x, P1_final_goal_y;
    nh_.getParam("/tele_control_params/scenario", scenario);
    if (scenario==1 || scenario==2){
        nh_.getParam("/tele_control_params/final_goal_sim/P1/x", P1_final_goal_x);
        nh_.getParam("/tele_control_params/final_goal_sim/P1/y", P1_final_goal_y);
      
    }
    else{
         nh_.getParam("/tele_control_params/final_goal/P1/x", P1_final_goal_x);
        nh_.getParam("/tele_control_params/final_goal/P1/y", P1_final_goal_y);
    }
    final_goal_position = Eigen::Vector3f(P1_final_goal_x,P1_final_goal_y, 0.8 );
  }
  else
  {
      std::mt19937 gen(rd());
      std::uniform_real_distribution<double> dis(0.0, 1.0);
      double random_number = dis(gen);
      float y_final_goal = P1_final_goal_position(1);
      if (random_number > 0.5){
          y_final_goal = -1*y_final_goal;
      }

      if (quad_pose(0) < 0)
      {
          if (scenario==1 || scenario==2){
           y_final_goal = P1_final_goal_position(1);
          }
      
        final_goal_position = Eigen::Vector3f(P1_final_goal_position(0), y_final_goal,P1_final_goal_position(2));
      }
      else
      {
         if (scenario==1 || scenario==2){
           y_final_goal = -1*P1_final_goal_position(1);
          }
      Eigen::Vector3f v = Eigen::Vector3f(-1*P1_final_goal_position(0), y_final_goal,P1_final_goal_position(2));
      final_goal_position = v;
      }
  }
  
  
    
}

void PlannerUtils::clear_arrays()
{
  gamma_vector.clear();
  planner_path_points.clear();
}

void PlannerUtils::get_quad_pose(Eigen::Vector3f quad_pose_, Eigen::Vector3f quad_orientation_)
{
  quad_pose = quad_pose_;
  quad_orientation = quad_orientation_;
}

void PlannerUtils::get_des_pose(Eigen::Vector3f des_pose_)
{
  des_pose = des_pose_;
}

void PlannerUtils::get_commanded_pose(Eigen::Vector3f xc_pose_)
{
  xc_pose = xc_pose_;
}

float PlannerUtils::obtain_drone_line_distance()
{
  //return drone line distance 
   return d_l_distance;
}

}