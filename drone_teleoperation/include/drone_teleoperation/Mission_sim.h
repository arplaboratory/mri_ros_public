
#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>
#include <numeric>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include "drone_teleoperation/arpl_drone_sim.h"

using namespace std;

class Mission
{
public:
  int state;
  int previous_state;
  float traj_x;
  int take_off_counter; //counter to wat the drone take off before switch to another state

  //Params for trajectory definition
  int trajectory_case;

  double x_error;
  double y_error;

  float D_MIN;
  float D_MAX;

  double F_x;
  double F_y;

  //Save the value of the previously received rrt horizon gial point
  geometry_msgs::Point old_horizon_goal;

  vector<float> setpoint_x_vec;
  vector<float> setpoint_y_vec;
  int setpoint_index_counter = 0;
  int step = 1;
  float d_old = 0.0; //distance old from the setpoint target

  //Define the desire damping function
  string damping_function;
  //Mission Goal (only in mission.case == 3)
  geometry_msgs::Point goal;

  geometry_msgs::Point initial_position;
  geometry_msgs::Point x_marker_proj_on_drone_line_l2;
  geometry_msgs::Point desired_position_along_line;
  geometry_msgs::Point drone_desired_position_along_line;
  geometry_msgs::Point drone_projected_position_along_the_line;
  geometry_msgs::Point setpoint;
  geometry_msgs::Point setpoint_old;
  
  bool keep_des_pos_fixed = false;
  float line_slope = 0;
  int trajectory_number;
  double trajectory_time_stamp = 0;
  bool init_new_des_projected_pose = false;
  bool take_off;
  bool mapping_flag = false;
  bool arm_motors_success = false;
  bool take_off_success = false;
  bool initial_hovering = false;

  Mission()
  {
  }

  ~Mission()
  {
  }

  //Trajectory Generator
  void trajectory_generator(float t)
  {
    //Generate a sin trajectory
    traj_x = sin(t);
  }

  void fill_mission_setpoints_vector(arpl_drone_sim *drone)
  {
    bool init = false;
    if (setpoint_x_vec.size() == 0)
    {
      //First waypoints received
      init = true;
    }
    setpoint_x_vec.clear();
    setpoint_y_vec.clear();
    
  
    for (int i = 0; i < drone->rrt_path_points.size(); i++)
    {

      setpoint_x_vec.push_back(drone->rrt_path_points[i].x);
      setpoint_y_vec.push_back(drone->rrt_path_points[i].y);

    }

    
    //Check if theb
    if (setpoint_x_vec.size() > 0)
    {
      if (init == true)
      {
        //First itration
        setpoint_index_counter = 0;
        old_horizon_goal = drone->rrt_path_points[drone->rrt_path_points.size() - 1]; // save the last orizon goal
      }
      else
      {
        //Upodate the setpoint_index_counter and the old horizon goal value only if the new horizon goal if different from the previous stored
        if (old_horizon_goal.x != setpoint_x_vec[drone->rrt_path_points.size() - 1] && old_horizon_goal.y != setpoint_y_vec[drone->rrt_path_points.size() - 1]) //&& drone->rrt_final_goal_reached == true)
        {
          setpoint_index_counter = 0;
          step = setpoint_x_vec.size() - 1;
          old_horizon_goal = drone->rrt_path_points[drone->rrt_path_points.size() - 1]; // save the last orizon goal
          
        }
      }
    }

    // setpoint_x_vec.clear();
    // setpoint_y_vec.clear();

    // //definire un meccanismo per il quale se il vettore è vuoto il path non viene caricato.

    //  for (int i = 1; i < 40; i++)
    //  {
    //    float x = i* 0.2;
    //    float y = 0.6*sin(x) + x/2;

    //    setpoint_x_vec.push_back(x);
    //    setpoint_y_vec.push_back(y);

    //  }
    /*
     setpoint_x_vec.push_back(1);
     setpoint_x_vec.push_back(1.2); 
     setpoint_x_vec.push_back(1.4);
     setpoint_x_vec.push_back(1.4);

     setpoint_y_vec.push_back(1);
     setpoint_y_vec.push_back(0.95);
     setpoint_y_vec.push_back(0.85);
     */
  }
  //EValuate drone distance from the setpoints alog the line
  // Return true when the drone is close enough and setpoint to consider is the next one

  /*
bool evaluate_drone_distance_from_the_current_setpoint(arpl_drone_sim *drone)
{
   float setpoint_x = setpoint_x_vec[setpoint_index_counter];
   float setpoint_y = setpoint_y_vec[setpoint_index_counter];

   float setpoint_old.x = 0.0;
   float setpoint_old.y = 0.0;

   if (setpoint_index_counter > 0)
   {
     setpoint_old.x =  setpoint_x_vec[setpoint_index_counter - 1];
     setpoint_old.y =  setpoint_y_vec[setpoint_index_counter - 1];
   }
   else
   {
     setpoint_old.x =  initial_position.x;
     setpoint_old.y =  initial_position.y;
   }
   //Find the line to follow slope and intercpet 
  
    float  m_l1 = (setpoint_y - setpoint_old.y)/(setpoint_x - setpoint_old.x);
   
   //Find intercept of the actual line to follow
   float b_l1 = setpoint_y - m_l1*setpoint_x;

   //Write the parameter a, b and c related to the implicit formulatiion of the line equation l1: ax + by + c = 0;
   float a_l1 = (setpoint_y - setpoint_old.y);
   float b_l1_imp = (setpoint_x - setpoint_old.x);
   float c_l1 = setpoint_old.y*setpoint_x - setpoint_old.x*setpoint_y;

   //Find the slope of the perpendicular line poassing through the drone position and the pointb D1.
   // The point D1 is the intersection point between the line tevaluate_drone_distance_from_the_current_setpoint
  {
    m_ldd1 = -1/0.01;
  }
  else
  {
     m_ldd1 = -1/m_l1;
  }
   cout << "m_l1: " << m_l1 << endl;
   cout << "m_ldd1: " << m_ldd1 << endl;
  //Find the intercept b_dd1 of the explicit line equation
  float b_dd1 = drone->position_GF.y - m_ldd1*drone->position_GF.x; //The point (0, b_dd1) is another point lying on the line passing through teh drone position and D1
                                                                    //Required to find the implicit equation of the line 
  //Write the equation of the line in implicit formulation 
  float a_dd1 = (drone->position_GF.y - b_dd1);
  float b_dd1_imp = (drone->position_GF.x  - 0.0);
  float c_dd1 = b_dd1*drone->position_GF.x   - 0.0*drone->position_GF.y;


  //Using the results obtained from the cramer formulation, is it possibleto have the coordinates of the point D1
  //The point D1 is the intersection between the line dd1 and the line l1
  // considering the two implicit equations: 
  // l1 : a_l1 x + b_l1_imp*y + c_l1 = 0.0;
  // ldd1 : a_dd1*x + b_dd1_imp*y + c_dd1 = 0.0; 

  float x_d1 = (b_l1_imp*c_dd1 - c_l1*b_dd1_imp )/(a_l1*b_dd1_imp - b_l1*a_dd1);
  float y_d1 = (a_l1*c_dd1 -c_l1*a_dd1 )/(a_l1*b_dd1_imp - b_l1*a_dd1);
  
  cout << "x_d1: " << x_d1 << " y_d1: " << y_d1 << endl;

  //EValuate distance between the point D1 and the current stepoint to reach 

  float distance_from_setpoint = sqrt(pow(setpoint_x - x_d1, 2) + pow(setpoint_y - y_d1, 2));
  cout << "distance_from_setpoint: " << distance_from_setpoint << endl;
  //If the distance is under a certain threshold, then the iterator is increased and the next segment is considered 
  if (distance_from_setpoint < 0.05)
  {
    setpoint_index_counter =  setpoint_index_counter +1;
  }

}

*/

  void evaluate_drone_distance_from_the_current_setpoint(arpl_drone_sim *drone)
  {
    // EVery time this function is called fill the stpoint vector with the latest evaluated setopints
    fill_mission_setpoints_vector(drone);

    setpoint.x = setpoint_x_vec[setpoint_index_counter + step];
    setpoint.y = setpoint_y_vec[setpoint_index_counter + step];

    // if (setpoint_index_counter > 0)
    // {
    setpoint_old.x = setpoint_x_vec[setpoint_index_counter];
    setpoint_old.y = setpoint_y_vec[setpoint_index_counter];

    cout << "[DRONE LINE DISTANCE NEW SETPOINT X: " <<  setpoint.x << " Y: " << setpoint.y << endl;
    cout << "[DRONE LINE DISTANCE OLD SETPOINT X: " <<  setpoint_old.x << " Y: " << setpoint_old.y << endl;
   


    // }
    // else
    // {
    //   setpoint_old.x = drone->position_GF.x + 0.1; //droneinitial_position.x;
    //   setpoint_old.y = drone->position_GF.y + 0.1;
    // }
    //Find the line to follow slope and intercpet

    float m_l1 = (setpoint.y - setpoint_old.y) / (setpoint.x - setpoint_old.x);

    //Find intercept of the actual line to follow
    float b_l1 = setpoint.y - m_l1 * setpoint.x;

    //Evaluate Drone line to follow  distance

    float a1 = setpoint_old.x - setpoint.x;
    float a2 = setpoint.y - drone->position_GF.y;

    float b1 = setpoint.x - drone->position_GF.x;
    float b2 = setpoint_old.y - setpoint.y;

    float d = sqrt(pow(a1, 2) + pow(b2, 2));

    float point_line_distance = abs(a1 * a2 - b1 * b2) / (d);

    //Given the equation of the line and the x_drone_GF, find the equivalent point y_W on the line

    float gamma = atan(m_l1);
    float intercept = setpoint.y - m_l1 * setpoint.x;
    float y_drone_on_line = m_l1 * drone->position_GF.x + intercept;

    // cout << "point_line_distance: " << point_line_distance << endl;
    // cout << "gamma: " << gamma << endl;

    //Evaluate the distance between the marker position and the point (drone->des_user_pos_GF.x, y_marker_on_line)
    float d1 = abs(drone->position_GF.y - y_drone_on_line);
    //Find the value of the angle between the line and the side d2 of the triangle built between the points (drone->des_user_pos_GF.x, y_marker_on_line),
    // Marker Position and the intersection of the marker position and the point line distance
    float alfa = (C_PI / 2) - gamma; //gamma is the angle between the line and the x axis and is the same between the three points considered before (guarda ipad)
    //Find the value of the triange side alog the line
    float d2 = abs(d1 * cos(alfa));
    //Fin the ccordinates of the interception between the drone point and the point line distance intersection (the projection
    //of the point on the line)
    float x_projection_value = 0.0;
    float y_drone_projection_on_line = 0.0;

    if (m_l1 > 0 && drone->position_GF.y > y_drone_on_line)
    {
      x_projection_value = d2 * cos(gamma) + drone->position_GF.x;
      y_drone_projection_on_line = m_l1 * x_projection_value + intercept;
    }
    else if (m_l1 > 0 && drone->position_GF.y < y_drone_on_line)
    {
      x_projection_value = drone->position_GF.x - d2 * cos(gamma);
      y_drone_projection_on_line = line_slope * x_projection_value + intercept;
    }
    else if (m_l1 < 0 && drone->position_GF.y > y_drone_on_line)
    {
      x_projection_value = drone->position_GF.x - d2 * cos(gamma);
      y_drone_projection_on_line = m_l1 * x_projection_value + intercept;
    }
    else
    {
      x_projection_value = d2 * cos(gamma) + drone->position_GF.x;
      y_drone_projection_on_line = m_l1 * x_projection_value + intercept;
    }

    if ((gamma < 0.05 && gamma > -0.05) || point_line_distance == 0)
    {
      y_drone_on_line = line_slope * drone->position_GF.x + intercept;
      y_drone_projection_on_line = y_drone_on_line;

      cout << "X drone proj value on the line " << drone->position_GF.x << endl;
      cout << "Y drone proj value on the line " << y_drone_projection_on_line << endl;

      drone_projected_position_along_the_line.x = drone->position_GF.x;
      drone_projected_position_along_the_line.y = y_drone_projection_on_line;
    }
    else
    {
      cout << "X drone proj value on the line " << x_projection_value << endl;
      cout << "Y drone proj value on the line " << y_drone_projection_on_line << endl;

      drone_projected_position_along_the_line.x = x_projection_value;
      drone_projected_position_along_the_line.y = y_drone_projection_on_line;
    }

    //EValuate distance between the point D1 and the current stepoint to reach

    float distance_from_setpoint = sqrt(pow(setpoint.x - drone_projected_position_along_the_line.x, 2) + pow(setpoint.y - drone_projected_position_along_the_line.y, 2));
    cout << "[MISSION] DIstance drone proj value and setpoint: " << distance_from_setpoint << endl;
    //If the distance is under a certain threshold, then the iterator is increased and the next segment is considered
    if (distance_from_setpoint < 0.05 && setpoint_x_vec.size() > setpoint_index_counter + step)
    {
      setpoint_index_counter = setpoint_index_counter + 1;
    }
    
     drone->publish_path2D_to_unity(setpoint_old, setpoint);
  }




  //Evaluate line equation
  float evaluate_point_line_distance(arpl_drone_sim *drone)
  {
    // setpoint.x = setpoint_x_vec[setpoint_index_counter];
    // setpoint.y = setpoint_y_vec[setpoint_index_counter];
    
    setpoint.x = setpoint_x_vec[setpoint_index_counter + step];
    setpoint.y = setpoint_y_vec[setpoint_index_counter + step];

    setpoint_old.x = setpoint_x_vec[setpoint_index_counter];
    setpoint_old.y = setpoint_y_vec[setpoint_index_counter];
   
    // if (setpoint_index_counter > 0)
    // {
    //   setpoint_old.x = setpoint_x_vec[setpoint_index_counter - 1];
    //   setpoint_old.y = setpoint_y_vec[setpoint_index_counter - 1];
    // }
    // else
    // {
    //   setpoint_old.x = initial_position.x;
    //   setpoint_old.y = initial_position.y;
    // }

    //!!!La retta non puo wssere tra drone e goal ma deve essere fissa
    float a1 = setpoint_old.x - setpoint.x;
    float a2 = setpoint.y - drone->des_user_pos_GF.y;

    float b1 = setpoint.x - drone->des_user_pos_GF.x;
    float b2 = setpoint_old.y - setpoint.y;

    float d = sqrt(pow(a1, 2) + pow(b2, 2));

    float point_line_distance = abs(a1 * a2 - b1 * b2) / (d);

    return point_line_distance;
  }

  /*
void evaluate_intersection_point_P(arpl_drone_sim *drone)
{
  //Intersection point between the user marker teleoperation and the line to follow
  //Pemits to find the desired position on the line to follow
   float setpoint_x = setpoint_x_vec[setpoint_index_counter];
   float setpoint_y = setpoint_y_vec[setpoint_index_counter];

   float setpoint_old.x = 0.0;
   float setpoint_old.y = 0.0;

   if (setpoint_index_counter > 0)
   {
     setpoint_old.x =  setpoint_x_vec[setpoint_index_counter - 1];
     setpoint_old.y =  setpoint_y_vec[setpoint_index_counter - 1];
   }
   else
   {
     setpoint_old.x =  initial_position.x;
     setpoint_old.y =  initial_position.y;
   }
   //Find the line to follow slope and intercpet 
  
    float  m_l1 = (setpoint_y - setpoint_old.y)/(setpoint_x - setpoint_old.x);
   

   //Find intercept of the actual line to follow
   float b_l1 = setpoint_y - m_l1*setpoint_x;

   //Write the parameter a, b and c related to the implicit formulatiion of the line equation l1: ax + by + c = 0;
   float a_l1 = (setpoint_y - setpoint_old.y);
   float b_l1_imp = (setpoint_x - setpoint_old.x);
   float c_l1 = setpoint_old.y*setpoint_x - setpoint_old.x*setpoint_y;

   //Find the slope of the perpendicular line poassing through the drone position and the pointb D1.
   // The point D1 is the intersection point between the line to follow and the line representing the point line distance from the line to follow and theb drone position 
  
  float m_ldd1 = 0.0;
  if (m_l1 == 0)
  {
    m_ldd1 = -1/0.01;
  }
  else
  {
     m_ldd1 = -1/m_l1;
  }

  
  //Find the intercept b_dd1 of the explicit line equation
  float b_dd1 = drone->des_user_pos_GF.y - m_ldd1*drone->des_user_pos_GF.x; //The point (0, b_dd1) is another point lying on the line passing through teh drone position and D1
                                                                    //Required to find the implicit equation of the line 
  //Write the equation of the line in implicit formulation 
  float a_dd1 = (drone->des_user_pos_GF.y - b_dd1);
  float b_dd1_imp = (drone->des_user_pos_GF.x  - 0.0);
  float c_dd1 = b_dd1*drone->des_user_pos_GF.x   - 0.0*drone->des_user_pos_GF.y;


  //Using the results obtained from the cramer formulation, is it possibleto have the coordinates of the point D1
  //The point D1 is the intersection between the line dd1 and the line l1
  // considering the two implicit equations: 
  // l1 : a_l1 x + b_l1_imp*y + c_l1 = 0.0;
  // ldd1 : a_dd1*x + b_dd1_imp*y + c_dd1 = 0.0; 

  float x_d1 = (b_l1_imp*c_dd1 - c_l1*b_dd1_imp )/(a_l1*b_dd1_imp - b_l1*a_dd1);
  float y_d1 = (a_l1*c_dd1 -c_l1*a_dd1 )/(a_l1*b_dd1_imp - b_l1*a_dd1);
  
  cout << "x_des_pos: " << x_d1 << " y_des_pos: " << y_d1 << endl;

  // //EValuate distance between the point D1 and the current stepoint to reach 

  // float distance_from_setpoint = sqrt(pow(setpoint_x - x_d1, 2) + pow(setpoint_y - y_d1, 2));
  // cout << "distance_from_setpoint: " << distance_from_setpoint << endl;




}

*/
  float evaluate_drone_des_pos_proj_distance(arpl_drone_sim *drone, float point_line_distance)
  {
    // setpoint.x = setpoint_x_vec[setpoint_index_counter];
    // setpoint.y = setpoint_y_vec[setpoint_index_counter];


    setpoint.x = setpoint_x_vec[setpoint_index_counter + step];
    setpoint.y = setpoint_y_vec[setpoint_index_counter + step];


    setpoint_old.x = setpoint_x_vec[setpoint_index_counter];
    setpoint_old.y = setpoint_y_vec[setpoint_index_counter];
    
     
    cout << "[MARKER LINE DISTANCE] NEW SETPOINT X: " <<  setpoint.x << " Y: " << setpoint.y << endl;
    cout << "[MARKER LINE DISTANCE] OLD SETPOINT X: " <<  setpoint_old.x << " Y: " << setpoint_old.y << endl;


    // if (setpoint_index_counter > 0)
    // {
    //   setpoint_old.x = setpoint_x_vec[setpoint_index_counter - 1];
    //   setpoint_old.y = setpoint_y_vec[setpoint_index_counter - 1];
    // }
    // else
    // {
    //   setpoint_old.x = initial_position.x;
    //   setpoint_old.y = initial_position.y;
    // }

    //Given the equation of the line and the x_value_W of the user marker, find the equivalent point y_W on the line
    line_slope = (setpoint.y - setpoint_old.y) / (setpoint.x - setpoint_old.x);
    float gamma = atan(line_slope);
    float intercept = setpoint.y - line_slope * setpoint.x;
    float y_marker_on_line = line_slope * drone->des_user_pos_GF.x + intercept;

    //Evaluate the distance between the marker position and the point (drone->des_user_pos_GF.x, y_marker_on_line)
    float d1 = abs(drone->des_user_pos_GF.y - y_marker_on_line);
    cout << "d1: " <<d1 << endl;
    //Find the value of the angle between the line and the side d2 of the triangle built between the points (drone->des_user_pos_GF.x, y_marker_on_line),
    // Marker Position and the intersection of the marker position and the point line distance
    float alfa = (C_PI / 2) - gamma; //gamma is the angle between the line and the x axis and is the same between the three points considered before (guarda ipad)
    // cout << "Alfa: " <<alfa << endl;
    //Find the value of the triange side alog the line
    float d2 = abs(d1 * cos(alfa));
    //cout << "triangle side on the line length: " << d2 << endl;

    //EValuate the distance between the  marker position in GF and the setpoint old and the setpoint to reach.
    // If the marker is closer to the previous setpoint, the desired distance on the line is the fixed in the last computed
    float distance_marker_to_next_setpoint = sqrt(pow(drone->des_user_pos_GF.x - setpoint.x, 2) + pow(drone->des_user_pos_GF.y - setpoint.y, 2));
    float distance_marker_to_prev_setpoint = sqrt(pow(drone->des_user_pos_GF.x - setpoint_old.x, 2) + pow(drone->des_user_pos_GF.y - setpoint_old.y, 2));
    bool keep_des_pos_fixed = false;
    //Check also the difference between the coordinates of
    if (distance_marker_to_prev_setpoint < distance_marker_to_next_setpoint)
    {
      keep_des_pos_fixed = true;
      //Fin the ccordinates of the interception between the marjker point and the point line distance intersection (the projection
      //of the point on the line)
    }

    if (init_new_des_projected_pose == false)
    {
      drone_desired_position_along_line.x = drone->position_GF.x;
      drone_desired_position_along_line.y = drone->position_GF.y;
    }

    if (keep_des_pos_fixed == false)
    {

      float x_projection_value = 0.0;
      float y_marker_projection_on_line = 0.0;

      if (line_slope > 0 && drone->des_user_pos_GF.y > y_marker_on_line)
      {
        x_projection_value = d2 * cos(gamma) + drone->des_user_pos_GF.x;
        y_marker_projection_on_line = line_slope * x_projection_value + intercept;
      }
      else if (line_slope > 0 && drone->des_user_pos_GF.y < y_marker_on_line)
      {
        x_projection_value = drone->des_user_pos_GF.x - d2 * cos(gamma);
        y_marker_projection_on_line = line_slope * x_projection_value + intercept;
      }
      else if (line_slope < 0 && drone->des_user_pos_GF.y > y_marker_on_line)
      {
        x_projection_value = drone->des_user_pos_GF.x - d2 * cos(gamma);
        y_marker_projection_on_line = line_slope * x_projection_value + intercept;
      }
      else
      {
        x_projection_value = d2 * cos(gamma) + drone->des_user_pos_GF.x;
        y_marker_projection_on_line = line_slope * x_projection_value + intercept;
      }

      //Obtain finally the deisred projection of the user marker position on the line to follow

      if ((gamma < 0.05 && gamma > -0.05) || point_line_distance == 0)
      {
        y_marker_on_line = line_slope * drone->des_user_pos_GF.x + intercept;
        y_marker_projection_on_line = y_marker_on_line;
        cout << "//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////" <<endl;
        cout << "X proj value on the line " << drone->des_user_pos_GF.x << endl;
        cout << "Y proj value on the line " << y_marker_projection_on_line << endl;

        drone_desired_position_along_line.x = drone->des_user_pos_GF.x;
        drone_desired_position_along_line.y = y_marker_projection_on_line;
      }
      else
      {

        cout << "###################################################################################################################################################" <<endl;
        cout << "X proj value on the line " << x_projection_value << endl;
        cout << "Y proj value on the line " << y_marker_projection_on_line << endl;
        
        drone_desired_position_along_line.x = x_projection_value;
        drone_desired_position_along_line.y = y_marker_projection_on_line;
      }
        

    }
    
     cout << " IN  drone_desired_position_along_line.x " <<    drone_desired_position_along_line.x  << endl;
    cout << "IN drone_desired_position_along_line.y " <<    drone_desired_position_along_line.y << endl;  
    //
  }

  //Draw line to follow in Rviz
  void visualize_line_to_follow_in_rviz(arpl_drone_sim *drone, string frame_id)
  {
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = frame_id;
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "line_to_follow";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    float x = 0.0;
    float y = 0.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < setpoint_x_vec.size(); ++i)
    {

      if (i == 0)
      {
        x = initial_position.x;
        y = initial_position.y;
      }
      else
      {
        x = setpoint_x_vec[i];
        y = setpoint_y_vec[i];
      }
      // else if (i == setpoint_x_vec.size() + 1)
      // {
      //   x = goal.x;
      //   y = goal.y;
      // }
      // else
      // {
      //   x = setpoint_x_vec[i-1];
      //   y = setpoint_y_vec[i-1];
      // }

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = 0;

      points.points.push_back(p);
      line_strip.points.push_back(p);
    }

    if (drone->rrt_final_goal_reached == true)
    {
      points.points.clear();
      line_strip.points.clear();
    }

    drone->line_to_follow_marker.publish(points);
    drone->line_to_follow_marker.publish(line_strip);
  }




void visualize_virtual_obstacles_in_RVIZ(arpl_drone_sim *drone, string frame_id)
 {
    visualization_msgs::Marker obstacles;
    int size = drone->virtual_obstacles_vec.size();
    int split_size = size/2;
    
    int counter = 0;
    int publish_counter =0;
   
    
    for (int i = 0; i < split_size; i ++)
    {
       obstacles.header.frame_id = frame_id;
       obstacles.header.stamp = ros::Time::now();
    obstacles.ns = "virtual_obstacles";
    obstacles.action  = visualization_msgs::Marker::ADD;
    obstacles.pose.orientation.w = 1.0;

    obstacles.id = 0;

    obstacles.type = visualization_msgs::Marker::CUBE;
    //Obtain marker center given the position o fthe min and max vertex 
    float min_x = drone->virtual_obstacles_vec[counter].x;
    float min_y = drone->virtual_obstacles_vec[counter].y;

    float max_x = drone->virtual_obstacles_vec[counter + 1].x;
    float max_y = drone->virtual_obstacles_vec[counter + 1].y;
    
    float x_center = (max_x - min_x)/2;
    x_center = max_x -x_center;

    float y_center = (max_y - min_y)/2;
    y_center = max_y -y_center;
    
    
    geometry_msgs::Point p;
    p.x = x_center;
    p.y = y_center;
    p.z = 0;
    
    //obstacles.points.push_back(p);
    obstacles.pose.position.x = x_center;
    obstacles.pose.position.y = y_center;
    obstacles.pose.position.z = 0;
    obstacles.pose.orientation.x = 0.0;
    obstacles.pose.orientation.y = 0.0;
    obstacles.pose.orientation.z = 0.0;
    obstacles.pose.orientation.w = 1.0;
    
    p.x =  (max_x - min_x);
    p.y =  (max_y - min_y);
    p.z = 1.0;

    //obstacles.scale.push_back(p);
    obstacles.scale.x = (max_x - min_x);
    obstacles.scale.y = (max_y - min_y);
    obstacles.scale.z = 1.0;

    obstacles.color.r = 1.0;
    obstacles.color.g = 1.0;
    obstacles.color.b = 0.0;
    obstacles.color.a = 1.0;
    
    if (publish_counter == 0)
    {
      drone->virtual_obstacle_marker_1.publish(obstacles);
      publish_counter = publish_counter + 1;
      //cout << "SONO QUIII ----------------------" << endl;
    }
    else if (publish_counter == 1)
    {
      drone->virtual_obstacle_marker_2.publish(obstacles);
      publish_counter = publish_counter + 1;
      
    }
    else
    {
        drone->virtual_obstacle_marker_3.publish(obstacles);
        publish_counter = 0;
        drone->virtual_obstacles_vec.clear();
    }
    
    counter = counter + 2;
    }
   
   

 }



 
  //Evaluate the variable Damping value
  void evaluate_variable_damping_coefficient(arpl_drone_sim *drone)
  {

    //Evaluate the slope of the line to follow
    float m_l1 = (setpoint.y - setpoint_old.y) / (setpoint.x - setpoint_old.x);

    //Find the euqation fo line l2, parallel to l1, but passing through the drone position
    float intercept = drone->position_GF.y - m_l1 * drone->position_GF.x;

    // EValuate point line distance  between the user marker position and l2 line --> Remember: line l2 pass through the drone position and the point (0.0, intercept)
    float a1 = drone->position_GF.x - 0.0;
    float a2 = intercept - drone->des_user_pos_GF.y;

    float b1 = 0.0 - drone->des_user_pos_GF.x;
    float b2 = drone->position_GF.y - intercept;

    float d = sqrt(pow(a1, 2) + pow(b2, 2));
    if (d == 0.0)
    {
      d = 0.001;
    }
    float d_u_l2 = abs(a1 * a2 - b1 * b2) / (d); //diatance user point line l2

    //Evaluate the distance betwene the drone and the user marker (the length of the force vector)

    float user_marker_drone_distance = sqrt(pow(drone->des_user_pos_GF.y - drone->position_GF.y, 2) + pow(drone->des_user_pos_GF.x - drone->position_GF.x, 2));

    // #######################################################################
    //Find the in line y_projection of the marker position on the line l2
    float y_marker_on_line = m_l1 * drone->des_user_pos_GF.x + intercept; //intercept is from line l2
                                                                          //Evaluate the distance between the marker position and the point (drone->des_user_pos_GF.x, y_marker_on_line)
    float d1 = abs(drone->des_user_pos_GF.y - y_marker_on_line);
    float delta = atan(m_l1);
    float alfa = (C_PI / 2) - delta;
    float d2 = abs(d1 * cos(alfa));

    float x_projection_value = 0.0;
    float y_marker_projection_on_line = 0.0;

    float x_marker_proj_on_drone_line_l2 = 0.0;
    float y_marker_proj_on_drone_line_l2 = 0.0;

    if (m_l1 > 0 && drone->des_user_pos_GF.y > y_marker_on_line)
    {
      x_projection_value = d2 * cos(delta) + drone->des_user_pos_GF.x;
      y_marker_projection_on_line = m_l1 * x_projection_value + intercept;
    }
    else if (m_l1 > 0 && drone->des_user_pos_GF.y < y_marker_on_line)
    {
      x_projection_value = drone->des_user_pos_GF.x - d2 * cos(delta);
      y_marker_projection_on_line = m_l1 * x_projection_value + intercept;
    }
    else if (m_l1 < 0 && drone->des_user_pos_GF.y > y_marker_on_line)
    {
      x_projection_value = drone->des_user_pos_GF.x - d2 * cos(delta);
      y_marker_projection_on_line = m_l1 * x_projection_value + intercept;
    }
    else
    {
      x_projection_value = d2 * cos(delta) + drone->des_user_pos_GF.x;
      y_marker_projection_on_line = m_l1 * x_projection_value + intercept;
    }

    //Obtain finally the deisred projection of the user marker position on the line to follow

    if ((delta < 0.05 && delta > -0.05) || d_u_l2 == 0)
    {
      y_marker_on_line = m_l1 * drone->des_user_pos_GF.x + intercept;
      y_marker_projection_on_line = y_marker_on_line;

      x_marker_proj_on_drone_line_l2 = drone->des_user_pos_GF.x;
      y_marker_proj_on_drone_line_l2 = y_marker_projection_on_line;
    }
    else
    {

      x_marker_proj_on_drone_line_l2 = x_projection_value;
      y_marker_proj_on_drone_line_l2 = y_marker_projection_on_line;
    }

    //EValuate the distance along the line between the projection of the user marker point ij the line and the drone
    float d_proj_drone = sqrt(pow(y_marker_proj_on_drone_line_l2 - drone->position_GF.y, 2) + pow(x_marker_proj_on_drone_line_l2 - drone->position_GF.x, 2));

    //Evaluate the angle that identify the direction of the force respect the direction of the line to follow l2, passing throug the drone position.

    float gamma = acos(d_proj_drone / user_marker_drone_distance);
    if (gamma != gamma)
    {
      gamma = C_PI;
    }
    //From acos, gamma varies from o to 1.57 and down to zero.
    //It should increase up to pi if the distance between the projection of the amrker on the line and
    // the next setpoint is greater than the distance between the drone projection on the line and the next setpoint

    float distance_marker_to_next_setpoint = sqrt(pow(drone->des_user_pos_GF.x - setpoint.x, 2) + pow(drone->des_user_pos_GF.y - setpoint.y, 2));
    float distance_marker_to_prev_setpoint = sqrt(pow(drone->des_user_pos_GF.x - setpoint_old.x, 2) + pow(drone->des_user_pos_GF.y - setpoint_old.y, 2));

    //Check also the difference between the coordinates of
    if (distance_marker_to_prev_setpoint < distance_marker_to_next_setpoint)
    {
      gamma = C_PI / 2 + (C_PI / 2 - gamma);
    }

    //Here change the value of damping implemnting the functiion respect the gamma value
    //Supposing a value of d = [0, 100]
    // Implementing the linear, exponential and sqrt function. In the final version the user can select it from the config file

    float d_min = (float)D_MIN;
    float d_max = (float)D_MAX;

    float d_output = 0.0;

    if (damping_function == "linear")
    {
      float m_linear = (d_max - d_min) / (C_PI - 0);
      d_output = m_linear * gamma;
      cout << "[ADMITTANCE CONTROLLER] d_output_linear : " << d_output << endl;
    }
    else if (damping_function == "exponential")
    {
      //Exponential
      float a = pow(d_max, 1 / C_PI); //Find the basis of the exponnetial function
      d_output = pow(a, gamma);
      cout << "[ADMITTANCE CONTROLLER] d_output_exp : " << d_output << endl;
    }
    else
    {
      //Sqrt
      float a_sqrt = (d_max - d_min) / (sqrt(C_PI - 0.0));
      d_output = a_sqrt * sqrt(gamma - 0.0) + d_min;
      cout << "[ADMITTANCE CONTROLLER] d_output_sqrt : " << d_output << endl;
    }

    // Only for debug and plot
    drone->d_output = d_output;
    drone->gamma_output = gamma;
  }
};

#endif
