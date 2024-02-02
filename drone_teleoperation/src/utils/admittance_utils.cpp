#include "drone_teleoperation/utils/admittance_utils.h"

#include <vector>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <numeric>

namespace hri_admittance {

  AdmittanceUtils::AdmittanceUtils(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private)
                        : visualizer(nh, nh_private),
                          P1(),
                          P2()
                        {

            nh_.param("/tele_control_params/safety_boundaries/P1/x",  P1(0), P1(0));
            nh_.param("/tele_control_params/safety_boundaries/P1/y",  P1(1), P1(1));
            nh_.param("/tele_control_params/safety_boundaries/P2/x",  P2(0), P2(0));
            nh_.param("/tele_control_params/safety_boundaries/P2/y",  P2(1), P2(1));
                
  }

std::string AdmittanceUtils::check_frames(int scenario)
{
    std::string frame_id; 
    if (scenario == 1 || scenario == 2) // simulation case
   {
     frame_id = "/simulator";
     publish_tf_transform_from_world_to_sim();
     
   }
   else
   {
    frame_id = "mocap";
   }

   return frame_id;
} 


double AdmittanceUtils::average(std::vector<float>  *v){
    if(v->empty()){
        return 0;
    }
    auto const count = static_cast<float>(v->size());
    return accumulate(v->begin(), v->end(), 0.0) / v->size();
}



float AdmittanceUtils::check_des_yaw(bool increasing_yaw, bool decreasing_yaw, float yaw, float d_yaw )
{  
   float des_yaw = d_yaw;
   float angle_rad_horizon = 10*M_PI/180;

   if (increasing_yaw)
   {
    if (counter_yaw_rate == 0)
    {
      yaw_drone_initial =  yaw;
      if (yaw_drone_initial > 170*M_PI/180 )
      {
        //Case when the final angle is bigger that 180
         float step  = M_PI - (yaw_drone_initial + angle_rad_horizon);
         final_angle = - M_PI - step;
      }
      else
      {
        final_angle = yaw_drone_initial + angle_rad_horizon;
      }
    }
    des_yaw =  yaw + 0.1;
    
    if (yaw > final_angle)
    {
       increasing_yaw = false;
       counter_yaw_rate = 0;
    }
    else
    {
        counter_yaw_rate = counter_yaw_rate + 1;
    }
   }

  if (decreasing_yaw)
  {
    if (counter_yaw_rate == 0)
    {
      yaw_drone_initial =  yaw;
      if (yaw_drone_initial < -170*M_PI/180 )
      {
        //Case when the final angle is bigger that 180
         float step  = -M_PI - (yaw_drone_initial - angle_rad_horizon);
         final_angle =  M_PI - step;
      }
      else
      {
        final_angle = yaw_drone_initial - angle_rad_horizon;
      }
     
    }

    
   des_yaw =  yaw - 0.1;
    if (yaw < final_angle)
    {
       decreasing_yaw = false;
       counter_yaw_rate = 0;
    }
    else
    {
        counter_yaw_rate = counter_yaw_rate + 1;
    }
   
  }

return des_yaw;
}

bool AdmittanceUtils::check_safety_boundaries(Eigen::Vector3f quad_pos)
  {
    bool enable_hovering = false;
    if ( quad_pos(0) >= P1(0))
    {
       enable_hovering = true;
    }
    else if (quad_pos(0) < P2(0)){
       enable_hovering = true;
    }
    if (quad_pos(1) <= P1(1)){
       enable_hovering = true;
    }
    else if (quad_pos(1) > P2(1)){
       enable_hovering = true;
    }

    //Publish visualization marker for the safety boundaries in Rviz
     visualizer.publish_safety_boundaries_visualization(P1, P2);
    return enable_hovering;
}


Eigen::Vector3f  AdmittanceUtils::check_force_direction_outside_borders(Eigen::Vector3f F_admittance,Eigen::Vector3f  quad_pos,Eigen::Vector3f marker_pos)
{
  Eigen::Vector3f F_a = Eigen::Vector3f(0.0, 0.0, 0.0);
  //Check force direction outside borders 
  float fx = F_admittance(0);
  float fy = F_admittance(1);
  if ( quad_pos(0) >= P1(0))
    {
    
       if (marker_pos(1) > quad_pos(1))
       {
          fy = -1*F_admittance(1);
       }
        fx = -1*F_admittance(0);
    }
  
  if (quad_pos(0) < P2(0)){
    if (marker_pos(1) > quad_pos(1))
       {
          fy = -1*F_admittance(1);
       }
        fx = F_admittance(0);
    }
  

  if (quad_pos(1) <= P1(1)){
    if (marker_pos(0) > quad_pos(0))
       {
          fx = -1*F_admittance(0);
       }
        fy = F_admittance(1);
    }
  
  if (quad_pos(1) > P2(1)){
    if (marker_pos(0) > quad_pos(0))
       {
          fx = -1*F_admittance(0);
       }
        fy = -1*F_admittance(1);
  }
   F_a = Eigen::Vector3f(0.4*fx, 0.4*fy, F_admittance(2));
  return F_a;
}



float AdmittanceUtils::yaw_tracker(float yaw, float yaw_des,bool * yaw_flag, float dt)
{
  /*
  This function create a simple tracker to change the yaw desired orientation.
  This is define din the world frame  
  */
  // if (yaw_traj_track_start)
  // {
  //   yaw_traj_track_start = false;
  //   yaw_traj_track_end = false;
  //   yaw_des_init = yaw_des;
  //   yaw_des_tracker = yaw;
  //   yaw_init = yaw;

    

  // }
yaw_offset = yaw_des - yaw;
if (yaw > 0.0 && yaw_des <= M_PI &&  yaw_des >= 0.0)
        if (yaw_offset > 0){

            increase = true;
        }
        else{
          increase = false;
        }
        
  if (yaw > 0.0 && yaw_des < 0 && yaw_des >= -M_PI){
      if (yaw_offset < -M_PI)
      {
        yaw_offset = yaw_des +yaw;
        increase = true;
      }
      else
      {
        increase = false;
      }
  }

  if (yaw <= 0 && yaw_des > 0 && yaw_des <= M_PI ){
      if (yaw_offset > M_PI)
      {
         
         increase = false;
      }
      else
      {
        increase = true;
      }
  }

  if (yaw < 0.0 && yaw_des >= -M_PI && yaw_des <= 0.0)
  {
   
    if (yaw_offset > 0)
    {
       increase = true;
    }
    else{
      increase = false;
    }

  }

  float rad_per_second = 0.1;
  float dx = 0.1; //rad_per_second*dt;
   if (dx > 0.1)
      dx = 0.1;
  
  if (increase == false)
  {
    dx = - dx;
  }


  // cout << "[UTILS] Yaw Des " << yaw_des << endl;
  // cout << "[UTILS] yaw " << yaw << endl;
  // cout << "[UTILS] Yaw Offset " << yaw_offset << endl;
  // cout << "[UTILS] dx " << dx << endl;
  // cout << "[UTILS] increase " << increase << endl;
  if (abs(yaw_offset) < 0.2)
  {
    // yaw_traj_track_end = true;
    // yaw_traj_track_start = true;
    yaw_des_tracker = yaw_des;
    *yaw_flag = false; 
    
  }
  else
  {
    //Overlaying the fastest rotation with always a positive increment dx 
     dx = 0.10;
    yaw_des_tracker = yaw + dx;
  }
  cout << "[UTILS] yaw_des_tracker " << yaw_des_tracker << endl;

  return yaw_des_tracker;
}

void AdmittanceUtils::publish_tf_transform_from_world_to_sim()
{
    ros::Time current_time_tf, last_time_tf;
  current_time_tf = ros::Time::now();
  last_time_tf = ros::Time::now();
  
  geometry_msgs::Quaternion quat;
  quat.w = 1.0; 

   geometry_msgs::Vector3 zero_pos;
 
   
  geometry_msgs::TransformStamped world_to_map;
  geometry_msgs::TransformStamped map_to_mocap;
  geometry_msgs::TransformStamped mocap_to_mocap_sr;
  geometry_msgs::TransformStamped mocap_sr_to_simulator;


  

  world_to_map.header.stamp = current_time_tf;
  map_to_mocap.header.stamp= current_time_tf;
  mocap_to_mocap_sr.header.stamp = current_time_tf;
  mocap_sr_to_simulator.header.stamp = current_time_tf;
  
  
  world_to_map.header.frame_id = "world";
  world_to_map.child_frame_id = "map"; //drone31/cam1_link
  map_to_mocap.header.frame_id = "map";
  map_to_mocap.child_frame_id = "mocap";
  mocap_to_mocap_sr.header.frame_id = "mocap";
  mocap_to_mocap_sr.child_frame_id = "mocap_sr";
  mocap_sr_to_simulator.header.frame_id = "mocap_sr";
  mocap_sr_to_simulator.child_frame_id = "simulator";
  
  world_to_map.transform.translation = zero_pos;
  map_to_mocap.transform.translation = zero_pos;
  mocap_to_mocap_sr.transform.translation = zero_pos;
  mocap_sr_to_simulator.transform.translation = zero_pos;



   //Translation from base link to camera link (rigidly attached)
  world_to_map.transform.rotation = quat;
  map_to_mocap.transform.rotation = quat;
  mocap_to_mocap_sr.transform.rotation = quat;
  mocap_sr_to_simulator.transform.rotation = quat;

  world_to_map_tf.sendTransform(world_to_map);
  map_to_mocap_tf.sendTransform(map_to_mocap);
  mocap_to_mocap_sr_tf.sendTransform(mocap_to_mocap_sr);
  mocap_sr_to_simulator_tf.sendTransform(mocap_sr_to_simulator);

}

//Line Tracker to link two different position especially during the transitions 
void AdmittanceUtils::line_tracker(Eigen::Vector3f quad_pos, float dt)
{ 
    if (traj_track_start == true)
    {
      pos_ = quad_pos;
      start_ = pos_;
      traj_track_start = false;
      trajectory_ended = false;
      tracker_in_action = true;
    }
     // Record distance between last position and current.
    const float dx = Eigen::Vector3f((pos_(0) - quad_pos(0)), (pos_(1) -quad_pos(1)), (pos_(2) - quad_pos(2))).norm();
    pos_ = quad_pos;
    
    current_traj_duration_ += dt;
    current_traj_length_ += dx;

    if (trajectory_ended)
    {
    curr_des_pos = goal_pose_tracker;
    curr_des_vel(0) = 0.0, curr_des_vel(1) = 0.0, curr_des_vel(2) = 0.0; 
    curr_des_acc(0) = 0.0, curr_des_acc(1) = 0.0, curr_des_acc(2) = 0.0; 

    trajectory_ended = true;
    tracker_in_action = false;
    return;
    }
    float v_des_ = 0.2;
    float a_des_ = 0.2;
    float epsilon_ = 0.10;
    const Eigen::Vector3f dir = (goal_pose_tracker - start_).normalized();
    const float total_dist = (goal_pose_tracker - start_).norm();
    const float d = (pos_ - start_).dot(dir);
    const Eigen::Vector3f proj = start_ + d * dir;
    const float v_max = std::min(std::sqrt(a_des_ * total_dist), v_des_);
    const float ramp_dist = v_max * v_max / (2 * a_des_);
    
    Eigen::Vector3f x(pos_), v(Eigen::Vector3f::Zero()), a(Eigen::Vector3f::Zero());

    if ((pos_ - goal_pose_tracker).norm() <= epsilon_) // Reached goal
    {


    current_traj_duration_ = 0.0;
    current_traj_length_ = 0.0;


    ROS_DEBUG_THROTTLE(1, "Reached goal");
    a = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    x = goal_pose_tracker;
    trajectory_ended = true;
    tracker_in_action = false;
   }
    else if (d > total_dist) // Overshoot
  {
    ROS_DEBUG_THROTTLE(1, "Overshoot");
    a = -a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = goal_pose_tracker;
  }
  else if (d >= (total_dist - ramp_dist) && d <= total_dist) // Decelerate
  {
    ROS_DEBUG_THROTTLE(1, "Decelerate");
    a = -a_des_ * dir;
    v = std::sqrt(2 * a_des_ * (total_dist - d)) * dir;
    x = proj + v * dt + 0.5 * a * dt * dt;
  }
   else if (d > ramp_dist && d < total_dist - ramp_dist) // Constant velocity
  {
    ROS_DEBUG_THROTTLE(1, "Constant velocity");
    a = Eigen::Vector3f::Zero();
    v = v_max * dir;
    x = proj + 2.3*(v * dt);
  }
  else if (d >= 0 && d <= ramp_dist) // Accelerate
  {
    ROS_DEBUG_THROTTLE(1, "Accelerate");
    a = a_des_ * dir;
    v = std::sqrt(2 * a_des_ * d) * dir;
    x = proj + 2.3*(v * dt) + 0.5 * a * dt * dt;
  }
  else if (d < 0)
  {
    ROS_DEBUG_THROTTLE(1, "Undershoot");
    a = a_des_ * dir;
    v = Eigen::Vector3f::Zero();
    x = start_ + 0.5 * a * dt * dt;
  }
  
  curr_des_pos = x;
  curr_des_vel = v;
  curr_des_acc = a;

     return ; 
} 


void AdmittanceUtils::safety_admittance_desired_position_transition()
{
  
  enable_transition = false;
  float dx = 0.01; //step on the line 
  float x_diff, y_diff = 0.0;
  x_diff = start_pos_for_transition(0) - final_position_for_transition(0);
  y_diff = start_pos_for_transition(1) - final_position_for_transition(1);
  float distance_2D_init =  sqrt(pow(x_diff,2) + pow(y_diff,2));
  float step_x = abs((dx*x_diff)/distance_2D_init);
  float step_y = abs((dx*y_diff)/distance_2D_init);
  
  cout << "start_pos_for_transition: " << start_pos_for_transition << endl;
  cout << "final_position_for_transition: " << final_position_for_transition << endl;
  cout << "step_x: " << step_x << endl;
  cout << "step_y: " << step_y << endl;
  cout << "x_diff: " << x_diff << endl;
  cout << "y_diff: " << y_diff << endl;
  cout << "distance_2D_init: " << distance_2D_init << endl;

 float x_val, y_val =0.0;
 
    if (x_diff < 0)
  {
    sum_x = sum_x + step_x;
    
  }
  else
  {
    sum_x = sum_x - step_x;
    x_val = sum_x + start_pos_for_transition(0);
   
  }

  if (y_diff < 0)
  {
    sum_y = sum_y + step_y;
    y_val = sum_y - start_pos_for_transition(1);
  }
  else
  {
    sum_y = sum_y - step_y;
    y_val = sum_y + start_pos_for_transition(1);
  }

 

  // x_diff =  sum_x - start_pos_for_transition(0);
  // y_diff =  sum_y - start_pos_for_transition(1);

  float distance_2D = sqrt(pow(sum_x,2) + pow(sum_y,2));
  cout << "sum_x: " << sum_x << endl;
  cout << "sum_y: " << sum_y << endl;
  cout << "distance_2D: " << distance_2D << endl;
  interpolation_updated_position = Eigen::Vector3f(start_pos_for_transition(0)+sum_x,start_pos_for_transition(0)+sum_y, final_position_for_transition(2));

  if (abs(distance_2D_init - distance_2D) < 0.10 )
  {
    enable_transition = true;
    sum_x =0.0;
    sum_y = 0.0;
    counter_transition = 0;
  }
  
  cout << "[safety_admittance_desired_position_transition] In Transition" << endl;

}

}




