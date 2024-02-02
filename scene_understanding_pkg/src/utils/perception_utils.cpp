#include "perception_utils.h"
#include <vector>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <numeric>

namespace perception {

  PerceptionUtils::PerceptionUtils(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private){}
    

float PerceptionUtils::align_theta_with_yaw(float theta, float dx, float dy)
{
  float theta_ = 0.0;
  float alfa = 0.0;
  float sigma = 0.0;

  //Robot is up right respect frame S of the point
   if (dx < 0.0 && dy < 0.0 )
   {
      alfa = theta - M_PI/2;
   }
   else if (dx < 0.0 && dy > 0.0 )
   {
    //Robot is down right respect frame S of the point
    alfa = -1*(M_PI/2 - theta); 
   }
   else if (dx > 0.0 && dy < 0.0)
   {
    //Robot is up left respect frame S of the point
    sigma = M_PI - theta;
    alfa = M_PI/2 + sigma;
   }
   else 
   {
     alfa = - M_PI/2 - theta;
   }

   return alfa;
}

double average(std::vector<float>  *v){
    if(v->empty()){
        return 0;
    }

    auto const count = static_cast<float>(v->size());
    return accumulate(v->begin(), v->end(), 0.0) / v->size();
}

float PerceptionUtils::average_theta_val(float theta, std::vector<float> *theta_vec)
{
  
  int avg_stamp_ = avg_stamp*3;
          // AVeraging Position along X axis 
  if (theta > 3.00 || theta < -2.90)
  {
    return theta;
  }
  theta_vec->push_back(theta);
  float theta_ = average(theta_vec);
  
  //Clean the vector 
  if (theta_vec->size() > avg_stamp)
  {
    theta_vec->erase(theta_vec->begin());
  } 

  return theta_;
}

float  PerceptionUtils::compute_theta_dot(float theta_min_distance, float theta_min_distance_old, float dt)
{
   float theta_dot = (theta_min_distance - theta_min_distance_old)/dt;
   return theta_dot;
}

int PerceptionUtils::getIndex(std::vector<float> *v, float K)
{
 auto it = find(v->begin(), v->end(), K);
 int index = it - v->begin();
 return index;
   
}

 void PerceptionUtils::average_marker_postions_on_multiple_stamps(float *fx_filter, float *fy_filter, float fx, float fy)
    {
    
      // AVeraging Position along X axis 
      fx_vec.push_back(fx);
      *fx_filter = average(&fx_vec);
      //Clean the vector 
      if (fx_vec.size() > avg_stamp)
      {
        fx_vec.erase(fx_vec.begin());
      } 
    
      // AVeraging Position along Y axis 
      fy_vec.push_back(fy);
      *fy_filter = average(&fy_vec);
      //Clean the vector 
      if (fy_vec.size() > avg_stamp)
      {
        fy_vec.erase(fy_vec.begin());
      } 
    
    
    }

void PerceptionUtils::saturate_force_value(float *dx_sat, float *dy_sat,  float dx_sum, float dy_sum, float sat_x_value, float sat_y_value)
{
   
   if (dx_sum > sat_x_value)
   {
     *dx_sat = sat_x_value;
   }
   else if (dx_sum < -sat_x_value)
   {
    *dx_sat= -sat_x_value;
   }
   else
   {
     *dx_sat = dx_sum;
   }

   if (dy_sum > sat_y_value)
   {
     *dy_sat = sat_y_value;
   }
   else if (dy_sum < -sat_y_value)
   {
    *dy_sat= -sat_y_value;
   }
   else
   {
    *dy_sat = dy_sum;
   }

 
}

void PerceptionUtils::euler_to_quat(Eigen::Vector4f *quat, float roll, float pitch, float yaw)
{
  float qx, qy, qz, qw;
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

  *quat << qx, qy, qz, qw;
}

}
