#ifndef UTILS_H_
#define UTILS_H_
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace perception {
  
class PerceptionUtils
{
    public:
    PerceptionUtils(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~PerceptionUtils(){}
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    float align_theta_with_yaw(float theta,float dx, float dy);
    float average_theta_val(float theta, std::vector<float> *theta_vec);
    float compute_theta_dot(float theta_min_distance, float theta_min_distance_old, float dt);
    int getIndex(std::vector<float> *v, float K);
    void saturate_force_value(float *dx_sat, float *dy_sat,  float dx_sum, float dy_sum, float sat_x_value, float sat_y_value);
    void average_marker_postions_on_multiple_stamps(float *fx_filter, float *fy_filter, float fx, float fy);
    void euler_to_quat(Eigen::Vector4f *quat, float roll, float pitch, float yaw);

    int avg_stamp = 10;
    std::vector<float> fx_vec;
    std::vector<float> fy_vec; 
    //private:
 

};

}
#endif


