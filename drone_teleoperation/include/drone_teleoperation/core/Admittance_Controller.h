
#ifndef _Admittance_Controller_H_
#define _Admittance_Controller_H_

using namespace std;
namespace hri_admittance {



class Admittance_Controller
{
    public:
    Admittance_Controller(double Ks,
                   double Kd,
                   double F_max,
                   double F_min,
                   double M, 
                   double D, 
                   double K);
    virtual ~Admittance_Controller(){}

    void calculate(double des_value, double actual_value, double des_vel_value, double drone_vel, double f_obs, double dt); 
    void calculate_xc_from_planned_trajectory(double interaction_des_position, double des_position, double actual_position, double des_interaction_vel_value,  double des_trajectory_vel_value, double drone_vel, double f_obs, double d_val,  double dt);
    

    double obtain_evaluated_force();
    double obtain_commanded_position();
    double obtain_commanded_velocity();

    private:
    Eigen::Matrix2f  A;
   Eigen::Vector2f  B;
   Eigen::Vector2f  yz; // State Vector as code in matlab
   Eigen::Vector2f dydz;  // Derivative of the state Vector as code in matlab
   Eigen::Vector2f  yz_predict;  //Derivation of the state at time t+1

    //States 
    float y,z;

    double dt_;
    double Ks_;
    double Kd_;
    double F_max_, F_min_, F_;
    double M_, D_, K_; 

     //RK Gains 
    float K0, K1,K2, K3;
    float L0, L1, L2, L3;


    //Error 
    double e_t = 0.0;
    double e_t_old = 0.0;
    double e_dot_t = 0.0;
    double e_dot_t_old = 0.0;

    double x_dot_c; 
    double x_c;

    bool init = true;
    
};
}
#endif