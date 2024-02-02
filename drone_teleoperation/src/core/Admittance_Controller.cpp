#ifndef _Admittance_Controller_SOURCE_
#define _Admittance_Controller_SOURCE_

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "drone_teleoperation/core/Admittance_Controller.h"



using namespace std;
namespace hri_admittance {
    Admittance_Controller::Admittance_Controller(double Ks,
                   double Kd,
                   double F_max,
                   double F_min,
                   double M, 
                   double D, 
                   double K)
                   : Ks_(Ks),
                     Kd_(Kd),
                     F_max_(F_max),
                     F_min_(F_min),
                     F_(),
                     M_(M),
                     D_(D),
                     K_(K),
                     x_dot_c(),
                     x_c(),
                     yz(),
                     dydz(),
                     A(),
                     B(),
                     y(),
                     z(),
                     K0(),
                     K1(),
                     K2(),
                     K3(),
                     L0(),
                     L1(),
                     L2(),
                     L3(),
                     yz_predict(),
                     init(){}


void Admittance_Controller::calculate(double des_position, double actual_position, double des_vel_value, double drone_vel, double f_obs, double dt)
{  
    //Here the desired position is the same of the interactive cube position 
    //Evaluate Force acting on x axis
    //evaluate_force(des_position, actual_position, F_, KS);
     float ks = Ks_/2.0;
     F_ = ks * abs(actual_position - des_position) + Kd_ * des_vel_value - f_obs;

    if (init)
    {
       A << 0.0, 1.0, -K_/M_, -D_/M_;
       B << 0.0, 1/M_;
       yz << 0.0, 0.0; 
       init = false;
    }
    else
    {
      
        A << 0.0, 1.0, -K_/M_, -D_/M_;
        B << 0.0, 1/M_;
        yz << y, z;
    }
    
    //First Step: Evaluate K0 K1
    dydz =  A * yz + B*F_; 
    
    cout << "F Interaction: " <<  F_<< endl;
   
    K0 = dt * dydz[0];
    L0 = dt * dydz[1];
    
   
    float yz_0 = yz[0] + 1/2* K0; //y + 1/2*K0
    float yz_1 = yz[1] + 1/2* L0; //z + 1/2*L0
   
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;

    //Evaluation K1, L1
    K1 = dt *dydz[0];
    L1 = dt *dydz[1];
 
    yz_0 = yz[0]+ 1/2* K1; 
    yz_1 = yz[1] + 1/2* L1; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;

    //Evaluation K2, L2
    K2 = dt *dydz[0];
    L2 = dt *dydz[1];
   
    yz_0 = yz[0]+ 1/2* K2; 
    yz_1 = yz[1] + 1/2* L2; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;
    
    //Evaluation K3, L3
    K3 = dt *dydz[0];
    L3 = dt *dydz[1];

    yz_0 = yz[0] + 1/2* K3; 
    yz_1 = yz[1] + 1/2* L3; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;
    //Prediction step
    float yz_predict_0 = y + 1/6.0*(K0 + 2*K1 + 2*K2 + K3);
    float yz_predict_1 = z + 1/6.0*(L0 + 2*L1 + 2*L2 + L3);
    //Value of the state at time t+1
    yz_predict <<  yz_predict_0, yz_predict_1;
   
    //Value for the next iterations 
    y = yz_predict[0];
    z = yz_predict[1];

    

    //Comanded Position and Velocity
    x_c = des_position - (double)y;
    x_dot_c = des_vel_value - (double)z;
    
    cout <<"x_c: " << x_c << " x_dot_c: " << x_dot_c << endl;
    /*
    double a1 = F_- K_*e_t_old;
    double a2 = a1 - D_*e_dot_t_old;
    double a3 = 1/M_ * (a2);
    e_dot_t += a3 * dt_;
    e_t += e_dot_t*dt_;
    x_dot_c = des_vel_value - e_dot_t;
    //x_dot_c = drone_vel - e_dot_t;
    x_c = des_position - e_t; 
    //x_c = actual_position - e_t; 
    
     float K1;
     float K2;
     float K3;

     float L0;

    */
}





void Admittance_Controller::calculate_xc_from_planned_trajectory(double interaction_des_position, double des_position, double actual_position, double des_interaction_vel_value, double des_trajectory_vel_value, double drone_vel, double f_obs, double d_eval,  double dt)
{  
     /*

    */ 
    //Evaluate Force acting on x axis
    //evaluate_force(des_position, actual_position, F_, Ks_);
   
    F_ = Ks_ * (actual_position - interaction_des_position) + Kd_ * des_interaction_vel_value - f_obs;
    //F_ = Ks_ *(des_position - interaction_des_position) + Kd_ * des_interaction_vel_value;
    D_ = d_eval;
    if (init)
    {
       A << 0.0, 1.0, -K_/M_, -D_/M_;
       B << 0.0, 1/M_;
       yz << 0.0, 0.0; 
       init = false;
    }
    else
    {
      
        A << 0.0, 1.0, -K_/M_, -D_/M_;
        B << 0.0, 1/M_;
        yz << y, z;
    }
    
    //First Step: Evaluate K0 K1
    dydz =  A * yz + B*F_; 
    
    cout << "F_: " <<  F_<< endl;
   
    K0 = dt * dydz[0];
    L0 = dt * dydz[1];
    
   
    float yz_0 = yz[0] + 1/2* K0; //y + 1/2*K0
    float yz_1 = yz[1] + 1/2* L0; //z + 1/2*L0
   
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;

    //Evaluation K1, L1
    K1 = dt *dydz[0];
    L1 = dt *dydz[1];
 
    yz_0 = yz[0]+ 1/2* K1; 
    yz_1 = yz[1] + 1/2* L1; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;

    //Evaluation K2, L2
    K2 = dt *dydz[0];
    L2 = dt *dydz[1];
   
    yz_0 = yz[0]+ 1/2* K2; 
    yz_1 = yz[1] + 1/2* L2; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;
    
    //Evaluation K3, L3
    K3 = dt *dydz[0];
    L3 = dt *dydz[1];

    yz_0 = yz[0] + 1/2* K3; 
    yz_1 = yz[1] + 1/2* L3; 
    yz << yz_0, yz_1;
    dydz = A * yz + B*F_;
    //Prediction step
    float yz_predict_0 = y + 1/6.0*(K0 + 2*K1 + 2*K2 + K3);
    float yz_predict_1 = z + 1/6.0*(L0 + 2*L1 + 2*L2 + L3);
    //Value of the state at time t+1
    yz_predict <<  yz_predict_0, yz_predict_1;
   
    //Value for the next iterations 
    y = yz_predict[0];
    z = yz_predict[1];

    

    //Comanded Position and Velocity
    x_c = des_position - (double)y;
    x_dot_c = des_trajectory_vel_value - (double)z;
    
    cout <<"x_c: " << x_c << "x_dot_c: " << x_dot_c << endl;
    
    //cout << "AC dt: " << dt << endl;
    /*
    double a1 = F_- K_*e_t_old;
    double a2 = a1 - D_*e_dot_t_old;
    double a3 = 1/M_ * (a2);
    e_dot_t += a3 * dt_;
    e_t += e_dot_t*dt_;
    x_dot_c = des_vel_value - e_dot_t;
    //x_dot_c = drone_vel - e_dot_t;
    x_c = des_position - e_t; 
    //x_c = actual_position - e_t; 
    
    e_t_old = e_t;
    e_dot_t_old = e_dot_t;

    */
}


double Admittance_Controller::obtain_evaluated_force()
{
   return F_;
}

double Admittance_Controller::obtain_commanded_position()
{
    return x_c;
}

double Admittance_Controller::obtain_commanded_velocity()
{
    return x_dot_c;
}

}

#endif