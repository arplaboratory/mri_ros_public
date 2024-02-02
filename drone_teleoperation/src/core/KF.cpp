#ifndef _KF_SOURCE_
#define _KF_SOURCE_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "drone_teleoperation/core/KF.h"
//#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/Drone.h"


using namespace std;
using namespace Eigen;

namespace hri_admittance {
    KF::KF()
    : A(),
      H(),
      K(),
      G(),
      _Px(),
      P_(),
      P(),
      _R(),
      X(),
      X_(), 
      Z(),
      K_count(){}


double average(std::vector<double> const& v)
{
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<double>(v.size());
    return accumulate(v.begin(), v.end(), 0.0) / v.size();
}


void KF::calculate(double pos, double vel ,double acc, double dt)
{   
    //Define  KF Matrices for position, velocities filtering 
    
    H << 1.0, 0.0, 
        0.0, 1.0;
    
    
    A << 1, dt, 
        0, 1;
    double exp_2 = pow(dt,2);
    double exp_3 = pow(dt,3);
    double exp_4 = pow(dt,4);
    exp_4 = exp_4/4.0;
    exp_3 = exp_3/2.0;
    
    double mul = 0.5*exp_2;
    G << mul, 
    dt; 
    
    Q << exp_4, exp_3,
        exp_3, exp_2;
    
    
    double u = acc;
    double sigma_a = 0.1; //Random Variance 

    //Useful Matrices 
     Eigen::Matrix2d res;
     Eigen::Matrix2d res1;
     Eigen::Matrix2d ev1;
    Eigen::Matrix2d I;
    Eigen::Vector2d inov(0.0,0.0);
    if (K_count > 1)
    {
	I << 1, 0, 
	    0, 1;
       //Obtain an observation of the state as pos
       Z << pos, vel;
      // Update the Kalman Gain with the actual observations
       
       
       res = H * P_ * H.transpose() + _R;
       res1 = res.inverse().eval();
       K = P_ * H.transpose() * res1;
       // Estimate the current state +
       inov = Z - H * X_;
       X = X_ + K * inov;


       //Update the current estimate uncertanty
       ev1 = (I - K* H);
       P_ = ev1 * P_ * ev1.transpose() + (K*_R*K.transpose());

       //Predict Next State 
       X_ = A * X +  G*u;
       
       // Predict Next Covariance Matrix
       P  = A * P_ * A.transpose()  + Q*(sigma_a*sigma_a);
       P_ = P;
    }
    else
    {
      //Initialization phase 
      X_  = A * X + G*u;
      P_ = A * _Px * A.transpose() + Q*(sigma_a*sigma_a);
    }

     K_count = K_count + 1;

      //Averaging the state velocity as output
     avg_vel_vec.push_back(X_[1]);
      X_[1] = average(avg_vel_vec);
      //Clean the vector 
      if (avg_vel_vec.size() > 2)
      {
        avg_vel_vec.erase(avg_vel_vec.begin());
      }

     

}


void KF::pass_to_class_initialization_matrices(Eigen::Matrix2d Px, Eigen::Matrix2d R, Eigen::Vector2d X_init)
{
     
     _Px = Px;
     _R = R;
      X_ = X_init;
      X = X_init;
      K_count = 0;
     cout <<"[KALMAN FILTER] R Matrix initialization: " << _R << endl;
     cout <<"[KALMAN FILTER] Px Covariance Matrix Initialization: " << Px << endl;
    
}


// Kalman FIlter Output
Eigen::Vector2d KF::Obtain_KF_estimated_state()
{
  
    return X_;
}
}

#endif
