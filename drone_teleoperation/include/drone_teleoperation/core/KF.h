#ifndef _KF_H_
#define _KF_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <numeric>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>



using namespace std;
using namespace Eigen;
namespace hri_admittance {
class KF
{
    public:
    KF();
    virtual ~KF(){}
    
       
    void calculate(double position, double velocity, double input_acc, double dt);
    void pass_to_class_initialization_matrices(Eigen::Matrix2d Px, Eigen::Matrix2d R,  Eigen::Vector2d X_init);
    Eigen::Vector2d Obtain_KF_estimated_state();
    geometry_msgs::Vector3 state_est;

    private:
    //KF Matrices 
    Eigen::Matrix2d  A;
    Eigen::Matrix2d  H;
    Eigen::Matrix2d  K;
    Eigen::Matrix2d  Q;
    Eigen::Matrix2d _Px;
    Eigen::Matrix2d P_;
    Eigen::Matrix2d P;
    Eigen::Matrix2d _R;
    Eigen::Vector2d  G;
    Eigen::Vector2d  X;
    Eigen::Vector2d  X_; //Updated state
    Eigen::Vector2d  Z; //Obs matrix
    int K_count = 0;
    
    vector<double> avg_vel_vec;


};
}
#endif  
