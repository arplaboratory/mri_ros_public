#ifndef _FEATURE_DETECTION_H_
#define _FEATURE_DETECTION_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace Eigen;

class Feature_detection_Impl;

class Feature_detection
{
    private:
    Feature_detection_Impl *Feature_detection_Implementation; 

    public:
    //Costruttore classe Feature_detection
    Feature_detection();
    //Eigen::Vector2f target_point_P1, target_point_P2 --> Real states 
    //Eigen::Vector2f obs_points_P1, obs_points_P1 --> observed points required to estimate the line parameters 
    void load_image(string path);
   
    
    ~Feature_detection();
};

#endif