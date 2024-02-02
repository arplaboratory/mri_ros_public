#include "ros/ros.h"
#include </home/arpl/luca_ws/devel/include/scene_understanding_pkg_msgs/MeshPos.h>
#include <stdio.h> 
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <sys/stat.h> 
#include <sstream>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <algorithm>
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



#include </home/luca/luca_ws/src/scene_understanding_pkg/include/Kimera_vio_data.h>
#include </home/luca/luca_ws/src/scene_understanding_pkg/include/SU_Unity_comm.h>
#include </home/luca/luca_ws/src/scene_understanding_pkg/include/Feature_detection.h>

using namespace std::chrono;
using namespace std;
using namespace Eigen;


#define C_PI (double)3.141592653589


/*
void load_image(string path, cv::Mat *baseline_image)
{  
    baseline_image = imread(path);
    //Convert in grayscale 
    if (baseline_image.channels() != 1) {
        cvtColor(&baseline_image, &baseline_image, cv::COLOR_RGB2GRAY);
    }
    //Resize
    Mat inImg = &baseline_image;
    resize(inImg, baseline_image, Size(), 0.45, 0.45);
   
    string windowName = "Baseline Image"; //Name of the window
    namedWindow(windowName); // Create a window 
    imshow(windowName, &baseline_image); // Show our image inside the created window.
    waitKey(0); // Wait for any keystroke in the window
    destroyWindow(windowName); //destroy the created window
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kimeradata");
    ros::NodeHandle nh;
    
    Kimera_vio_data kimera_data;
    Feature_detection FD = Feature_detection();

    FD.pass_to_class_baseline_image_path("/home/arpl/luca_ws/src/scene_understanding_pkg/src/Feature_detector/images/baseline_image.png");
    
    
    FD.load_image();
    cv::Mat baseline_image = FD.obtain_baseline_image();
    /*
    string windowName = "Baseline Image"; //Name of the window
    namedWindow(windowName); // Create a window 
    imshow(windowName, baseline_image); // Show our image inside the created window.
    waitKey(0); // Wait for any keystroke in the window
    destroyWindow(windowName); //destroy the created window
    */

    
    ros::Rate r(30);
    //La image bnaseline deve essere della stessa dimensione della current frame image 
    while (nh.ok())
    {
       
       if  (kimera_data.flagRealSenseNewColorFrame == true && kimera_data.new_color_frame.rows > 0)
       {
        //Convert image to Black and white and resize
       
        if (kimera_data.new_color_frame.channels() != 1) {
            cv::cvtColor( kimera_data.new_color_frame,   kimera_data.bw_resized_frame, cv::COLOR_BGR2GRAY);
        }
        cv::resize(kimera_data.bw_resized_frame, kimera_data.new_color_frame, cv::Size(), 0.75, 0.75);
        //if a new image is available --> send it to to the detector 
        string algorithm = "FAST";
        FD.feature_detector(algorithm, baseline_image, kimera_data.bw_resized_frame);
       }
    
    
        //cv::waitKey(2); // Wait for any keystroke in the window
        //cv::destroyWindow(windowName); //destroy the created window
        //Tracking new feature from new images respect the baseline 
       //FD.feature_tracking_from_new_image(last_acquired_frame);
    
       kimera_data.flagRealSenseNewColorFrame = false;
       ros::spinOnce();
       r.sleep();
       
       
    }

    

    return 0;
}



