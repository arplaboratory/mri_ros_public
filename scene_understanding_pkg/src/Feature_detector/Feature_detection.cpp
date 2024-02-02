#ifndef _FEATURE_DETECTION_SOURCE_
#define _FEATURE_DETECTION_SOURCE_

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
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include </home/luca/luca_ws/src/scene_understanding_pkg/include/Feature_detection.h>

using namespace std;
using namespace Eigen;



class Feature_detection_Impl
{
    private:
    string path_; 
    cv::Mat baseline_image_; 
    cv::Mat reference_image_;
    cv::Mat current_frame_;
    
    //images with detected keypoints
    
    vector<cv::KeyPoint> ktps_1;
    vector<cv::KeyPoint> ktps_2;

    public:
    
    

    //Class constructor, prende i termini dati in input alla class PID
    Feature_detection_Impl();
    
    void load_image();
    void feature_detector(string alg, cv::Mat reference_image, cv::Mat current_frame);

    //Value pass to Class
    void pass_to_class_baseline_image_path(string path);   
    
    //from class 
    cv::Mat obtain_baseline_image();
    //Destructor
     ~Feature_detection_Impl()
    {

    }
    
};

//Class PanelImpl call from header
Feature_detection::Feature_detection()
{
    Feature_detection_Implementation = new Feature_detection_Impl();
}

//###################  FUNCTION WITH COMPUTTATION 
//Function dcall
void Feature_detection::load_image()
{
    return Feature_detection_Implementation -> load_image();
}

void Feature_detection::feature_detector(string alg, cv::Mat reference_image, cv::Mat current_frame)
{
    return Feature_detection_Implementation -> feature_detector(alg, reference_image, current_frame);
}


void Feature_detection::pass_to_class_baseline_image_path(string path)
{
    return Feature_detection_Implementation -> pass_to_class_baseline_image_path(path);
}


cv::Mat Feature_detection::obtain_baseline_image()
{
    return Feature_detection_Implementation -> obtain_baseline_image();
}


//Il distruttore dealloca anche la classe Panel
Feature_detection::~Feature_detection() 
{
    delete Feature_detection_Implementation;
}

/* Implementation */

Feature_detection_Impl::Feature_detection_Impl():
//Definisco TUTTE le variabiliche verranno utilizzate nelle funzioni e definite in PanelImpl
//Le variabili passate con costruttore devono essere rinominate e passate al file cpp definendole in private
path_(),
baseline_image_(),
reference_image_(),
current_frame_(),
ktps_1(),
ktps_2()
{
}

void Feature_detection_Impl::load_image()
{    
   baseline_image_ = cv::imread(path_);
   //Convert in grayscale 
    if (baseline_image_.channels() != 1) {
        cv::cvtColor(baseline_image_, baseline_image_, cv::COLOR_RGB2GRAY); //In, Out 
    }
    //Resize
    //cv::resize(baseline_image_, baseline_image_, cv::Size(), 0.45, 0.45); //In, Out 
   /*
    string windowName = "Baseline Image"; //Name of the window
    namedWindow(windowName); // Create a window 
    imshow(windowName,baseline_image_); // Show our image inside the created window.
    waitKey(0); // Wait for any keystroke in the window
    destroyWindow(windowName); //destroy the created window
    */
    
}

void feature_detector_fast(cv::Mat image,  vector<cv::KeyPoint>& kpts, cv::Mat descriptors)
{
    int minHessian = 400;
    //cv::Ptr<cv::SURF> detector = cv::SURF::create( minHessian );
    //cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
   // detector->detectAndCompute( image, cv::noArray(), kpts, descriptors );
   
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(50, true);
    detector->detect(image, kpts);
}

void feature_extraction(cv::Mat image,  vector<cv::KeyPoint>& kpts, cv::Mat descriptors)
{
    //cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::Extractor::create();
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::ORB::create();
  
    descriptorExtractor->compute(image, kpts, descriptors);
    
}

void find_matches(cv::Mat descriptors1, cv::Mat descriptors2, std::vector< vector<cv::DMatch >> matches)
{
     //cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
    //matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
     //matcher->knnMatch(descriptors1, descriptors2, matches, 2);
     //cv::Ptr<cv::FlannBasedMatcher> matcher(new cv::flann::LshIndexParams(20, 10, 2)); 
     //cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
   
     cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      //matcher(new cv::flann::LshIndexParams(20, 10, 2))
      matcher->knnMatch(descriptors1, descriptors2, matches,2, false);  
      cout << matches.size() << endl;
   
}


void  Feature_detection_Impl::feature_detector(string alg, cv::Mat reference_image, cv::Mat current_frame)
{
    cv::Mat descriptors_1_;
    cv::Mat descriptors_2_;
    std::vector< vector<cv::DMatch > > matches;
    //vector<cv::DMatch> matches;
    //Define the two images to compare 
    reference_image_ = reference_image;
    current_frame_ = current_frame;

 if (alg == "FAST")
    {
        
       //Feature detection on the baseline  
       feature_detector_fast(reference_image_, ktps_1,descriptors_1_);
       feature_extraction(reference_image_, ktps_1, descriptors_1_);
       //Feature Visualization on displayed image 
       
       //Feature detection on the new acquired frame   
       feature_detector_fast(reference_image_, ktps_2,descriptors_2_);
       feature_extraction(reference_image_, ktps_2, descriptors_2_);
       find_matches(descriptors_1_, descriptors_2_, matches);
       
     
        cout << matches.size()<< endl;
       std::vector<cv::DMatch> match1;
       std::vector<cv::DMatch> match2;

        for(int i=0; i<matches.size(); i++)
        {
            match1.push_back(matches[i][0]);
            match2.push_back(matches[i][1]);
            cout <<"SONO QUI" << endl;
        }
        
        cv::Mat img_matches1, img_matches2;
        cv::drawMatches(reference_image_, ktps_1, reference_image_, ktps_2, match1, img_matches1);
        cv::drawMatches(reference_image_, ktps_1, reference_image_, ktps_2, match2, img_matches2);
        
        //-- Show detected matches
        cv::imshow("Matches", img_matches1 );
         cv::waitKey(5);
        cv::imshow("Matches", img_matches2 );
         cv::waitKey(5);

 /*
       cv::Mat image_baseline1 = current_frame_;
        cv::cvtColor( image_baseline1,   image_baseline1, cv::COLOR_GRAY2RGB);
       for (int i = 0; i < ktps_2.size(); i++)
       {
           //float x = ktps_1.pt[0];
          // cout << ktps_1[i].pt << endl;
           //publish [px, py]
         cv::Point pt1 =  ktps_2[i].pt;
         int myradius = 3;
         cv::circle(image_baseline1,cv::Point(pt1.x,pt1.y),myradius,CV_RGB(100,0,0),-1,8,0);
       }
        windowName = "current frame with circles"; //Name of the window
        cv::namedWindow(windowName); // Create a window 
        cv::imshow("current frame with circles", image_baseline1);
        cv::waitKey(5);
        */
        match1.clear();
        match2.clear();
    }

    else
    {
       cout<< " No Algorithm selected. Exit " << endl;
    }
    ktps_1.clear();
    ktps_2.clear();
    matches.clear();
   
}

void Feature_detection_Impl::pass_to_class_baseline_image_path(string path)
{
     
    path_ = path;
    
}

cv::Mat Feature_detection_Impl::obtain_baseline_image()
{ 
   return baseline_image_;
}

/* ############################################à EKF ##################################àà
*/





#endif
