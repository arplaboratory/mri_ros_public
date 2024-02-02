#ifndef _MESH_CONSTRUCTOR_H_
#define _MESH_CONSTRUCTOR_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <shape_msgs/Mesh.h>
using namespace std;
using namespace Eigen;


class Mesh_constructor_Impl;

class Mesh_constructor
{
    private:
    Mesh_constructor_Impl *Mesh_constructor_Implementation; 

    public:
    shape_msgs::Mesh cloud_W_msgs;
    
    //Costruttore classe Feature_detection
    Mesh_constructor();

    void create_polygons(vector<vector< geometry_msgs::Point>> pc_W);
    void create_mesh_linkage_polygons(vector< geometry_msgs::Point> pc_link_W);
    void filtering_realsense_point_cloud(vector<double> x, vector<double> y, vector<double> z);
    void clear_vectors();
    
    //Variable passed to class
    //void pass_to_class_baseline_image_path(string path);    
    //From class to outside
    vector<shape_msgs::Mesh> obtain_world_mesh_as_sensor_msgs();
    
    ~Mesh_constructor();
};





#endif