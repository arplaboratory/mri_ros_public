#ifndef _MESH_CONSTRUCTOR_SOURCE_
#define _MESH_CONSTRUCTOR_SOURCE_

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
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_msgs/Vertices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_cloud.h>

#include <numeric>
#include "Mesh_constructor.h"

using namespace std;
using namespace Eigen;

std::ofstream outFile1("/home/arpl/luca_ws/points.txt");

class Mesh_constructor_Impl
{
    private:
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   
    public:
    
    shape_msgs::Mesh cloud_W;
    vector<shape_msgs::Mesh> cloud_W_vector;
    //Class constructor, prende i termini dati in input alla class PID
    Mesh_constructor_Impl();
    void create_polygons(vector<vector< geometry_msgs::Point>> pc_W);
    void create_mesh_linkage_polygons(vector< geometry_msgs::Point> pc_link_W);
    void filtering_realsense_point_cloud(vector<double> x, vector<double> y, vector<double> z);

    void clear_vectors();
    //Value pass to Class
    //void pass_to_class_baseline_image_path(string path);   
    
    //from class 
    vector<shape_msgs::Mesh>  obtain_world_mesh_as_sensor_msgs();
    //Destructor
     ~Mesh_constructor_Impl()
    {

    }
    
};

//Class PanelImpl call from header
Mesh_constructor::Mesh_constructor()
{
    Mesh_constructor_Implementation = new Mesh_constructor_Impl();
}

//###################  FUNCTION WITH COMPUTTATION 
//Function dcall
void Mesh_constructor::create_polygons(vector<vector< geometry_msgs::Point>> pc_W)
{
    return Mesh_constructor_Implementation -> create_polygons(pc_W);
}

void Mesh_constructor::create_mesh_linkage_polygons(vector< geometry_msgs::Point> pc_link_W)
{
    return Mesh_constructor_Implementation -> create_mesh_linkage_polygons(pc_link_W);
}

void Mesh_constructor::clear_vectors()
{
    return Mesh_constructor_Implementation -> clear_vectors();
}

void  Mesh_constructor::filtering_realsense_point_cloud(vector<double> x, vector<double> y, vector<double> z)
{
     return Mesh_constructor_Implementation -> filtering_realsense_point_cloud(x,y,z);
}

vector<shape_msgs::Mesh>  Mesh_constructor::obtain_world_mesh_as_sensor_msgs()
{
    return Mesh_constructor_Implementation -> obtain_world_mesh_as_sensor_msgs();
}




//Il distruttore dealloca anche la classe Panel
Mesh_constructor::~Mesh_constructor() 
{
    delete Mesh_constructor_Implementation;
}

/* Implementation */

Mesh_constructor_Impl::Mesh_constructor_Impl():
//Definisco TUTTE le variabiliche verranno utilizzate nelle funzioni e definite in PanelImpl
//Le variabili passate con costruttore devono essere rinominate e passate al file cpp definendole in private
cloud_(),
cloud_W() //Same but in sensor msgs
{
}

void Mesh_constructor_Impl::create_polygons(vector<vector< geometry_msgs::Point>> pc_W)
{  

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
  //Check vector<vector>> size for fill the cloud_.width 
 int counter = 0;
  for (size_t i = 0; i < pc_W.size(); ++i) //Itero sui vettori interni
  {
      for (size_t j= 0; j < pc_W[i].size(); ++j) //itero sui punti
      {
        counter = counter + 1;
         
      }
  }
 //Fill the cloud_ data 
  cloud->height = 1;
  cloud->width = counter;
  cloud->points.resize (cloud->width * cloud->height);
  cloud->is_dense = false;
  counter = 0;
  
  for (size_t i = 0; i < pc_W.size(); ++i) //Itero sui vettori interni
  {
      for (size_t j= 0; j < pc_W[i].size(); ++j) //itero sui punti
      {
        (*cloud)[counter].x = pc_W[i][j].x;
        (*cloud)[counter].y = pc_W[i][j].y;
        (*cloud)[counter].z = pc_W[i][j].z;
      
        counter = counter + 1;
      }
  }
    
    //Convert to pcl PointCloud 2
    if (cloud->size() > 3)
    {
       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

   // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
  
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
  
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (5);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (500);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

 /* pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);   
viewer.addPolygonMesh(triangles,"meshes",0);
viewer.addCoordinateSystem (1.0);
viewer.initCameraParameters ();
while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
*/
  pcl::io::saveVTKFile ("mesh.vtk", triangles);
 // I punti non vengono visualizzati ma la mesh e create correttaemnte.
 //Esportarla in Rviz per visaulizzarla.
 //Pensare a cosa ha detto giuspeep sul tracking delle feature per verificare la rotazione 



     //convert to ROS format
    pcl_msgs::PolygonMesh pcl_msg_mesh;
    pcl_conversions::fromPCL(triangles, pcl_msg_mesh);
    sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

    size_t size = pcd_modifier.size();

    cloud_W.vertices.resize(size); //shape msg is a ros format to contain th emesh 
    //std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";
    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

     //obtain the position xyz of the points relative to the triplets described by vertices 
    for(size_t i = 0; i < size ; i++, ++pt_iter){
        cloud_W.vertices[i].x = pt_iter[0];
        cloud_W.vertices[i].y = pt_iter[1];
        cloud_W.vertices[i].z = pt_iter[2];
        
    }

    cloud_W.triangles.resize(triangles.polygons.size());

    for (size_t i = 0; i < triangles.polygons.size(); ++i)
    {
        if(triangles.polygons[i].vertices.size() < 3)
        {
            ROS_WARN("Not enough points in polygon. Ignoring it.");
            continue;
        }
    

    for (int j = 0; j < 3; ++j)
    {
        cloud_W.triangles[i].vertex_indices[j] = triangles.polygons[i].vertices[j]; //associate a ros mesh triqangles to each polygons 
       
    }
    }
   
    }

   
   
}

/* This function creates a linkage mesh polygons between two consecutive frame taking only the vertices that actually are very close and 
are under the mean of the vector collecting all the minimum diastance value for each row of the previously built distance matrix
*/
void Mesh_constructor_Impl::create_mesh_linkage_polygons(vector< geometry_msgs::Point> pc_link_W)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr link_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    link_cloud->height = 1;
    link_cloud->width = pc_link_W.size();
    link_cloud->points.resize (link_cloud->width * link_cloud->height);
    link_cloud->is_dense = false;

    for (size_t j= 0; j < pc_link_W.size(); ++j) //itero sui punti
    {
       
      (*link_cloud)[j].x = pc_link_W[j].x;
      (*link_cloud)[j].y = pc_link_W[j].y;
      (*link_cloud)[j].z = pc_link_W[j].z;
      outFile1 <<  pc_link_W[j].x << ", " <<  pc_link_W[j].y << ", " <<  pc_link_W[j].z <<"\n";
    }
     if (link_cloud->size() > 3)
    {

     
    



     //Create plygons 
       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
       pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
       tree->setInputCloud (link_cloud);
       n.setInputCloud (link_cloud);
       n.setSearchMethod (tree);
       n.setKSearch (20);
       n.compute (*normals);
   
      // Concatenate the XYZ and normal fields*
       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
       pcl::concatenateFields (*link_cloud, *normals, *cloud_with_normals);
       //* cloud_with_normals = cloud + normals
     
       // Create search tree*
       pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
       tree2->setInputCloud (cloud_with_normals);
     
       // Initialize objects
       pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
       pcl::PolygonMesh triangles;
   
       // Set the maximum distance between connected points (maximum edge length)
       gp3.setSearchRadius (1);
     
       // Set typical values for the parameters
       gp3.setMu (2.5);
       gp3.setMaximumNearestNeighbors (500);
       gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
       gp3.setMinimumAngle(M_PI/18); // 10 degrees
       gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
       gp3.setNormalConsistency(false);
     
       // Get result
       gp3.setInputCloud (cloud_with_normals);
       gp3.setSearchMethod (tree2);
       gp3.reconstruct (triangles);
     
       // Additional vertex information
       std::vector<int> parts = gp3.getPartIDs();
       std::vector<int> states = gp3.getPointStates();
    
    /*
       pcl::visualization::PCLVisualizer viewer ("3D Viewer");
       viewer.setBackgroundColor (0, 0, 0);   
       viewer.addPolygonMesh(triangles,"meshes",0);
       viewer.addCoordinateSystem (1.0);
       viewer.initCameraParameters ();
       while (!viewer.wasStopped ()){
           viewer.spinOnce (100);
           boost::this_thread::sleep (boost::posix_time::microseconds (100000));
       }
   */
    pcl::io::saveVTKFile ("mesh_link.vtk", triangles);
    cout << "Mesh Generated" << endl;
     //convert to ROS format
   
   
    pcl_msgs::PolygonMesh pcl_msg_mesh;
    pcl_conversions::fromPCL(triangles, pcl_msg_mesh);
    sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

    size_t size = pcd_modifier.size();

    cloud_W.vertices.resize(size); //shape msg is a ros format to contain th emesh 
    //std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";
    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

     //obtain the position xyz of the points relative to the triplets described by vertices 
    for(size_t i = 0; i < size ; i++, ++pt_iter){
        cloud_W.vertices[i].x = pt_iter[0];
        cloud_W.vertices[i].y = pt_iter[1];
        cloud_W.vertices[i].z = pt_iter[2];
        
    }

    cloud_W.triangles.resize(triangles.polygons.size());

    for (size_t i = 0; i < triangles.polygons.size(); ++i)
    {
        if(triangles.polygons[i].vertices.size() < 3)
        {
            ROS_WARN("Not enough points in polygon. Ignoring it.");
            continue;
        }
    

    for (int j = 0; j < 3; ++j)
    {
        cloud_W.triangles[i].vertex_indices[j] = triangles.polygons[i].vertices[j]; //associate a ros mesh triqangles to each polygons 
       
    }
    }
    }
}


void Mesh_constructor_Impl::filtering_realsense_point_cloud(vector<double> x, vector<double> y, vector<double> z)
{
   //Filter Point cloud with pcl librqary 

   //First filtering the point cloud along the z axis to acccept values that are only inside a certain range between a mean and a variance
   //Compute the average distance of the points along the z axis 
   double sum_of_elems = 0.0;
   sum_of_elems = std::accumulate(z.begin(), z.end(), 0.0);
   double average = sum_of_elems/z.size();

    std::vector<double> diff(z.size());
    std::transform(z.begin(), z.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), average));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / z.size());
    
    //cout<<"Average: " << average << endl;
    //cout << "std: " << stdev << endl;

    //Take only the values around the average plus the std values 
    vector<double> z_filtered;
    //cout << "z size:" << z.size() << endl;
    double th = stdev/3.0;

    for (int ii = 0; ii < z.size(); ii++)
    {
        if (z[ii] > average+ th && z[ii] < average- th)
        {
            
            continue;
        }
        else
        {
           z_filtered.push_back(z[ii]);
        }
       
    } 

      //cout << " z_filtered size:" << z_filtered.size() << endl;

      //Voxelize the point cloud reducing the number of points
    //Create 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud< pcl::PointXYZ>);
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    msg->header.frame_id = "left_camera";
    msg->height = 1;
    msg->points.clear();
    int counter = 0;
    for (int ii = 0; ii < z.size(); ii++)
    {
        geometry_msgs::Point p;
        if (z[ii] > average + th && z[ii] < average- th)
        {
            
            continue;
        }
        else
        {
          p.x = x[ii];
          p.y = y[ii];
          p.z = z[ii];
          msg->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
               
          counter = counter + 1;

        }
    }
     msg->width = counter;
      /*
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(*msg, *cloud);
      pcl::PCLPointCloud2 pcl2_cloud;
      pcl_conversions::toPCL(*cloud,pcl2_cloud);
    
    pcl::PCLPointCloud2::Ptr cloud_filtered_pcl2 (new pcl::PCLPointCloud2 ());
*/
  
  //std::cerr << "PointCloud before filtering: " << pcl2_cloud.width * pcl2_cloud.height << std::endl;
 //converting pcl pointcloud to pcl pointcloud2
  //pcl::fromPCLPointCloud2(*cloud_pcl2, *msg); 
  // Create the filtering object
  //cout << cloud_pcl2->width << endl;
  /*
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(*pcl2_cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_pcl2);

  std::cerr << "PointCloud after filtering: " << cloud_filtered_pcl2->width * cloud_filtered_pcl2->height << std::endl;
*/

}

vector<shape_msgs::Mesh>  Mesh_constructor_Impl::obtain_world_mesh_as_sensor_msgs()
{
    cloud_W_vector.clear();
    cloud_W_vector.push_back(cloud_W);
   
    
    return cloud_W_vector;
}



void Mesh_constructor_Impl::clear_vectors()
{
    cloud_.clear();
  
}


#endif
