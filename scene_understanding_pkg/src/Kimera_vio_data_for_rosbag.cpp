#include "ros/ros.h"
//#include </home/arpl/luca_ws/devel/include/scene_understanding_pkg_msgs/MeshPos.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <sys/stat.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "Kimera_vio_data_for_rosbag.h"
#include "SU_Unity_comm.h"
#include "Mesh_constructor.h"
#include <shape_msgs/Mesh.h>
#include <typeinfo>

//Voxblox Libraries
#include <voxblox/core/esdf_map.h>
//#include "/home/lucamora/voxblox_ws/src/voxblox/voxblox_ros/include/voxblox_ros/esdf_server.h"
//#include "voxblox_ros/esdf_server.h"

using namespace std::chrono;
using namespace std;
using namespace Eigen;


#define C_PI (double)3.141592653589

// Global variables
auto last_published_mesh = high_resolution_clock::now();
std::ofstream outFile2("/home/arpl/luca_ws/len_triangles_perimeter.txt");

int frame_counter = 0;

/*Da cancellare */
float x_camera = 0.0;
float y_camera = 0.0;
float z_camera = 0.0;
float theta_camera = 0.0;
float alfa_camera = 0.0;
float yaw_camera = 0.0;
float point_x_camera_frame = 0.0;
float point_y_camera_frame = 0.0;
float point_z_camera_frame = 0.0;

void initialize_visualization_message(Kimera_vio_data_for_rosbag *kimera_data)
{
  kimera_data->triangles.points.clear();
  kimera_data->arrows.points.clear();

  kimera_data->triangles.header.frame_id = "/mocap";
  kimera_data->arrows.header.frame_id = "/world";

  kimera_data->triangles.header.stamp = ros::Time::now();
  kimera_data->arrows.header.stamp = ros::Time::now();

  // Markers Namespace
  kimera_data->triangles.ns = "triangle_list";
  kimera_data->arrows.ns = "arrow_list";
  // Assign a type of action to each marker
  kimera_data->triangles.action = visualization_msgs::Marker::ADD;
  kimera_data->arrows.action = visualization_msgs::Marker::ADD;

  // Pose -- Since are points the orientation is not important
  kimera_data->triangles.pose.orientation.w = 1.0;
  kimera_data->arrows.pose.orientation.w = 1.0;

  // Marker Type (Arrow, Sphere, points )
  kimera_data->triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  kimera_data->arrows.type = visualization_msgs::Marker::LINE_LIST;

  kimera_data->triangles.id = 0;
  kimera_data->arrows.id = 0;

  // POINTS markers use x and y scale for width/height respectively
  kimera_data->triangles.scale.x = 1;
  kimera_data->triangles.scale.y = 1;
  kimera_data->triangles.scale.z = 1;

  kimera_data->arrows.scale.x = 1;
  kimera_data->arrows.scale.y = 1;
  kimera_data->arrows.scale.z = 1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

  // Points are green
  kimera_data->triangles.color.g = 1.0f;
  kimera_data->triangles.color.a = 0.3;
  
 

  kimera_data->arrows.color.r = 1.0f;
  kimera_data->arrows.color.a = 1.0;
}

void take_only_offset_rotation_around_x_axis(float theta, float &theta_diff)
{
  // Realsense camera if the ray from camera is parallel to the floor, kimera define a rotation around x axis of +pi/2
  // theta = 0 if camera is facing the ceiling
  // All the angles are expressed in radians
  if (theta >= -C_PI && theta < 0)
  {
    // When looking forward the angle theta is zero
    //  We need to subtract -C_PI
    theta_diff = theta - (-C_PI / 2);
  }
  else if (theta >= 0 && theta < C_PI / 2)
  {
    theta_diff = theta + C_PI / 2;
  }
  else
  {
    float x = theta - C_PI;
    theta_diff = -C_PI + x;
  }
}

void rotate_realsense_pc_in_world_frame(Kimera_vio_data_for_rosbag *kimera_data)
{
//   float tx = 0.0;
//   float ty = 0.0; 
//   float tz = 0.0;
//   float theta = 0.0;
//   float alfa = 0.0;
//   float yaw = 0.0;
//   float theta_diff = 0.0;


//     tx = kimera_data->camera_odometry_position.x; // x_camera
//     ty = kimera_data->camera_odometry_position.y; // y_camera
//     tz = kimera_data->camera_odometry_position.z; // z_camera
  
//     theta = kimera_data->camera_odometry_orientation.x; // theta_camera; rotation of the camera frame around the x axis of the world frame
//     theta_diff = theta;                                 // 0.0;
//     take_only_offset_rotation_around_x_axis(theta, theta_diff);
  
 
//   //theta_diff = 0.0;
//     alfa = kimera_data->camera_odometry_orientation.y; // alfa_camera;
//     alfa = 0.0;
//     theta_diff = 0.0;
//     yaw = kimera_data->camera_odometry_orientation.z;  // yaw_camera;
  
    

  
 
//   cout << "Yaw: " << yaw << "theta: " << theta_diff << "alfa: " << alfa << endl;
//     //yaw = 0.0;
//   // Before rotating the points from B to w rotate the world frame of pi/2 degree

//   Matrix4f Rx_w;
//   Rx_w << 1, 0, 0, 0,
//       0, cos(M_PI / 2), sin(M_PI / 2), 0,
//       0, -sin(M_PI / 2), cos(M_PI / 2), 0,
//       0, 0, 0, 1;

//   Matrix4f T;
//   T << 1, 0, 0, tx,
//       0, 1, 0, ty,
//       0, 0, 1, tz,
//       0, 0, 0, 1;

//   Matrix4f Rx;
//   Rx << 1, 0, 0, 0,
//       0, cos(theta_diff), sin(theta_diff), 0,
//       0, -sin(theta_diff), cos(theta_diff), 0,
//       0, 0, 0, 1;

//   Matrix4f Ry;
//   Ry << cos(alfa), 0, -sin(alfa), 0,
//       0, 1, 0, 0,
//       sin(alfa), 0, cos(alfa), 0,
//       0, 0, 0, 1;

//   Matrix4f Rz;
//   Rz << cos(yaw), -sin(yaw), 0, 0,
//       sin(yaw), cos(yaw), 0, 0,
//       0, 0, 1, 0,
//       0, 0, 0, 1;
 
//   Matrix3f Rz_test;
//   Rz_test << cos(yaw), -sin(yaw), tx,
//       sin(yaw), cos(yaw), ty,
//       0, 0, 1;
     
//   Vector4f P_W;
//   P_W << 0.0, 0.0, 0.0, 0.0;

 
//   Vector4f P_C;
//   P_C << 0.0, 0.0, 0.0, 0.0;
  
 

//   /*
//   P_C <<  -1*point_y_camera_frame,  point_x_camera_frame,  point_z_camera_frame, 1;
//   P_W = T * Rx * Ry * Rz * P_C;

//   cout << "x: " << P_W(0) << " y: " <<  P_W(1) << " z: " << P_W(2) << endl;
//   cout << "x c: " << tx << " y c: " <<  ty << " z c: " <<tz << endl;
// */

  geometry_msgs::Point p_w;
  for (int i = 0; i < kimera_data->point_y.size(); i++)
  {
   
    p_w.x =  kimera_data->point_x[i];
    p_w.y =  kimera_data->point_y[i];
    p_w.z =  kimera_data->point_z[i];

    kimera_data->point_cloud_realsense_W.push_back(p_w);
  }

  if (kimera_data->start_mapping == true)
  {
    kimera_data->publish_rs_pointcloud_world( kimera_data->point_cloud_realsense_W);
    cout << "START MAPPING" << endl;
  }
  else
  {
     cout << "Mapping Stopped" << endl;
  }
  
  kimera_data->point_cloud_realsense_W.clear();

}



void check_last_pose_before_transformation(Kimera_vio_data_for_rosbag *kimera_data)
{
  
  //Check the last pose received before computing the rotation
  int counter = 0;
  cout << "SIZE voxl pose array: " <<  kimera_data->voxl_pose_array.size() << endl;
  vector<double> difference_array;
  for (int i = 0; i < kimera_data->voxl_pose_array.size(); i++) 
  {
    //check for the header stamp
    
   
   
    double pose_stamp = double( kimera_data->voxl_pose_array[i].header.stamp.sec) + double( kimera_data->voxl_pose_array[i].header.stamp.nsec)*1e-9;
    double difference = pose_stamp -  kimera_data->last_pointcloud_stamp;
    cout << "difference: " << abs(difference) << endl;
    difference_array.push_back(abs(difference));
  }

  //Find the closest elemet to zeor
  auto it = std::min_element(std::begin(difference_array), std::end(difference_array));

  int index_smallest = std::distance(std::begin(difference_array), it);

  std::cout << "index of smallest element: " << std::distance(std::begin(difference_array), it) << endl;
  
  kimera_data->voxl_position_GF_for_rotation = kimera_data->voxl_pose_array[index_smallest].pose.pose.position;
  kimera_data->euler_angles_for_pc_rotation.x = kimera_data->voxl_euler_array[index_smallest].x;
  kimera_data->euler_angles_for_pc_rotation.y = kimera_data->voxl_euler_array[index_smallest].y;
  kimera_data->euler_angles_for_pc_rotation.z = kimera_data->voxl_euler_array[index_smallest].z;

  cout << "roll: " << kimera_data->euler_angles_for_pc_rotation.x << "pitch: " << kimera_data->euler_angles_for_pc_rotation.y << "yaw: " <<  kimera_data->euler_angles_for_pc_rotation.z << endl;
  
 

  kimera_data->voxl_pose_array.erase( kimera_data->voxl_pose_array.begin(), kimera_data->voxl_pose_array.begin()+index_smallest);
   kimera_data->voxl_euler_array.erase( kimera_data->voxl_euler_array.begin(),  kimera_data->voxl_euler_array.begin()+ index_smallest);
  cout << "Size after : " <<   kimera_data->voxl_pose_array.size() << endl;


}



//####################################################################

void rotate_voxl3_pc_in_world_frame(Kimera_vio_data_for_rosbag *kimera_data)
{



  for (int i = 0; i < kimera_data->point_y.size(); i++)
  {
  
    geometry_msgs::Point p_w;
    
    p_w.x =  kimera_data->point_x[i];
    p_w.y =  kimera_data->point_y[i];
    p_w.z =  kimera_data->point_z[i];

  

    kimera_data->point_cloud_realsense_W.push_back(p_w);
  }


   if (kimera_data->with_holo == false)
   {
      kimera_data->start_mapping = true;    
   }
  
   if (kimera_data->start_mapping == true)
   {
    kimera_data->publish_rs_pointcloud_world( kimera_data->point_cloud_realsense_W);
     cout << "Mapping in Progress" << endl;
   }
   else{
     cout << "Mapping Suspended" << endl;
   }
 kimera_data->point_cloud_realsense_W.clear();
}




void rotation_camera_to_world_frame(Kimera_vio_data_for_rosbag *kimera_data)
{
  // Rotation and translation in 3D
  //  In this function a point is translated from the camera frame to the 3D world Frame

  float tx = kimera_data->camera_odometry_position.x;
  float ty = kimera_data->camera_odometry_position.y;
  float tz = kimera_data->camera_odometry_position.z;

  // The camera frame displays points correctly in RVIZ World frame,
  // but it present a rotation o -pi/2 around the x axis.
  // In the rotation matrix we have to considere only the offset rotation around the x axis, in order to avoid errors (Guardare Ipad Oer maggiori informazioni)
  float theta = kimera_data->camera_odometry_orientation.x; // rotation of the camera frame around the x axis of the world frame
  float theta_diff = 0.0;
  take_only_offset_rotation_around_x_axis(theta, theta_diff);
  // cout << "theta: " << theta << "theta_diff:  "<< theta_diff << endl;

  float alfa = kimera_data->camera_odometry_orientation.y;
  float yaw = kimera_data->camera_odometry_orientation.z;

  // float x = kimera_data->shape_msg_mesh.vertices[ kimera_data->shape_msg_mesh.triangles[0].vertex_indices[0]]; //x
  // float y = kimera_data->shape_msg_mesh.vertices[ kimera_data->shape_msg_mesh.triangles[0].vertex_indices[0]]; //y
  // float z = kimera_data->shape_msg_mesh.vertices[ kimera_data->shape_msg_mesh.triangles[0].vertex_indices[0]]; //z
  // Define the rotations and translation matrices with Eigen
  Matrix4f T;
  T << 1, 0, 0, tx,
      0, 1, 0, ty,
      0, 0, 1, tz,
      0, 0, 0, 1;

  Matrix4f Rx;
  Rx << 1, 0, 0, 0,
      0, cos(theta_diff), sin(theta_diff), 0,
      0, -sin(theta_diff), cos(theta_diff), 0,
      0, 0, 0, 1;

  Matrix4f Ry;
  Ry << cos(alfa), 0, -sin(alfa), 0,
      0, 1, 0, 0,
      sin(alfa), 0, cos(alfa), 0,
      0, 0, 0, 1;

  Matrix4f Rz;
  Rz << cos(yaw), -sin(alfa), 0, 0,
      sin(yaw), cos(yaw), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Vector4f P_W;
  P_W << 0.0, 0.0, 0.0, 0.0;

  Vector4f P_C;
  P_C << 0.0, 0.0, 0.0, 0.0; // Vertices Posityion in camera frame
  /*
  cout << "Theta_diff: " << theta_diff << endl;
  cout << "alfa: " << alfa << endl;
  cout << "yaw: " << theta_diff << endl;
  cout << "Theta_diff: " << theta_diff << endl;
  */

  // Initialize Rviz Messages to publish vertices as Marker
  // initialize_visualization_message(kimera_data);
  // Find the number of triangles in shape Msgs

  size_t size_triangles = kimera_data->shape_msg_mesh.triangles.size();

  size_t i = 0;
  // Iterate on the number of triangles
  for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index)
  {
    for (int i = 0; i < 3; i++)
    {
      // take the vertices x,y,z and store them in p
      geometry_msgs::Point p;
      geometry_msgs::Point p_w;
      p = kimera_data->shape_msg_mesh.vertices[kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[i]];

      P_C << p.x, p.y, p.z, 1;
      // cout << "P_C: " << P_C << endl;
      P_W = T * Rx * Ry * Rz * P_C;

      p_w.x = P_W(0);
      p_w.y = P_W(1);
      p_w.z = P_W(2);
      // Save data rotated in world frame
      kimera_data->point_cloud_W.push_back(p);
    }

    // Convert

    /*

    p = kimera_data->shape_msg_mesh.vertices[kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[0]];
    //Populate eigen vector
    P_C << p.x, p.y, p.z, 1;
    //cout << "P_C: " << P_C << endl;
    P_W = T*Rx*Ry*Rz*P_C;
    //cout << "P_W: " << P_W << endl;
    //insert the vertices in WF in points of the visualization msgs
    kimera_data-> triangles.points[i].x = P_W(0,0);
    kimera_data-> triangles.points[i].y = P_W(1,0);
    kimera_data-> triangles.points[i].z = P_W(2,0);

    //Repeat the operation for the other two vertices reamaing in each triangle
    p = kimera_data->shape_msg_mesh.vertices[kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[1]];
    //Populate eigen vector
    P_C << p.x, p.y, p.z, 1;
    P_W = T*Rx*Ry*Rz*P_C;
    //insert the vertices in WF in points of the visualization msgs
    kimera_data-> triangles.points[i+1].x = P_W(0,0);
    kimera_data-> triangles.points[i+1].y = P_W(1,0);
    kimera_data-> triangles.points[i+1].z = P_W(2,0);

    p = kimera_data->shape_msg_mesh.vertices[kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[2]];
    //Populate eigen vector
    P_C << p.x, p.y, p.z, 1;
    P_W = T*Rx*Ry*Rz*P_C;
    //insert the vertices in WF in points of the visualization msgs

    kimera_data-> triangles.points[i+2].x = P_W(0,0);
    kimera_data-> triangles.points[i+2].y = P_W(1,0);
    kimera_data-> triangles.points[i+2].z = P_W(2,0);

    i = i + 3;
  */
  }
}

void fill_grid(Kimera_vio_data_for_rosbag *kimera_data, Eigen::MatrixXd mat)
{
  size_t size_triangles = kimera_data->shape_msg_mesh.triangles.size();
  // Evaluate mean on x and y of the point position
  float x_sum = 0.0;
  float y_sum = 0.0;
  float x_mean = 0.0;
  float y_mean = 0.0;
  // Create vector with X coo in W and Y coo in World
  for (size_t i = 0; i < kimera_data->triangles.points.size(); ++i)
  {
    x_sum = x_sum + kimera_data->triangles.points[i].x;
    y_sum = y_sum + kimera_data->triangles.points[i].y;
  }

  // Evaluate the mean value
  x_mean = x_sum / (kimera_data->triangles.points.size());
  y_mean = y_sum / (kimera_data->triangles.points.size());

  // cout<< "x_mean: " << x_mean << "y_mean: " << y_mean << endl;

  // Verifico quanto e discretizzata la matrice
  float discretization = 100 / (mat.rows());
  // Per esempio se discretizattion 5, x_mean = 16 --> cell_x = 3
  int cell_n_x = x_mean / discretization;
  int cell_n_y = y_mean / discretization;
  // cout<< "cell_n_x: " << cell_n_x << "cell_n_y: " << cell_n_y << endl;
  // Fill the matrix
  mat(int(cell_n_x), int(cell_n_y)) = 1;
}

double avg(std::vector<float> const &v, double percentage)
{

  float sum = 0;
  for (int i = 0; i < v.size(); i++)
  {
    sum = sum + v[i];
  }

  double mean = sum / v.size();
  double mean_p = (mean * percentage) / 100;
  double mean_f = mean + mean_p;

  return mean_f;
}

bool filter_triangles(geometry_msgs::Point p, geometry_msgs::Point p1, geometry_msgs::Point p2, float x, float y, float z)
{
  float distance_length1 = 0.0;
  float distance_length2 = 0.0;
  float distance_length3 = 0.0;
  bool skip_triangle = false;
  distance_length1 = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));
  distance_length2 = sqrt(pow(p.x - p2.x, 2) + pow(p.y - p2.y, 2) + pow(p.z - p2.z, 2));
  distance_length3 = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));

  float len_perimeter = distance_length1 + distance_length2 + distance_length3;
  outFile2 << len_perimeter << "\n";

  if (isinf(len_perimeter))
  {
    // cout<< "ISINF"<<endl;
    return skip_triangle = true;
  }
  if (isnan(len_perimeter))
  {
    // cout<< "ISNAN"<<endl;
    return skip_triangle = true;
  }
  if (len_perimeter < 0.001)
  {
    // cout << "LESS THAN 0.001 " << len_perimeter << endl;
    return skip_triangle = true;
  }
  if (len_perimeter > 3.5)
  {
    // cout << "MORE THAN 5 " << len_perimeter << endl;
    return skip_triangle = true;
  }

  // Check also the position of the triangles.
  if (p.z > 2.5 || p1.z > 2.5 || p2.z > 2.5)
  {
    return skip_triangle = true;
  }

  // Verify the distanxce of the triangle from the camera

  // Evaluate distance between the camera and each point of the triangle
  float dist1 = sqrt(pow(p.x - x, 2) + pow(p.y - y, 2) + pow(p.z - z, 2));
  float dist2 = sqrt(pow(p1.x - x, 2) + pow(p1.y - y, 2) + pow(p1.z - z, 2));
  float dist3 = sqrt(pow(p2.x - x, 2) + pow(p2.y - y, 2) + pow(p2.z - z, 2));
  /*
    if (dist1 > 6 || dist2 > 6 || dist3 > 6)
    {
      return skip_triangle = true;
    }
  */
  return skip_triangle;
}

void fill_RVIZ_triangulation_with_new_kimera_frames(Kimera_vio_data_for_rosbag *kimera_data)
{
  // Vector pc_W_final: contains the kimera vertices in woirld frame of the mesh considered during linkage (the series of old one and the new one)
  // kimera_data-> triangles.points: store the mesh vertices to be displayed in RVIZ
  //  kimera_data->mesh_vector_CF: store the vertices in CF and the indices of the intereseted kimera frames. (used to obtain the indices to recreate the correct trinagulation with the WF vertices )

  // Iterare sui vertici contenuti nel  vettore pc_W_final espressi nel world frame e riferityi alle mesh da cosiderare
  // cout<< "PC VECTOR SIZE: " << kimera_data->pc_W_final.size() << endl;

  for (int ii = 0; ii < kimera_data->pc_W_final.size(); ii++) // Nel mio caso dovrebbe essere 2 la dimensione di pc_W_final
  {
    // Selezioni gli indici relativi alla prima mesh in posizione [0]
    //  Itero sui triangoli e su ciascun vertice del triangolo
    size_t size_triangles = kimera_data->mesh_vector_CF[ii].triangles.size();
    for (int tri_index = 0; tri_index < size_triangles; tri_index++)
    {
      // Per ciascun triangolo considero indici dei vertici in posizione 0,1, 2
      geometry_msgs::Point p;
      int index1 = kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[2];

      geometry_msgs::Point p1;
      int index2 = kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[1];

      geometry_msgs::Point p2;
      int index3 = kimera_data->shape_msg_mesh.triangles[tri_index].vertex_indices[0];

      if (index1 == index2)
      {
        cout << "Degenerate triangles: remove" << endl;
        continue;
      }
      if (index1 == index3)
      {
        cout << "Degenerate triangles: remove" << endl;
        continue;
      }
      if (index2 == index3)
      {
        cout << "Degenerate triangles: remove" << endl;
        continue;
      }

      // Considero i vertici nel worl frame relativi a quell'indice
      p = kimera_data->pc_W_final[ii][index1];
      // Considero i vertici nel worl frame relativi a quell'indice
      p1 = kimera_data->pc_W_final[ii][index2];
      // Considero i vertici nel worl frame relativi a quell'indice
      p2 = kimera_data->pc_W_final[ii][index3];

      // Check the perimeter of each triangle (chek if it is > th, < 0.01 or isInf or isNAN)
      float x = kimera_data->camera_odometry_position.x;
      float y = kimera_data->camera_odometry_position.y;
      float z = kimera_data->camera_odometry_position.z;
      if (filter_triangles(p, p1, p2, x, y, z) == true)
      {
        cout << "Triangle Skipped" << endl;
        continue;
      }
      else
      {
        // inserisco il vertice nella lista dei triangoli di RVIz
        kimera_data->triangles.points.push_back(p);
        // inserisco il vertice nella lista dei triangoli di RVIz
        kimera_data->triangles.points.push_back(p1);
        // inserisco il vertice nella lista dei triangoli di RVIz
        kimera_data->triangles.points.push_back(p2);
      }

      // cout << "len edge between v1 and v2: " << abs(p1.x - p.x) << endl;
    }
  }
}


  



int counter_inside_if = 0;
int voxblox_size_previous  = 0;
bool stop_publish_debug = false;
//int counter = 0;
void populate_RVIZ_triangle_list_with_voxblox_pc(Kimera_vio_data_for_rosbag *kimera_data, std::chrono::microseconds elapesed_time, double mesh_publish_period)
{
  
 

  rotate_voxl3_pc_in_world_frame(kimera_data);
  

  
    // Using VoxBlox Meshes
    int counter = 0;
    int counter_clock_wise =0;
    
  

    for (int tri_index = 0; tri_index < kimera_data->voxblox_mesh_vertices.size() / 3; ++tri_index)  //kimera_data->voxblox_mesh_vertices.size() / 3
    {

      geometry_msgs::Point p;

      p = kimera_data->voxblox_mesh_vertices[counter]; //ccounter //[counter_clock_wise + 2
      //cout << "Point Coordinate: " << p << endl; 
      kimera_data->triangles.points.push_back(p);

      p = kimera_data->voxblox_mesh_vertices[counter + 1]; //counter_clock_wise + 1
      //cout << "Point Coordinate: " << p << endl; 
      // inserisco il vertice nella lista dei triangoli di RVIz
      kimera_data->triangles.points.push_back(p);

      p = kimera_data->voxblox_mesh_vertices[counter + 2]; //counter_clock_wise
     // cout << "Point Coordinate: " << p << endl; 
     
      // inserisco il vertice nella lista dei triangoli di RVIz
      kimera_data->triangles.points.push_back(p);
      counter = counter + 3;
      counter_clock_wise = counter;

     
    }

    // Fill the arrows list with normals
    int counter_normal = 0;
    for (int point_index = 0; point_index < kimera_data->triangles_normals.size() / 2; point_index++)
    {
      geometry_msgs::Point p;
      p = kimera_data->triangles_normals[counter];
      kimera_data->arrows.points.push_back(p);

      p = kimera_data->triangles_normals[counter + 1];
      kimera_data->arrows.points.push_back(p);

      counter_normal = counter_normal + 2;
    }

    counter_inside_if = counter_inside_if + 1;
    counter_inside_if = 0;

  
  voxblox_size_previous =  kimera_data->voxblox_mesh_vertices.size();
  // Publish Vertices at a given frequency
  if (elapesed_time.count() > mesh_publish_period && stop_publish_debug == false)
  {

    if (kimera_data->triangles.points.size() > 0)
    {
     
      kimera_data->publish_mesh_to_unity(kimera_data->triangles);
      // Translating the Mesh Verteces position to the Mocap Sr position from the Mocap SR
      if (kimera_data->voxl3_odometry != 0)
      {
        for (int j = 0; j < kimera_data->triangles.points.size(); j++)
        {
          kimera_data->triangles.points[j].x = kimera_data->triangles.points[j].x + kimera_data->mocap_sr_unity_world_position.x;
          kimera_data->triangles.points[j].y = kimera_data->triangles.points[j].y + kimera_data->mocap_sr_unity_world_position.y;
          kimera_data->triangles.points[j].z = kimera_data->triangles.points[j].z + kimera_data->mocap_sr_unity_world_position.z;
        }
      }
      kimera_data->publish_mesh_vertices_to_rviz(kimera_data->triangles);







      //Before Publish check the new vertices --> i want to publish only the new vertices 
      /*
      if (kimera_data->triangles_old.points.size() > 0)
      {
         kimera_data->triangles_old.color.r = 1.0f;
        

       
        if ( kimera_data->triangles_old.points.size() < kimera_data->triangles.points.size())
        {
          int size_difference = kimera_data->triangles.points.size() - kimera_data->triangles_old.points.size();
          int start_position = kimera_data->triangles.points.size() - size_difference;
          cout << "start_position: " << start_position << " size_difference: " << size_difference << endl;
          kimera_data->triangles_old.points.clear();
          geometry_msgs::Point p;
         
          for (int i = start_position; i < kimera_data->triangles.points.size(); i++)
          {
            /*
            float x_square = pow(kimera_data->triangles.points[i].x - kimera_data->triangles_old.points[i].x, 2); 
            float y_square = pow(kimera_data->triangles.points[i].y - kimera_data->triangles_old.points[i].y, 2); 
            float z_square = pow(kimera_data->triangles.points[i].z - kimera_data->triangles_old.points[i].z, 2); 

            float distance = sqrt(x_square + y_square + z_square);
            */
           /*
             p = kimera_data->triangles.points[i];
             kimera_data->triangles_old.points.push_back(p);
             
            //cout << " DISTANCE con OLD < new: " << distance << endl;
          }

         
           kimera_data->publish_mesh_to_unity(kimera_data->triangles_old);
          kimera_data->publish_old_mesh_vertices_to_rviz(kimera_data->triangles_old);
          //kimera_data->publish_mesh_to_unity(kimera_data->triangles_difference);
          
        }
      
      }
       
        else
        {
          for (int i = 0; i < kimera_data->triangles.points.size(); i++)
          {
            float x_square = pow(kimera_data->triangles.points[i].x - kimera_data->triangles_old.points[i].x, 2); 
            float y_square = pow(kimera_data->triangles.points[i].y - kimera_data->triangles_old.points[i].y, 2); 
            float z_square = pow(kimera_data->triangles.points[i].z - kimera_data->triangles_old.points[i].z, 2); 

            float distance = sqrt(x_square + y_square + z_square);
            //cout << " DISTANCE con new < OLD: " << distance << endl;
          }

        }
          
      }
      */
       
    
     
      //kimera_data->publish_mesh_to_unity_as_pointcloud(kimera_data->triangles);
      // kimera_data->publish_arrows_vertices_to_rviz(kimera_data->arrows);
      
      //Genrate a new file when all the triangles list is saved 
     
      stringstream ss;
      ss << kimera_data->counter_triangle_published;
      string txt_output_name = "triangle_vertices_" + ss.str() + ".txt"; //Salvare le posizioni dei voxel date da voxblox
      std::ofstream outFile1(kimera_data->stringpath + txt_output_name);
      for (int i = 0; i < kimera_data->triangles.points.size(); i++)
      {
        outFile1 <<kimera_data->triangles.points[i].x << ", "<<kimera_data->triangles.points[i].y << ", "<< kimera_data->triangles.points[i].z << "\n";
      }
     
      cout << "-----------------------------Data Published-------------------------------------------" << endl;

      //Fill the old triangle list with the published triangles 
      //Cleaning the previous  kimera_data->triangles_old before the new assignment 
      kimera_data->triangles_old = kimera_data->triangles;

      kimera_data->data_published_to_rviz = true;
      kimera_data->counter_triangle_published = kimera_data->counter_triangle_published + 1;
    }
    

  
    kimera_data->triangles.points.clear();     //uncomment to delete the mesh 


    kimera_data->arrows.points.clear();
    //Mesh_Const->clear_vectors();
    // kimera_data-> point_cloud_W.clear();
    // kimera_data->pc_W_final.clear();
    // kimera_data->mesh_vector_CF.clear();
    kimera_data->mesh_vector_GF.clear();
    
   
    last_published_mesh = high_resolution_clock::now();
    
      stop_publish_debug = false;
   
  }

  //Clear the voxblox Vector otherwise the same triamngles are added to the Visualization 
  //Marker Message 
   kimera_data->voxblox_mesh_vertices.clear();
    kimera_data->triangles_normals.clear();

}




int counter_inside = 0;
int baseline_counter = 0;
void populate_RVIZ_triangle_list_with_kimera_pc(Kimera_vio_data_for_rosbag *kimera_data,
                                                Mesh_constructor *Mesh_Const, std::chrono::microseconds elapesed_time, double mesh_publish_period, Eigen::MatrixXd population_grid)
{
  /*
  In questa funzione tengo in considerazione un frame di riferimento e il sucessivo al fine di create una mesh di link tra i punti piu vicini di uno e dell'altro frame.
  Nella versione finale avro un vettore contenetente i triangoli gia ordinati per essere visualizzati in rviz contenenti le mesh di linkage.
  Un frame viene preso come riferimento rispetto ad un altro preso all'istante sucessivo.
  il frame baseline e quello sucessivo vengono linkati solo quando non c'e piu matching di feature tra un frame e quello sucessivo.
  Una matrice di distanza viene creata per trovare i punti del frame baseline piu vicini ai punti del nuovo frame da linjkare.
Le righe della matrice di distanza rappresentano i punti nel frame baseline, le colonne i pyunti nel nuovo frame.
Per ciascuna riga cerco il punto piu vicino corrispondente nella colonna.
Una volta che ho tutti i punti corrispondenti riga colonna, calcolo la media delle distanze e prendo solamente i punti che sono al di sotto della media.
In questa maniera avro i punti che minimizzano la distanza con il nuovo frame, qualsiasi sia la direzione.
I punti di link vengono inviati a pcl che crea una mesh.
I punti del frame baseline, del new frame , del linkage vengo inseriti nella triangulation di RViz per essere rappresentata.

TODO: Convertire il frame nuovo nelle coordinate del frame baseline, effettuare oi calcoli in questo frame, poi convertire successivamente nel world frame.
FAr ela procedura per tot frame e rappresentare in RViz solo quelli gia visualizzati nel world frame.
  */
  std::ofstream outFile1("/home/arpl/luca_ws/point1.txt");

  // Se feature matching false >> take new frame as baseline and update the distance matrix
  // kimera_data->feature_matching_flag = false;
  /// cout << "Matching Flag: " << kimera_data->feature_matching_flag << endl;
  // cout << "Frame counter: " << frame_counter << endl;
  if (kimera_data->feature_matching_flag == true)
  {
    counter_inside = 0;
    // cout << "counter resetted : " << counter_inside << endl;
  }

  if (kimera_data->feature_matching_flag == false && kimera_data->use_voxblox_pc == false) //&& frame_counter == 1 || frame_counter == 5)
  {
    std::ofstream outFile3("/home/arpl/luca_ws/triangles_vertices.txt");
    counter_inside = counter_inside + 1;
    // C++ code goes faster--> it enters multiple time inside the previous if while the feature_matching_flag is still false.
    //  Entering multiple time the comparison cloud is not from the new baseline but is the next one.
    //  In this way it prevented to enter multiople times
    //  Rotate Feature Points cloud in world frame

    if (counter_inside == 1)
    {
      // rotate_voxl3_pc_in_world_frame(kimera_data);
      // kimera_data->pc_W_final.push_back(kimera_data->point_cloud_W); // point_cloud_W
      baseline_counter = baseline_counter + 1;
      cout << "Update Baseline counter " << baseline_counter << endl;
    }

    // Obtain
  
    // Update distance matrix
    if (kimera_data->pc_W_final.size() > 1 && kimera_data->pc_W_final[0].size() > 1 && kimera_data->pc_W_final[1].size() > 1 )
    {
      cout << counter_inside << endl;
      // Every time a new match is generated the triangle list must be cleared --> in future impl myust be take into account of previous iteration until the mesh is sent to unity
      // kimera_data->triangles.points.clear();
     
      // Fill kimera data triangles RVIZ with the kimera mesh taken at frame 1 and 5, (the vertices present in pc_W_final of both frames)
      fill_RVIZ_triangulation_with_new_kimera_frames(kimera_data);
      int n_rows = kimera_data->pc_W_final[0].size(); // Vertices in mesh of baseline frame
      int n_cols = kimera_data->pc_W_final[1].size(); // Vertices in mesh of new frame
      // cout << "n_rows: " << n_rows << " n_cols: " << n_cols << endl;
      kimera_data->distance_GF.resize(0, 0);
      kimera_data->distance_GF.resize(n_rows, n_cols);

      // Fill the eigen matrix with 3D distance values between mesh verttices in both frames.

      float distance_xyz = 0.0;
      vector<float> distance_vector_row;
      double min_value_per_row = 0.0;
      int min_element_index = 0;
      vector<float> min_value_per_row_v;
      vector<float> min_index_per_row_v;

      for (int i = 0; i < n_rows; ++i)
      {
        distance_vector_row.clear();
        for (int j = 0; j < n_cols; ++j)
        {
          // Itero su punti frame 0
          float x_frame0 = kimera_data->pc_W_final[0][i].x;
          float y_frame0 = kimera_data->pc_W_final[0][i].y;
          float z_frame0 = kimera_data->pc_W_final[0][i].z;

          // itero su punti frame 1
          float x_frame1 = kimera_data->pc_W_final[1][j].x;
          float y_frame1 = kimera_data->pc_W_final[1][j].y;
          float z_frame1 = kimera_data->pc_W_final[1][j].z;

          // per ciascuna riga calcolo la distanxza con ciascuna colonna
          float distance = sqrt(pow(x_frame1 - x_frame0, 2) + pow(y_frame1 - y_frame0, 2) + pow(z_frame1 - z_frame0, 2));
          distance_vector_row.push_back(distance); // contine eciascuna riga
          // kimera_data->distance_GF(i,j) = distance;
          // cout << "distance_vector_row[j]: " << distance_vector_row[j] << endl;
        }
        // Save the value and index of the min value per each rows
        min_value_per_row = *min_element(distance_vector_row.begin(), distance_vector_row.end());
        min_element_index = min_element(distance_vector_row.begin(), distance_vector_row.end()) - distance_vector_row.begin();

        // Store min value and min index in a vector
        min_value_per_row_v.push_back(min_value_per_row);
        min_index_per_row_v.push_back(min_element_index);
      }

      // Evaluate the mean between the distances of all the closer vertices per each row
      double percentage = 50;

      double mean_min_val = avg(min_value_per_row_v, percentage);
      // cout << "Mean Value: " << mean_min_val << endl;
      // Search for elements in minVal below the mean value
      vector<float> min_value_under_mean_v;
      vector<float> min_index_under_mean_v;
      vector<int> rows_under_mean_v;
      vector<int> prev_selected_col_v;
      bool check_col_exist = false;

      for (int i = 0; i < min_value_per_row_v.size(); ++i)
      {
        if (min_value_per_row_v[i] < mean_min_val)
        {
          // Avoid to add the same column if it was selected before
          // prev_selected_col_v.push_back(min_index_per_row_v[i]);
          for (int k = 0; k < prev_selected_col_v.size(); k++)
          {
            if (min_index_per_row_v[i] == prev_selected_col_v[k])
            {
              check_col_exist = true;
              break;
            }
            else
            {
              check_col_exist = false;
            }
          }
          if (check_col_exist == false)
          {
            prev_selected_col_v.push_back(min_index_per_row_v[i]);
            min_value_under_mean_v.push_back(min_value_per_row_v[i]);
            min_index_under_mean_v.push_back(min_index_per_row_v[i]); // corrisponde ad una colonna per ciascuna riga
            rows_under_mean_v.push_back(i);
            // outFile1 <<min_index_per_row_v[i] << "\n";
          }

          // rows_under_mean_v.push_back(i); //tiene traccia della riga che contiene il match al di sotto della media
        }
      }
      /*
                for (int k = 0; k < min_index_under_mean_v.size(); k++)
                {
                    cout << "col: " << min_index_under_mean_v[k] << endl;
                }
      */
      min_value_per_row_v.clear();
      min_index_per_row_v.clear();

      // Search the vertices coo related to the min value under the mean
      geometry_msgs::Point p;
      int index = 0;
      /* In this loop are selected the correpondant link vertices coordinates for frame 0 and frame 1,
        related to the indecies obtained before.
        Frame 0 cooresponnd to the rows in the distance matrix like Frame1 corresponds to the cols.
        The index of the rows below that presebt a correspondance with a vertex in the other frame below the mean are stored
        in the vector rows_under_mean_v.
        The indeces of the selected cols for each selected row are stored in min_index_under_mean_v.
        */
     // cout << "Rows size: " << rows_under_mean_v.size() << endl;
      for (int i = 0; i < rows_under_mean_v.size(); ++i)
      {

        // Frame 0 vertices ---> correspondant to rows in the distance matrix
        index = rows_under_mean_v[i];
        p.x = kimera_data->pc_W_final[0][index].x;
        p.y = kimera_data->pc_W_final[0][index].y;
        p.z = kimera_data->pc_W_final[0][index].z;
        kimera_data->link_vertices_v.push_back(p);
        // outFile1 <<  p.x << ", " <<  p.y << ", " <<  p.z <<"\n";
      }
      for (int i = 0; i < min_index_under_mean_v.size(); i++)
      {
        // Frame1 vertices ---> correspondant to cols in the distance matrix
        p.x = kimera_data->pc_W_final[1][min_index_under_mean_v[i]].x;
        p.y = kimera_data->pc_W_final[1][min_index_under_mean_v[i]].y;
        p.z = kimera_data->pc_W_final[1][min_index_under_mean_v[i]].z;
        kimera_data->link_vertices_v.push_back(p);
        outFile1 << p.x << ", " << p.y << ", " << p.z << "\n";
      }

      if (rows_under_mean_v.size() > 0)
      {
        // Send the linkage vertices to the Mesh Constructor to obtain polygons of the connecting mesh between the kimera mesh of two consecutive frames
        Mesh_Const->create_mesh_linkage_polygons(kimera_data->link_vertices_v);
        kimera_data->link_vertices_v.clear();
      }

      // Once a while i have the indices i can search the vertices coordinates in the previous vector. (each element in minIndex is the minimum elements in the correspondant rows of the distance matrix)
    }

    // cout<< " SIZE: " <<  kimera_data->pc_W_final.size() << endl;
    // pc_W_final contiene dati correttamente

    // Mesh_Const->create_polygons(kimera_data->pc_W_final);
    // Save vertyices in the array to be sent to mesh generator
    shape_msgs::Mesh cloud_W_msgs;

    if (Mesh_Const->obtain_world_mesh_as_sensor_msgs().size() > 0)
    {
      cloud_W_msgs = Mesh_Const->obtain_world_mesh_as_sensor_msgs()[0];
      // cout << "size_triangles: " << cloud_W_msgs.triangles.size() << endl;
      kimera_data->data_published_to_rviz = true;

      size_t size_triangles = cloud_W_msgs.triangles.size();
      // cout << "size_triangles: " << size_triangles << endl;

      // Resize the visualization marker msgs to accept three vertex for each triangle
      // kimera_data-> triangles.points.resize(size_triangles*3);
      size_t i = 0;
      int counter = 0;
      // Iterate on the number of triangles
      for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index)
      {
        geometry_msgs::Point p;
        int index1 = cloud_W_msgs.triangles[tri_index].vertex_indices[0];

        geometry_msgs::Point p1;
        int index2 = cloud_W_msgs.triangles[tri_index].vertex_indices[1];

        geometry_msgs::Point p2;
        int index3 = cloud_W_msgs.triangles[tri_index].vertex_indices[2];

        if (index1 == index2)
        {
          cout << "Degenerate triangles: remove" << endl;
          continue;
        }
        if (index1 == index3)
        {
          cout << "Degenerate triangles: remove" << endl;
          continue;
        }
        if (index2 == index3)
        {
          cout << "Degenerate triangles: remove" << endl;
          continue;
        }

        p = cloud_W_msgs.vertices[index1];
        p1 = cloud_W_msgs.vertices[index2];
        p2 = cloud_W_msgs.vertices[index3];
        float x = kimera_data->camera_odometry_position.x;
        float y = kimera_data->camera_odometry_position.y;
        float z = kimera_data->camera_odometry_position.z;
        if (filter_triangles(p, p1, p2, x, y, z) == true)
        {
          cout << "Triangle Skipped" << endl;
          continue;
        }
        else
        {
          // inserisco il vertice nella lista dei triangoli di RVIz
          kimera_data->triangles.points.push_back(p);
          // inserisco il vertice nella lista dei triangoli di RVIz
          kimera_data->triangles.points.push_back(p1);
          // inserisco il vertice nella lista dei triangoli di RVIz
          kimera_data->triangles.points.push_back(p2);
        }
        // Check for degenerate triangles

        counter = counter + 3;
      }

      for (int ii = 0; ii < kimera_data->triangles.points.size(); ++ii)
      {
        geometry_msgs::Point pp;
        pp = kimera_data->triangles.points[ii];

        outFile3 << pp.x << ", " << pp.y << ", " << pp.z << endl;
      }
    }
  }

  // Data are published here
  if (elapesed_time.count() > mesh_publish_period)
  {

    // populate triangle list for the exportation in rviz
    // populate_triangle_list()
    if (kimera_data->use_voxblox_pc == false)
    {
      if (kimera_data->triangles.points.size() > 0)
      {
        kimera_data->publish_mesh_vertices_to_rviz(kimera_data->triangles);
        cout << "Data Published" << endl;
        kimera_data->data_published_to_rviz = true;
      }
    }

    Mesh_Const->clear_vectors();
    // kimera_data-> point_cloud_W.clear();
    // kimera_data->pc_W_final.clear();
    // kimera_data->mesh_vector_CF.clear();
    kimera_data->mesh_vector_GF.clear();
    kimera_data->voxblox_mesh_vertices.clear();
    kimera_data->triangles_normals.clear();

    last_published_mesh = high_resolution_clock::now();
  }

  // Se la size() di pc_W_final > 2 vuol dire che all'interno del vettore esiste una baseline e
  // un novo frame. quindi vuol dire che devo togliere la vechhia baseline, percio l'elemnto che sitrova in posizione zerpo.

  if (kimera_data->pc_W_final.size() > 1)
  {

    int n = kimera_data->pc_W_final.size();
    int range = kimera_data->pc_W_final.size() - 1;
    kimera_data->pc_W_final.erase(kimera_data->pc_W_final.begin() + (range - 1));
  }

  if (baseline_counter > 1)
  {
    kimera_data->mesh_vector_CF.clear();
    cout << "Kimera data cleaned: " << baseline_counter << endl;
    baseline_counter = 0;
  }

  kimera_data->point_cloud_W.clear();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv,"MappingVoxl");
  

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
   
  
  Kimera_vio_data_for_rosbag kimera_data;
  
  Mesh_constructor Mesh_Const = Mesh_constructor();
  
  int n_waypoints = 0;
  //voxblox::EsdfMap *voxblox_map = new voxblox::EsdfMap();
  //voxblox::EsdfServer voxblox_server_(nh, nh_private);
  // Initialize Rviz Messages to publish vertices as Marker
  initialize_visualization_message(&kimera_data);

  //Fino a qui 
  nh.getParam("/kimera_data_params/use_realsense_pc", kimera_data.use_voxblox_pc);
  nh.getParam("/kimera_data_params/receive_pc_from_image_proc", kimera_data.pc_from_image_proc);
  nh.getParam("/kimera_data_params/voxl3_odometry", kimera_data.voxl3_odometry);
  nh.getParam("/kimera_data_params/with_holo", kimera_data.with_holo);

  
  // Initialize matrix related to the grid
  Eigen::MatrixXd population_grid(100, 100);
  auto start = high_resolution_clock::now();
  auto elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
  auto mesh_publish_period = 3000000.0;

  
  //Save the list of trinagles in different files in order to recreate the complete map offline --> Only for testing 
   //Create Directory for print txt files:
   string folder_name = "Map_2";
   kimera_data.stringpath = "/home/luca/luca_ws/DATA/TRIANGULATED_MAP_SURFACES/" + folder_name + "/";
   int status = mkdir( kimera_data.stringpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
   if (status != 0)
   {
     cout << "[TELE CONTROL] Impossible to create folder to store txt output files" << endl;
   }


  bool call_restart_mapping_service = false;
  int counter_restart_mapping = 0;
  
  int publish_way_counter = publish_way_counter + 1;
  ros::Rate r(50);
  while (nh.ok())
  {
   

    // Reoredering the Vertices and translate them in marker msgs for a complete vuisualization nin Rviz
    // Take the Point Cloud array and divide it in triplets
    if (kimera_data.flagKimeraRealsensePointCloud == true)
    {
        populate_RVIZ_triangle_list_with_voxblox_pc(&kimera_data, elapesed_time, mesh_publish_period);
    }
   
  
      // populate_RVIZ_triangle_list_with_kimera_pc(&kimera_data, &Mesh_Const, elapesed_time, mesh_publish_period, population_grid);
   

    if (kimera_data.flagKimeraMesh == true)
    {
      // cout << kimera_data.mesh_point_x[0] << kimera_data.mesh_point_y[0] << kimera_data.mesh_point_z[0] << endl;
      frame_counter = frame_counter + 1;
    }

    if (frame_counter > 5)
    {
      frame_counter = 0;
    }

    //Try to obtain the distance from a position in the esdf map in voxblox
    Eigen::Vector3d position;
    double distance = 0.0; 
    position << 1, 0, 1;
    
    //Call Service clear map if received the command to restart mapping from unity 
    
    if (kimera_data.restart_mapping_flag)
    {
      call_restart_mapping_service = true;
    }

    if (call_restart_mapping_service)
    {
      bool value_call;
      value_call = kimera_data.call_clear_map_service();
      cout << "Map Cleaned Successfully" << endl;
      if (counter_restart_mapping > 10)
      {
        call_restart_mapping_service = false;
        counter_restart_mapping = 0;
      }
      counter_restart_mapping = counter_restart_mapping + 1;  

    }

    //cout << "ESDF Disatnce: " <<distance << endl;
    

    
    
    // cout << kimera_data.camera_odometry_position.x <<", " << kimera_data.camera_odometry_position.y <<", " << kimera_data.camera_odometry_position.z << endl;
    // cout << kimera_data.camera_odometry_orientation.x <<", " <<  kimera_data.camera_odometry_orientation.y << ", " << kimera_data.camera_odometry_orientation.z << endl;
    kimera_data.point_x.clear();
    kimera_data.point_y.clear();
    kimera_data.point_z.clear();

    kimera_data.mesh_point_x.clear();
    kimera_data.mesh_point_y.clear();
    kimera_data.mesh_point_z.clear();

    kimera_data.point_cloud_W.clear();
    // kimera_data.camera_odometry_position.clear();
    // kimera_data.camera_odometry_orien if (kimera_data.voxl3_odometry == 2)
    
    kimera_data.flagKimeraMesh = false;
    kimera_data.flagKimeraPointCloud = false;
    kimera_data.flagKimeraOdometry = false;
    kimera_data.flagboolFeatureMatching = false;
    kimera_data.flagKimeraRealsensePointCloud = false;
    kimera_data.flagKimeraImageProcPointCloud = false;
    kimera_data.flagDragonflyImu = false;
    kimera_data.flagDragonflyPose = false;
    kimera_data.flagVoxl3Pose = false;
    kimera_data.flagboolStartMapping = false;
    kimera_data.flagboolRestartMapping = false;
    kimera_data.flagVoxl3VioPose = false;
    kimera_data.flagQuadrotorSimPose = false;
    kimera_data.flagHoloPosition = false;
    kimera_data.flagInteractiveMarkerPosition = false;

    /*#######################################################################
    ########################### TELEMETRY ######################## */
    cout << "############################################" << endl;
    cout << "Position: X: " << kimera_data.voxl3_position.x << " Y: " << kimera_data.voxl3_position.y << " Z: " << kimera_data.voxl3_position.z << endl;// x_camera
    cout << "Yaw: " << kimera_data.voxl3_euler_orientation.z << endl;
    
    publish_way_counter = publish_way_counter + 1;

    start = high_resolution_clock::now();
    elapesed_time = duration_cast<microseconds>(start - last_published_mesh);

    //kimera_data.publish_dragonfly_tf();
    if (kimera_data.voxl3_odometry == 0)
    {
      //Only sim: publish tf between simulation adn mocap as link for voxblox which publish on mocap_sr
        kimera_data.publish_mocap_sim_tf();
        kimera_data.publish_mocap_to_mocap_sr_tf();
    }
    
    if (kimera_data.voxl3_odometry ==1)
    {
      //This case correspond to fly the real drone via rviz and the tf between mocap and mocap_sr is published by drone teleoperation launch file
         kimera_data.publish_voxl3_tf();
          //kimera_data.publish_mocap_sim_tf();
        // kimera_data.publish_mocap_to_mocap_sr_tf();
         kimera_data.publish_from_mocap_to_world_tf();
         kimera_data.publish_unity_mocap_to_mocap_sr_tf();
    }
    
    if (kimera_data.voxl3_odometry == 2)
    {
       //Real + Holo: publish tf between mocap adn mocap_sr as link for voxblox which publish on mocap_sr
       kimera_data.publish_unity_mocap_to_mocap_sr_tf();
       kimera_data.publish_voxl3_tf();
       kimera_data.publish_from_mocap_to_world_tf();
       kimera_data.publish_hololens_pose_in_RVIZ_frame_mocap_sr();
       kimera_data.publish_hololens_pose_in_RVIZ_frame_world();
    }

  
       kimera_data.publish_interactive_marker_position_in_rviz();

       //Only when a rosbag is played back to visualize corrrectly everithing on the hololens 
       kimera_data.publish_start_assistive_guidance_flag(kimera_data.start_assistive_guidance);
       kimera_data.publish_exit_search_mode(kimera_data.exit_search_mode);
    
    
  
    //kimera_data.publish_realsense_tf();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
