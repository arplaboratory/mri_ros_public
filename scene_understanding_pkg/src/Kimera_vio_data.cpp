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
#include <algorithm>


#include "Kimera_vio_data.h"
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

void initialize_visualization_message(Kimera_vio_data *kimera_data)
{
  kimera_data->triangles.points.clear();
  kimera_data->obs_magn_force_b_frame.points.clear();

  kimera_data->triangles.header.frame_id = "map";//"/mocap_sr";
  kimera_data->obs_magn_force_b_frame.header.frame_id = "mocap";

  kimera_data->triangles.header.stamp = ros::Time::now();
  kimera_data->obs_magn_force_b_frame.header.stamp = ros::Time::now();

  // Markers Namespace
  kimera_data->triangles.ns = "triangle_list";
  kimera_data->obs_magn_force_b_frame.ns = "force_arrow";
  // Assign a type of action to each marker
  kimera_data->triangles.action = visualization_msgs::Marker::ADD;
  kimera_data->obs_magn_force_b_frame.action = visualization_msgs::Marker::ADD;

  // Pose -- Since are points the orientation is not important
  kimera_data->triangles.pose.orientation.w = 1.0;
  kimera_data->obs_magn_force_b_frame.pose.orientation.w = 1.0;

  // Marker Type (Arrow, Sphere, points )
  kimera_data->triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  kimera_data->obs_magn_force_b_frame.type = visualization_msgs::Marker::ARROW;

  kimera_data->triangles.id = 0;
  kimera_data->obs_magn_force_b_frame.id = 0;

  // POINTS markers use x and y scale for width/height respectively
  kimera_data->triangles.scale.x = 1;
  kimera_data->triangles.scale.y = 1;
  kimera_data->triangles.scale.z = 1;

  kimera_data->obs_magn_force_b_frame.scale.x = 1;
  kimera_data->obs_magn_force_b_frame.scale.y = 0.05;
  kimera_data->obs_magn_force_b_frame.scale.z = 0.05;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

  // Points are green
  kimera_data->triangles.color.g = 1.0f;
  kimera_data->triangles.color.a = 0.3;
  
 

  kimera_data->obs_magn_force_b_frame.color.g = 1.0f;
  kimera_data->obs_magn_force_b_frame.color.a = 1.0;
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





void check_last_pose_before_transformation(Kimera_vio_data *kimera_data)
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







void rotation_camera_to_world_frame(Kimera_vio_data *kimera_data)
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

void fill_grid(Kimera_vio_data *kimera_data, Eigen::MatrixXd mat)
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

void fill_RVIZ_triangulation_with_new_kimera_frames(Kimera_vio_data *kimera_data)
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
void populate_RVIZ_triangle_list_with_voxblox_pc(Kimera_vio_data *kimera_data, std::chrono::microseconds elapesed_time, double mesh_publish_period)
{
   

  
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

    // // Fill the arrows list with normals
    // int counter_normal = 0;
    // for (int point_index = 0; point_index < kimera_data->triangles_normals.size() / 2; point_index++)
    // {
    //   geometry_msgs::Point p;
    //   p = kimera_data->triangles_normals[counter];
    //   kimera_data->arrows.points.push_back(p);

    //   p = kimera_data->triangles_normals[counter + 1];
    //   kimera_data->arrows.points.push_back(p);

    //   counter_normal = counter_normal + 2;
    // }

    counter_inside_if = counter_inside_if + 1;
    counter_inside_if = 0;

  
  voxblox_size_previous =  kimera_data->voxblox_mesh_vertices.size();
  // Publish Vertices at a given frequency
  
  if (elapesed_time.count() > mesh_publish_period && stop_publish_debug == false)
  {

    if (kimera_data->triangles.points.size() > 0)
    {
     

      kimera_data->publish_mesh_vertices_to_rviz(kimera_data->triangles);
      kimera_data->publish_mesh_to_unity(kimera_data->triangles);
      
      cout << "-----------------------------Data Published-------------------------------------------" << endl;

      //Fill the old triangle list with the published triangles 
      //Cleaning the previous  kimera_data->triangles_old before the new assignment 
      kimera_data->triangles_old = kimera_data->triangles;

      kimera_data->data_published_to_rviz = true;
      kimera_data->counter_triangle_published = kimera_data->counter_triangle_published + 1;
    }
    

  
    kimera_data->triangles.points.clear();     //uncomment to delete the mesh 
    //kimera_data->arrows.points.clear();
  
    
   
    last_published_mesh = high_resolution_clock::now();
    
      stop_publish_debug = false;
   
  }

  //Clear the voxblox Vector otherwise the same triamngles are added to the Visualization 
  //Marker Message 
    kimera_data->voxblox_mesh_vertices.clear();
    kimera_data->triangles_normals.clear();

}




float align_theta_with_yaw(float theta, float dx, float dy)
{
  float theta_ = 0.0;
  float alfa = 0.0;
  float sigma = 0.0;

  //Robot is up right respect frame S of the point
   if (dx < 0.0 && dy < 0.0 )
   {
      alfa = theta - M_PI/2;
   }
   else if (dx < 0.0 && dy > 0.0 )
   {
    //Robot is down right respect frame S of the point
    alfa = -1*(M_PI/2 - theta); 
   }
   else if (dx > 0.0 && dy < 0.0)
   {
    //Robot is up left respect frame S of the point
    sigma = M_PI - theta;
    alfa = M_PI/2 + sigma;
   }
   else 
   {
     alfa = - M_PI/2 - theta;
   }

   return alfa;
}


void saturate_force_value(float *dx_sat, float *dy_sat,  float dx_sum, float dy_sum, float sat_x_value, float sat_y_value)
{
   
   if (dx_sum > sat_x_value)
   {
     *dx_sat = sat_x_value;
   }
   else if (dx_sum < -sat_x_value)
   {
    *dx_sat= -sat_x_value;
   }
   else
   {
     *dx_sat = dx_sum;
   }

   if (dy_sum > sat_y_value)
   {
     *dy_sat = sat_y_value;
   }
   else if (dy_sum < -sat_y_value)
   {
    *dy_sat= -sat_y_value;
   }
   else
   {
    *dy_sat = dy_sum;
   }

 
}


void euler_to_quat(Eigen::Vector4f *quat, float roll, float pitch, float yaw)
{
  float qx, qy, qz, qw;
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);

  *quat << qx, qy, qz, qw;
}


void publish_obs_force_array_marker(Kimera_vio_data *kimera_data, float theta, float Fmag)
{
   float roll = 0.0;
   float pitch = 0.0;
   float yaw = theta;

   Eigen::Vector4f quat;
   euler_to_quat(&quat, roll, pitch, yaw);
  //Convert theta from radians orientation (0,0, theta) only on the yaw to a quaternion value. 

  kimera_data->obs_magn_force_b_frame.pose.orientation.x = quat(0);
  kimera_data->obs_magn_force_b_frame.pose.orientation.y = quat(1);
  kimera_data->obs_magn_force_b_frame.pose.orientation.z = quat(2);
  kimera_data->obs_magn_force_b_frame.pose.orientation.w = quat(3);
  
  //Position is the position of the robot
  kimera_data->obs_magn_force_b_frame.pose.position.x  = ( kimera_data->mocap_sr_unity_world_position.x + kimera_data->voxl3_position.x); 
  kimera_data->obs_magn_force_b_frame.pose.position.y  = ( kimera_data->mocap_sr_unity_world_position.y + kimera_data->voxl3_position.y); 
  kimera_data->obs_magn_force_b_frame.pose.position.z  = kimera_data->voxl3_position.z; 

   kimera_data->obs_magn_force_b_frame.scale.x = 0.2 * Fmag;
   kimera_data->publish_force_obs_arrow_to_rviz(  kimera_data->obs_magn_force_b_frame);

}

int getIndex(vector<float> v, float K)
{
 auto it = find(v.begin(), v.end(), K);
 int index = it - v.begin();
 return index;
   
}


float compute_theta_dot(float theta_min_distance, float theta_min_distance_old, float dt)
{
   float theta_dot = (theta_min_distance - theta_min_distance_old)/dt;
   return theta_dot;
}



void evaluate_obstacle_force_field(Kimera_vio_data *kimera_data)
{
  //The pointcloud received from Voxblox is expressed on the frame Map which is aliigne with the frame world.

  //Take the robot position and the surface pointcloud position 
  float horizon = kimera_data->horizon; 
  float theta = 0.0;

  float dx_sum = 0.0;
  float dy_sum = 0.0;
   float Fs = kimera_data->Fs;
  float sat_x_value = Fs;
  float sat_y_value = Fs;
  
 
  float landa = kimera_data->lamda;

 
  vector<float> theta_vec;
  
  

  //Looking for voxels insde the robot horizon 
  pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
  cout << "kimera_data->pc_surface_.size(): " << kimera_data->pc_surface_.size() << endl;
  if (kimera_data->pc_surface_.size() > 0)
  {
     kimera_data->distance_vec.clear();
     kimera_data->F_magn = 0.0;
  }
  for (item = kimera_data->pc_surface_.begin(); item != kimera_data->pc_surface_.end(); item++) 
  {
   
    float dx = item->x - ( kimera_data->mocap_sr_unity_world_position.x + kimera_data->voxl3_position.x); 
    float dy = item->y - (kimera_data->mocap_sr_unity_world_position.y + kimera_data->voxl3_position.y); 
    float dz = item->z - kimera_data->voxl3_position.z; 
    float distance_2D = sqrt(pow((double)dx, 2) + pow((double)dy, 2));
     
    cout << "distance_2D: " <<distance_2D << endl;
    //Samples the points lower that the drone horizon 
    if (distance_2D < horizon && abs(dz) < 0.3) 
    {  
     
      //Define the Force Vector representend in a frame S located on the point and aligned with the World frame (the origin of the vector is in the world frame)
      //The Force vector is defined by the Fx and Fy versor on S, Pointing to the robot with intensity inversely proportional to the distance. 
      //To Evaluate the for ce vector direction i need to find the angle theta on the frame S whic the dscribe the direction of the vector F pointing to the drone 
      //from the origin of the gframe S. 
      Eigen::Vector2f Force_not_scaled; 
      Force_not_scaled << dx, dy; 
       //Evaluate the angle between the world frame and the surface point considered 
      theta =  acos(Force_not_scaled(1)/ distance_2D);
      //Theta needs to be represented as the yaw angle is shown in  the world frame. 
      theta = align_theta_with_yaw(theta, dx, dy);
      
      // //Add all the d

      float k = 1 - exp(horizon);
      kimera_data->F_magn = (Fs/k)*exp(-landa*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      
      //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
      float F_magn_squared = pow((double)kimera_data->F_magn, 2);
      float dy_ = kimera_data->F_magn * sin(theta); 
      float dy_squared = pow((double)dy_, 2);
      float dx_ = sqrt(F_magn_squared - dy_squared);
      
      //Change properly the sign of the x component depending 
      if (dx > 0)
      {
        dx_ = -1*dx_;
      }
      
      dx_sum += dx_;
      dy_sum += dy_;
      
      kimera_data->distance_vec.push_back(distance_2D);
      theta_vec.push_back(theta);
    
    }
  }
  

     //add to the force also the force generated by the virtual obstacles if presents 
    int counter_obs = 0;
     for (int ii = 0; ii <  kimera_data->virtual_obstacles_vec.size()/2; ii++)
     {
        float min_x = kimera_data->virtual_obstacles_vec[counter_obs].x;
        float min_y = kimera_data->virtual_obstacles_vec[counter_obs].y;

        float max_x = kimera_data->virtual_obstacles_vec[counter_obs + 1].x;
        float max_y = kimera_data->virtual_obstacles_vec[counter_obs + 1].y;

        float x_center = (max_x - min_x)/2;
        x_center = max_x -x_center;

        float y_center = (max_y - min_y)/2;
        y_center = max_y -y_center;


       float dx = x_center - kimera_data->voxl3_position.x; 
       float dy = y_center - kimera_data->voxl3_position.x; 

      
       float distance_2D = sqrt(pow((double)dx, 2) + pow((double)dy, 2));
      
        //Samples the points lower that the drone horizon 
    // if (distance_2D < horizon ) 
    // {  
     
     
    //   Eigen::Vector2f Force_not_scaled; 
    //   Force_not_scaled << dx, dy; 
    //    //Evaluate the angle between the world frame and the surface point considered 
    //   theta =  acos(Force_not_scaled(1)/ distance_2D);
    //   //Theta needs to be represented as the yaw angle is shown in  the world frame. 
    //   theta = align_theta_with_yaw(theta, dx, dy);
      
    //   // //Add all the d

    //   float k = 1 - exp(horizon);
    //   kimera_data->F_magn = 0.3*(Fs/k)*exp(-landa*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      
    //   //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
    //   float kimera_data->F_magn_squared = pow((double)kimera_data->F_magn, 2);
    //   float dy_ = kimera_data->F_magn * sin(theta); 
    //   float dy_squared = pow((double)dy_, 2);
    //   float dx_ = sqrt(kimera_data->F_magn_squared - dy_squared);
      
    //   //Change properly the sign of the x component depending 
    //   if (dx > 0)
    //   {
    //     dx_ = -1*dx_;
    //   }
      
    //   dx_sum += dx_;
    //   dy_sum += dy_;

    //   kimera_data->distance_vec.push_back(distance_2D);
    //   theta_vec.push_back(theta);
    
    // }
    counter_obs = counter_obs + 2; 
      
     }

   
  


  //  //Normalize the resultant vector
  float ip = sqrt(pow((double)dx_sum, 2) + pow((double)dy_sum, 2));
  float dx_sum_norm = dx_sum/ip;
  float dy_sum_norm = dy_sum/ip;

  float norm_ip = sqrt(pow(dx_sum_norm,2) + pow(dy_sum_norm,2));

  //Evaluate theta of the current sum vector 
  kimera_data->theta_res = acos(dx_sum_norm/norm_ip);
  if (dy_sum_norm <= 0)
  {
    kimera_data->theta_res = -kimera_data->theta_res;
  }

  



  //Search for the minimum value in the distance vector
  float min_dist = 0.0;
  int index_min_distance = 0.0;
  float theta_min_distance = 0.0;
  float dx_new = 0.0;
  float dy_new = 0.0;
  float dx_fil = 0.0;
  float dy_fil = 0.0;
  float dt = 0.02;
  float theta_dot = 0.0;
  float safe_distance = 0.0;
  
  cout << "kimera_data->distance_vec.size(): " << kimera_data->distance_vec.size() << endl; 
  if (kimera_data->distance_vec.size() > 0 || kimera_data->F_magn > 0.01)
  {

    //average the value of kimera_data->theta_res to avoid big oscillation 
    kimera_data->theta_res = kimera_data->average_theta_val(kimera_data->theta_res);
      // //Compute average of the distances 
    float sum = accumulate( kimera_data->distance_vec.begin(), kimera_data->distance_vec.end(), 0);
    float d_vec_avg = sum/kimera_data->distance_vec.size();

   
    min_dist = *min_element(kimera_data->distance_vec.begin(), kimera_data->distance_vec.end());
    // //Find the index related to the minimum distance element 
    index_min_distance = getIndex(kimera_data->distance_vec, min_dist);

    //  //Search in the angle vector the realted angle theta for this value of distance 
    //theta_min_distance = theta_vec[index_min_distance];


    //EValuate the force from the resultant given the correct Magnitude provided in function of the distance 


    //Copute and check the derivative of theta to avoid big oscillation where in wall cavity or near corners 
    theta_dot = compute_theta_dot(kimera_data->theta_res, kimera_data->theta_min_distance_old, dt);
    
    //The new Magnitude of the vector is given by the exponetial decay relationship evaluated to the element in the surface with the ckoser distance 
    //kimera_data->F_magn = Fs*exp(-landa*min_dist); 
    float k = 1 - exp(horizon);
    kimera_data->F_magn = (Fs/k)*exp(-landa*min_dist)*(1-exp(horizon - min_dist)); //k_coeff/pow((double)distance_2D, 3);
    
    // cout << "theta dot: " <<  theta_dot << endl;
    // if (abs(theta_dot) > 0.40 || kimera_data->safe_flag == true)
    // {
    //   //Keep the last angle to compute the force direction. 
    //   if (kimera_data->safe_flag == false)
    //   {
    //     //First time we enter here 
    //     kimera_data->theta_res = theta_min_distance;
    //     //save the position of the robot respect the world frame the first time it enters here 
    //     kimera_data->position_in_safe_dist =  kimera_data->voxl3_position;

    //   }
     

    //   kimera_data->safe_flag = true;
    // }
    // else
    // {
    //   kimera_data->position_in_safe_dist = kimera_data->voxl3_position;
    // }
    

    //Found the component on the x y axis of the world frame of the related force 
    // dx_new = kimera_data->F_magn * cos(kimera_data->theta_min_distance_);
    // dy_new = kimera_data->F_magn * sin(kimera_data->theta_min_distance_);
    dx_new = kimera_data->F_magn * cos(kimera_data->theta_res);
    dy_new = kimera_data->F_magn * sin(kimera_data->theta_res);
    kimera_data->theta_min_distance_ = kimera_data->theta_res;
    //FIlter the components of the force to avoid big change in the direction of the force 
  
    kimera_data->average_marker_postions_on_multiple_stamps(&dx_fil, &dy_fil, dx_new, dy_new);
    
    //The safe_flag can be turned false only when the distance from the  safe position is bigger that .... cm. 
    // float dist_x = kimera_data->voxl3_position.x -  kimera_data->position_in_safe_dist.x;
    // float dist_y = kimera_data->voxl3_position.y -  kimera_data->position_in_safe_dist.y;
    
    // safe_distance = sqrt(pow((double)dist_x, 2) + pow((double)dist_y, 2));
    // if (safe_distance > 0.2)
    // {
    //   //Change the flag to false--> it mean the user is trying to move the robot away this area
    //   kimera_data->safe_flag = false;
    // }

    

    cout << "theta_min_distance: " << kimera_data->theta_res << endl;
    cout << "dx_new: " << dx_fil << endl;
    cout << "dy_new: " << dy_fil << endl;

    kimera_data->theta_min_distance_old = kimera_data->theta_res;


  }
  else
  {
    kimera_data->F_magn = 0.0;
    kimera_data->distance_vec.clear();

  }
 
  
  
  
  float dx_sum_sat = 0.0;
  float dy_sum_sat = 0.0;
  saturate_force_value(&dx_sum_sat, &dy_sum_sat, dx_fil,dy_fil, sat_x_value, sat_y_value );
  
  //Publosh the arrow representing the magnitude of the onstacle ofrce field centered on the robot frame 
  publish_obs_force_array_marker(kimera_data, kimera_data->theta_min_distance_, kimera_data->F_magn);
 
  //This is fine, try with the sqaured of the cubic of the distance. 
  //If still to high, normalie and put a gain or saturate the Force up to a certain value on dx and dy 
  cout << "kimera_data->F_magn: " << kimera_data->F_magn << endl;
  cout << "dx_sum: " << dx_sum_sat << " dy_sum: " << dy_sum_sat <<  endl;
  kimera_data->publish_obstacle_force_resultant(dx_sum_sat, dy_sum_sat, kimera_data->F_magn);
  
  kimera_data->F_mag = kimera_data->F_magn;
  
}



int main(int argc, char **argv)
{
  ros::init(argc, argv,"MappingVoxl");
  

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
   
  
  Kimera_vio_data kimera_data;
  
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
  nh.getParam("/kimera_data_params/horizon", kimera_data.horizon);
  nh.getParam("/kimera_data_params/lamda", kimera_data.lamda);
  nh.getParam("/kimera_data_params/Fs", kimera_data.Fs);
  nh.getParam("/kimera_data_params/avg_stamps", kimera_data.avg_stamp);
  
  

  
  // Initialize matrix related to the grid
  Eigen::MatrixXd population_grid(100, 100);
  auto start = high_resolution_clock::now();
  auto elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
  auto mesh_publish_period = 3000000.0;

  
  //Save the list of trinagles in different files in order to recreate the complete map offline --> Only for testing 
   //Create Directory for print txt files:
  

  bool call_restart_mapping_service = false;
  int counter_restart_mapping = 0;
  
  int publish_way_counter = publish_way_counter + 1;
  ros::Rate r(50);
  while (nh.ok())
  {
   

    // Reoredering the Vertices and translate them in marker msgs for a complete vuisualization nin Rviz
    // Take the Point Cloud array and divide it in triplets
   
   
    populate_RVIZ_triangle_list_with_voxblox_pc(&kimera_data, elapesed_time, mesh_publish_period);
    
   
  
     
   

    if (kimera_data.flagKimeraMesh == true)
    {
      // cout << kimera_data.mesh_point_x[0] << kimera_data.mesh_point_y[0] << kimera_data.mesh_point_z[0] << endl;
      frame_counter = frame_counter + 1;
    }

    if (frame_counter > 5)
    {
      frame_counter = 0;
    }


    //Function to evaluet the force field from obstacle inside a drone certain horizon.
    evaluate_obstacle_force_field(&kimera_data);

    //Try to obtain the distance from a position in the esdf map in voxblox
    Eigen::Vector3d position;
    double distance = 0.0; 
    position << 1, 0, 1;
    
    //Call Service clear map if received the command to restart mapping from unity 
    

    //TO DO place this inside the callback
    if (kimera_data.reset_map_flag)
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
    // kimera_data.camera_odometry_orientation.clear();
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
    kimera_data.flagSurfacePointcloudVoxblox = false;
    kimera_data.flagRRTVirtualObstacles = false;

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
      //kimera_data.publish_voxl3_tf();
          //kimera_data.publish_mocap_sim_tf();
        // kimera_data.publish_mocap_to_mocap_sr_tf();
         //kimera_data.publish_from_mocap_to_world_tf();
       //kimera_data.publish_unity_mocap_to_mocap_sr_tf();
    }
    
    if (kimera_data.voxl3_odometry == 2)
    {
       //Real + Holo: publish tf between mocap adn mocap_sr as link for voxblox which publish on mocap_sr
       //kimera_data.publish_unity_mocap_to_mocap_sr_tf();
       //kimera_data.publish_voxl3_tf();
       //kimera_data.publish_from_mocap_to_world_tf();
       kimera_data.publish_hololens_pose_in_RVIZ();
    }
    
   
    //kimera_data.publish_realsense_tf();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
