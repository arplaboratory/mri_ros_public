#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>


#include "SU_Unity_comm.h"

using namespace std;
using namespace std::chrono;

//Global variables 
 auto last_published_mesh =  high_resolution_clock::now();

void initialize_visualization_message(SU_Unity_comm *unity_data)
{
  unity_data->triangles.points.clear();

  unity_data-> triangles.header.frame_id =  unity_data-> perimeter_lines.header.frame_id = "/mocap";
  unity_data->triangles.header.stamp  =   unity_data->perimeter_lines.header.stamp = ros::Time::now();
    //Markers Namespace 
  unity_data-> triangles.ns =  unity_data-> perimeter_lines.ns = "triangle_list";
    // Assign a type of action to each marker
  unity_data-> triangles.action =  unity_data-> perimeter_lines.action = visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important
  unity_data-> triangles.pose.orientation.w =  unity_data-> perimeter_lines.pose.orientation.w = 1.0;


    // Marker Type (Arrow, Sphere, points )
  unity_data-> triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;   
  unity_data-> perimeter_lines.type = visualization_msgs::Marker::LINE_LIST;
    
  unity_data-> triangles.id = 0;
  unity_data-> perimeter_lines.id = 0;

    // POINTS markers use x and y scale for width/height respectively
  unity_data-> triangles.scale.x = 1;
  unity_data-> triangles.scale.y = 1;
  unity_data-> triangles.scale.z = 1;
    
  unity_data-> perimeter_lines.scale.x = 0.05;
  unity_data-> perimeter_lines.scale.y = 0.05;
  unity_data-> perimeter_lines.scale.z = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
   


    // Points are green
  unity_data-> triangles.color.g = 1.0f;
  unity_data-> triangles.color.a = 0.3;

  unity_data-> perimeter_lines.color.g = 0.0f;
  unity_data->  perimeter_lines.color.r = 0.0f;
  unity_data-> perimeter_lines.color.b = 0.0f;
  unity_data-> perimeter_lines.color.a = 1.0;
}


void publish_hololens_pose_in_RVIZ(SU_Unity_comm *unity_data)
{
  unity_data->sphere.points.clear();

  unity_data-> sphere.header.frame_id = "/mocap";
  unity_data->sphere.header.stamp  = ros::Time::now();
    //Markers Namespace 
  unity_data-> sphere.ns = "holo_position_in_unity";
    // Assign a type of action to each marker
  unity_data-> sphere.action =  visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important

  unity_data-> sphere.pose.position.x =  unity_data->Holo_position(0);
  unity_data-> sphere.pose.position.y =  unity_data->Holo_position(1);
  unity_data-> sphere.pose.position.z =  unity_data->Holo_position(2);
  



    // Marker Type (Arrow, Sphere, points )
  unity_data-> sphere.type = visualization_msgs::Marker::SPHERE;   
     
  unity_data-> sphere.id = 0;
  
    // POINTS markers use x and y scale for width/height respectively
  unity_data-> sphere.scale.x = 0.3;
  unity_data-> sphere.scale.y = 0.3;
  unity_data-> sphere.scale.z = 0.3;
    
  unity_data-> sphere.color.b = 1.0f;
  unity_data-> sphere.color.a = 1.0;

 // unity_data->publish_holo_pose_to_RVIZ(unity_data->sphere);

}

void publish_drone_pose_in_RVIZ(SU_Unity_comm *unity_data)
{
  unity_data->drone_sphere.points.clear();

  unity_data-> drone_sphere.header.frame_id = "/mocap";
  unity_data->drone_sphere.header.stamp  = ros::Time::now();
    //Markers Namespace 
  unity_data-> drone_sphere.ns = "drone_position_in_unity";
    // Assign a type of action to each marker
  unity_data-> drone_sphere.action =  visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important
  

  unity_data-> drone_sphere.pose.position.x =  unity_data->drone_position_unity_frame(0);
  unity_data-> drone_sphere.pose.position.y =  unity_data->drone_position_unity_frame(1);
  unity_data-> drone_sphere.pose.position.z =  unity_data->drone_position_unity_frame(2);

 


    // Marker Type (Arrow, Sphere, points )
  unity_data-> drone_sphere.type = visualization_msgs::Marker::SPHERE;   
     
  unity_data-> drone_sphere.id = 0;
  
    // POINTS markers use x and y scale for width/height respectively
  unity_data-> drone_sphere.scale.x = 0.3;
  unity_data-> drone_sphere.scale.y = 0.3;
  unity_data-> drone_sphere.scale.z = 0.3;
    
  unity_data-> drone_sphere.color.r = 1.0f;
  unity_data-> drone_sphere.color.a = 1.0;

 // unity_data->publish_drone_pose_to_RVIZ(unity_data->drone_sphere);
}

/*
This Function permits to reorder the mesh vertices as exported from unity, following the indices vertices.
In Unity a qaud is composed by four vertices and six indices, because composed by two triangles.
The vertices in common between the two triangles along the diagonal of the quad MUST be considered as two separated vertices.
It is important to reordered it, following the indeces list, in order to have a correct representatio in RVIZ.
Example: 4 Vertices: V0, V1, V2, V3
Indices: (0, 2, 1)
          (1, 3, 2)
This Means that the list of vertices must be reordered as follow:
(V0, V2, V1)  --> one triangle
(V1, V3, V2)  --> one triangle

This process Must be done for each Mesh exported represented as a sequence of vertices.
The number of vertices and indices for each mesh are contained inside the vectors: vertices_n, indices_n
Each element in  vertices_n gives an information about how many rows you have to consider in vertices vector to recreate the mesh. (points of triangles)
Each element in  indices_n gives an information about how many rows you have to consider in indices vector. 
The element in indices vector represent the related row in vector vertices to consider.

Differently from unity, where you pass the list of indice and vertices, in rviz the vertices in the list must be ordered following the indices list,
because rviz create a triangle considereing the first three point oin the array, then the second three points and so on.
Vedere appunti per maggiori informazioni
*/
void reordering_vertices_list_with_indices(SU_Unity_comm *unity_data)
{
  
  std::ofstream outFile1("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/Vertices.txt");
  std::ofstream outFile2("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/index_n.txt");
  std::ofstream outFile3("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/index.txt");
 std::ofstream outFile4("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/vertices_new.txt");
 std::ofstream outFile5("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/vertices_n.txt");
 std::ofstream outFile6("/home/luca/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/test_July/Exp_5/size_new_indices.txt");

  int row_indices_reached = 0;
  int row_vertices_reached = 0;
  
  vector<float> x_coo;
  vector<float> y_coo;
  vector<float> z_coo;

  vector<float> x_coo_new;
  vector<float> y_coo_new;
  vector<float> z_coo_new;

   unity_data->new_Meshes_vertices.clear();
   cout << " unity_data->Meshes_vertices.size(): " <<  unity_data->Meshes_vertices.size() << endl;
  
  for (int ii = 0; ii <  unity_data->Meshes_vertices.size(); ii++)
  {
    x_coo.push_back(unity_data->Meshes_vertices[ii][0]);
    y_coo.push_back(unity_data->Meshes_vertices[ii][1]);
    z_coo.push_back(unity_data->Meshes_vertices[ii][2]);
     outFile1 <<unity_data->Meshes_vertices[ii][0]  <<", "<<unity_data->Meshes_vertices[ii][1]<<", "<<unity_data->Meshes_vertices[ii][2]<<"\n";
  } 

  for (int i = 0; i < unity_data->Meshes_indices_number_array.size(); i++) //mesh_data->indices_n.size()
  {
    // Create list of indices related to the considered mesh 
    vector<int> new_indices;
    int index_n = unity_data->Meshes_indices_number_array[i];
    outFile2 <<index_n<<"\n";
    for (int ii = 0; ii < index_n; ii++)
    {
      int row_indices_list = row_indices_reached + ii;
      new_indices.push_back(unity_data->Meshes_indices_array[row_indices_list]);
      outFile3 <<unity_data->Meshes_indices_array[row_indices_list]<<"\n";
      //cout << "new_indices i: " <<  new_indices[ii] << endl;
      }

    outFile6 << new_indices.size() << endl;
    for  (int j = 0; j < new_indices.size(); j++)
    {
      //Vertices row to consider and to add to the new vector
      int row = row_vertices_reached + new_indices[j]; //mesh_data->vertices[row_vertices_reached + new_indices[j]];
      //unity_data->new_Meshes_vertices.push_back(unity_data->Meshes_vertices[row]);
    

     x_coo_new.push_back(x_coo[row]);
     y_coo_new.push_back(y_coo[row]);
     z_coo_new.push_back(z_coo[row]);
    
  
      
    }
  
    row_vertices_reached = row_vertices_reached + unity_data->Meshes_vertices_number_array[i];
    //cout << "row_vertices_reached : " <<  row_vertices_reached << endl; 
  
    row_indices_reached = row_indices_reached + index_n;
    //cout << "row_indices_reached : " <<  row_indices_reached << endl;
    outFile5 << unity_data->Meshes_vertices_number_array[i]<<"\n";
  }


  for (int ii = 0; ii <  x_coo_new.size(); ii++)
  {
    vector<float> vert;
    vert.push_back(x_coo_new[ii]);
    vert.push_back(y_coo_new[ii]);
    vert.push_back(z_coo_new[ii]);

    unity_data->new_Meshes_vertices.push_back(vert);
    outFile4 <<x_coo_new[ii]  <<", "<<y_coo_new[ii]<<", "<<z_coo_new[ii]<<"\n";
  }
}


void publish_holo_SU_meshes_in_RVIZ(SU_Unity_comm *unity_data, std::chrono::microseconds elapesed_time, double mesh_publish_period)
{
  
  
  //Publish Datat To Rviz
     if (elapesed_time.count()  > mesh_publish_period)
     {
       
       // Initialize Rviz Messages to publish vertices as Marker
        initialize_visualization_message(unity_data);
         // Reoreder list of vertices following indices list to be correctly displayed by the mesh generator in rviz
        reordering_vertices_list_with_indices(unity_data);

        //the number of triangles in mesh is 1/3 the number of vertices 
        int n_triangles =  unity_data->new_Meshes_vertices.size()/3;
        
    
        int count = 0;
          // Create the vertices for the points and lines
        for (uint32_t i = 0; i < n_triangles; i++)
        {
        
         
          geometry_msgs::Point p;
          //vertex 1
          p.x = unity_data->new_Meshes_vertices[count][0]; //(int32_t)i - 50;  // 2, 0, 1
          p.y = unity_data->new_Meshes_vertices[count][2];
          p.z = unity_data->new_Meshes_vertices[count][1];
          unity_data->triangles.points.push_back(p);

       
          // Vertex 2
          p.x = unity_data->new_Meshes_vertices[count + 1][0]; //(int32_t)i - 50;
          p.y = unity_data->new_Meshes_vertices[count + 1][2];
          p.z = unity_data->new_Meshes_vertices[count + 1][1];
          unity_data->triangles.points.push_back(p);
         
         
          // Vertex 3
          p.x = unity_data->new_Meshes_vertices[count + 2][0]; //(int32_t)i - 50;
          p.y = unity_data->new_Meshes_vertices[count + 2][2];
          p.z = unity_data->new_Meshes_vertices[count + 2][1];
          unity_data->triangles.points.push_back(p);
      
          
          
          count = count + 3;
         
         //
          
         
        }
    
       
        //marker_pub.publish(points);
         unity_data->publish_mesh_vertices_to_rviz(unity_data->triangles);
         unity_data->publish_mesh_point_cloud_to_rviz( unity_data->triangles.points);
         cout << "Points Published!" << endl;
        last_published_mesh = high_resolution_clock::now();
      
        
     }
}




int main( int argc, char** argv )
{
  ros::init(argc, argv, "publish_mesh_to_rviz");
  ros::NodeHandle n;
  
  SU_Unity_comm unity_data;
  
 


  
  auto start = high_resolution_clock::now();
 
  auto elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
  auto mesh_publish_period = 1000000.0;
  
  ros::Rate r(30);
 
  while (ros::ok())
  {
   
  // Publish Meshes receive from hololens to RVIZ
    publish_holo_SU_meshes_in_RVIZ(&unity_data, elapesed_time, mesh_publish_period);
    
  //  publish_hololens_pose_in_RVIZ(&unity_data);
   
   // publish_drone_pose_in_RVIZ(&unity_data);
   
    unity_data.flagMeshVerticesPosition = false;
    unity_data.flagMeshIndices = false;
    unity_data.flagMeshVerticesNumber = false;
    unity_data.flagMeshIndicesNumber = false;
    unity_data.flagDronePoseFromUnity = false;

    //unity_data.publish_mesh_triangles_perimeter_to_rviz(unity_data.perimeter_lines);
    //  unity_data.publish_world_to_sim();
    //  unity_data.publish_holo_to_sim_tf();

     
    start = high_resolution_clock::now();
    elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
    ros::spinOnce();
    r.sleep();

    
  }
}