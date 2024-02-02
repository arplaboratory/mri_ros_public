#include "ros/ros.h"
#include "scene_understanding_pkg_msgs/MeshPos.h"
#include <stdio.h> 
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <sys/stat.h> 
#include <sstream>

#include "SU_Unity_comm.h"

using namespace std::chrono;
using namespace std;





void save_data_in_folder(SU_Unity_comm *Data, string filename, string data_to_save)
{

    std::ofstream outFile1(filename);
    
    if (data_to_save == "Meshes_vertices_coo")
    {
        cout << "Data->Meshes_vertices.size(): " << Data->Meshes_vertices.size() << endl;
        for (int i = 0; i < Data->Meshes_vertices.size(); i++ )
       {
        outFile1 <<Data->Meshes_vertices[i][0]  <<", "<<Data->Meshes_vertices[i][1]<<", "<<Data->Meshes_vertices[i][2]<<"\n";
       }

    }
    else if (data_to_save == "Meshes_indices_list")
    {
       std::ofstream outFile2(filename);
       for (int i = 0; i < Data->Meshes_indices_array.size(); i++ )
       {
           outFile2 <<Data->Meshes_indices_array[i]  <<"\n";
       }

    }
    else if (data_to_save == "Meshes_vertices_number")
    {
        std::ofstream outFile3(filename);
       for (int i = 0; i < Data->Meshes_vertices_number_array.size(); i++ )
       {
           outFile3 <<Data->Meshes_vertices_number_array[i]  <<"\n";
       }

    }
    else if (data_to_save == "Meshes_indices_number")
    {
       std::ofstream outFile4(filename);
       for (int i = 0; i < Data->Meshes_indices_number_array.size(); i++ )
       {
           outFile4 <<Data->Meshes_indices_number_array[i]  <<"\n";
       }

    }
    else
    {
        std::ofstream outFile5(filename);
        
        outFile5 <<Data->Holo_pos_x <<", "<<Data->Holo_pos_y<<", "<< Data->Holo_pos_z<<"\n";

    }
    


}

void write_txt_file(SU_Unity_comm *Data, string data_to_save, int file_num, string stringpath, string file_name)
{
    stringstream ss;
    ss << file_num;
    string num = ss.str();
    string file_namefinal = file_name + num; // + ".txt";
    string name1 = file_namefinal + ".txt";
    string filename = stringpath + name1;
    cout << "filename: " << filename << endl;
    save_data_in_folder(Data, filename, data_to_save);

    
       
    cout << "Mesh Vertex Position Saved!" << endl;
}







int main(int argc, char **argv)
{
    ros::init(argc, argv, "SU_UNITY_COMM");
    ros::NodeHandle nh;
    
    


    SU_Unity_comm Data;
    string folder_name;
    
    nh.getParam("/unity_comm_param/desired_txt_folder_name", folder_name );
    //Create Directory for print txt files: 
    string stringpath = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/" + folder_name + "/";
    int status = mkdir(stringpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0)
    {
        cout << "[SU_UNITY_COMM] Impossible to create folder to store txt output files" << endl;
    }
    
    int file_num = 0;
    
    string data_to_save = "";
    ros::Rate r(1);
    while (nh.ok())
    {
       //Define Cube Positionn
       if ( Data.flagMeshVerticesPosition == true )
       {
         

      
           data_to_save = "Meshes_vertices_coo";
           string file_name = "meshes_vertices_pos_";
           write_txt_file(&Data, data_to_save, file_num, stringpath, file_name);

           data_to_save = "Meshes_indices_list";
           file_name = "meshes_indices_list_";
           write_txt_file(&Data, data_to_save, file_num, stringpath, file_name);

           data_to_save = "Meshes_vertices_number";
           file_name = "meshes_vertices_num_";
           write_txt_file(&Data, data_to_save, file_num, stringpath, file_name);

           data_to_save = "Meshes_indices_number";
           file_name = "meshes_indices_num_";
           write_txt_file(&Data, data_to_save, file_num, stringpath, file_name);


           data_to_save = "Holo_pos";
           file_name = "holo_position_";
           write_txt_file(&Data, data_to_save, file_num, stringpath, file_name);
         
        
        
           file_num = file_num + 1;
    
       }
      
       

       Data.flagMeshVerticesPosition = false;
       Data.flagMeshIndices = false;
       Data.flagMeshVerticesNumber = false;
       Data.flagMeshIndicesNumber = false;

       ros::spinOnce();
       r.sleep();
    }

    

    return 0;
}




