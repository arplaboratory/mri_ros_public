#ifndef _SU_UNITY_COMM_H
#define _SU_UNITY_COMM_H

#include <ros/ros.h>
#include "scene_understanding_pkg_msgs/MeshPos.h"
#include "scene_understanding_pkg_msgs/MeshPosArray.h"
#include "scene_understanding_pkg_msgs/MeshRelatedData.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
class SU_Unity_comm_Impl;



class SU_Unity_comm
{

    ros::NodeHandle nh_;
    
    tf::TransformBroadcaster sim_base_link_broadcaster;
    tf::TransformBroadcaster holo_to_sim_broadcaster;
    //tf::TransformBroadcaster mocap_to_sim_broadcaster;

    //Place Declaration Service here
    //Place Declaration Subscriber Here
    //Place Declaration Publisher Here
    ros::Subscriber MeshPosition;
    ros::Subscriber HoloPosition;
    ros::Subscriber MeshIndices;
    ros::Subscriber MeshVerticesNumber;
    ros::Subscriber MeshIndicesNumber;
    ros::Subscriber drone_pose_unity_frame;
    ros::Subscriber mocap_scene_root_in_unity_world;

    ros::Publisher marker_pub_vertices_rviz;
    ros::Publisher marker_pub_mesh_perimeter_rviz;
    // ros::Publisher holo_position_rviz;
    // ros::Publisher drone_pose_rviz;
    ros::Publisher holo_mesh_pointcloud;


public:
    float MeshVertex_x = 0.0;
    float MeshVertex_y = 0.0;
    float MeshVertex_z = 0.0;
    vector<float> Mesh_vertices_x;
    vector<float> Mesh_vertices_y;
    vector<float> Mesh_vertices_z;
    
  

   
    vector<vector<float>> Meshes_vertices;
    vector<vector<float>> new_Meshes_vertices; // Meshes vector after being reoredered for bein displayed in Rviz
    // Obtain Mesh Indices data
    vector<int> Meshes_indices_array;
    vector<int> Meshes_indices_number_array;
    vector<int> Meshes_vertices_number_array;
    
    

    float Holo_pos_x = 0.0;
    float Holo_pos_y = 0.0;
    float Holo_pos_z = 0.0;
    
    Eigen::Vector3d Holo_position;
    Eigen::Vector4d Holo_orientation_quat;
    
    Eigen::Vector3d drone_position_unity_frame;
    Eigen::Vector4d drone_orientation_unity_frame;

    geometry_msgs::Point mocap_sr_unity_world_position;
    geometry_msgs::Vector3 mocap_sr_euler_angles_unity_world;
    geometry_msgs::Quaternion mocap_sr_orientation_unity_world;
    
    //Initialize Visualization Marker Data to display Meshes in Rviz
    visualization_msgs::Marker triangles, perimeter_lines;
    //Initialize Visual Msgs for hololens position in rviz
     visualization_msgs::Marker sphere, drone_sphere;

    float MeshVertex_x_old = 0.0;
    float MeshVertex_y_old = 0.0;
    float MeshVertex_z_old = 0.0;

    bool flagMeshVerticesPosition = false;
    bool flagMeshIndices = false;
    bool flagMeshVerticesNumber = false;
    bool flagMeshIndicesNumber = false;
    bool flagHoloPosition = false;
    bool flagDronePoseFromUnity = false;

    SU_Unity_comm()
    {

        //Services
        //Subscribers
        MeshPosition = nh_.subscribe("meshes_vertices_coo", 50000, &SU_Unity_comm::meshes_vertices_callback, this);
        HoloPosition = nh_.subscribe("hololens_position", 50000, &SU_Unity_comm::holo_position_callback, this);
        MeshIndices = nh_.subscribe("meshes_vertices_indices_list", 50000, &SU_Unity_comm::meshes_indices_callback, this);
        MeshVerticesNumber = nh_.subscribe("meshes_vertices_number", 50000, &SU_Unity_comm::meshes_vertices_number_callback, this);
        MeshIndicesNumber = nh_.subscribe("meshes_indices_number", 50000, &SU_Unity_comm::meshes_indices_number_callback, this);
        drone_pose_unity_frame = nh_.subscribe("drone_pose_from_holo_unity_frame", 50000, &SU_Unity_comm::drone_pose_from_holo_unity_callback, this);
        //mocap_scene_root_in_unity_world = nh_.subscribe("from_unity/mocap_frame_in_unity_world_coo", 50000, &SU_Unity_comm::mocap_scene_root_in_unity_frame_callback, this);
        

        //Publisher
        marker_pub_vertices_rviz = nh_.advertise<visualization_msgs::Marker>("mesh_triangles_vertices_marker", 10);
        marker_pub_mesh_perimeter_rviz = nh_.advertise<visualization_msgs::Marker>("mesh_triangles_marker_wireframe", 10);
        //holo_position_rviz = nh_.advertise<visualization_msgs::Marker>("hololens_frame", 10);
        //drone_pose_rviz = nh_.advertise<visualization_msgs::Marker>("drone_frame", 10);
        holo_mesh_pointcloud = nh_.advertise<sensor_msgs::PointCloud2>("holo_mesh_pointcloud_world", 1);
       
    }

    ~SU_Unity_comm()
    {
    }

// Mesh Vertices cakllback
    void meshes_vertices_callback(const scene_understanding_pkg_msgs::MeshPosArray &msg)
    {
        Meshes_vertices.clear();
        Mesh_vertices_x.clear();
        Mesh_vertices_y.clear();
        Mesh_vertices_z.clear();
        
        cout << "VerticesMSG Array Size: " << msg.data_x.size() << endl;
        for (int i = 0; i < msg.data_x.size(); i++)
        {
            //Store x, y, z positio of the mesh point exported from unity
            vector<float> vertices;
            vertices.push_back(msg.data_x[i]);
            vertices.push_back(msg.data_y[i]);
            vertices.push_back(msg.data_z[i]);
            Meshes_vertices.push_back(vertices);
            
            
            Mesh_vertices_x.push_back(msg.data_x[i]);
            Mesh_vertices_y.push_back(msg.data_y[i]);
            Mesh_vertices_z.push_back(msg.data_z[i]);
           
            
        }
       
        flagMeshVerticesPosition = true;
    }

// Mesh indices cakllback
    void meshes_indices_callback(const scene_understanding_pkg_msgs::MeshRelatedData &msg)
    {
        Meshes_indices_array.clear();
        
         cout << "Indicices List MSG Array Size: " << msg.data.size() << endl;
        for (int i = 0; i < msg.data.size(); i++)
        {
            Meshes_indices_array.push_back(msg.data[i]);
             
        }
        flagMeshIndices = true;
    }

// Mesh vertices number cakllback
    void meshes_vertices_number_callback(const scene_understanding_pkg_msgs::MeshRelatedData &msg)
    {
        Meshes_vertices_number_array.clear();
        cout << "vertices number List MSG Array Size: " << msg.data.size() << endl;
        for (int i = 0; i < msg.data.size(); i++)
        {
            Meshes_vertices_number_array.push_back(msg.data[i]);
        }
        flagMeshVerticesNumber = true;
    }

    void meshes_indices_number_callback(const scene_understanding_pkg_msgs::MeshRelatedData &msg)
    {
        cout << "Indicices number MSG Array Size: " << msg.data.size() << endl;
        Meshes_indices_number_array.clear();
        for (int i = 0; i < msg.data.size(); i++)
        {
            Meshes_indices_number_array.push_back(msg.data[i]);
           
        }
        flagMeshIndicesNumber = true;
    }





    void holo_position_callback(const geometry_msgs::Pose &msg)
    {

        Holo_pos_x = msg.position.x;
        Holo_pos_y = msg.position.y;
        Holo_pos_z = msg.position.z;
        
        Holo_position << msg.position.x, msg.position.y, msg.position.z;
        Holo_orientation_quat << msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w;
        flagHoloPosition = true;
    }

    void drone_pose_from_holo_unity_callback(const geometry_msgs::PoseStamped &msg)
    {
      // Use this to display the position of the drone in Rviz
      drone_position_unity_frame << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
      drone_orientation_unity_frame << msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w;
     

      flagDronePoseFromUnity = true; 

    }

    /*

    void mocap_scene_root_in_unity_frame_callback(const geometry_msgs::PoseStamped &msg)
    {
        //Mocap root in Unity world frame 
        mocap_sr_unity_world_position.x = msg.pose.position.x;
        mocap_sr_unity_world_position.y = msg.pose.position.y;
        mocap_sr_unity_world_position.z = msg.pose.position.z;
         

        mocap_sr_orientation_unity_world = msg.pose.orientation;
        // Covert to euler angles
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // the found angles are written in a geometry_msgs::Vector3

        mocap_sr_euler_angles_unity_world.x = roll;
        mocap_sr_euler_angles_unity_world.y = pitch;
        mocap_sr_euler_angles_unity_world.z = yaw;
       
        publish_mocap_to_sim_tf();


    }
*/
    void publish_mesh_vertices_to_rviz(visualization_msgs::Marker triangles)
    {
        marker_pub_vertices_rviz.publish(triangles);
    }

    void publish_mesh_triangles_perimeter_to_rviz(visualization_msgs::Marker lines)
    {
        marker_pub_mesh_perimeter_rviz.publish(lines);
    }

    // void publish_holo_pose_to_RVIZ(visualization_msgs::Marker arrow_x)
    // {
    //     holo_position_rviz.publish(arrow_x);
    // }

    // void publish_drone_pose_to_RVIZ(visualization_msgs::Marker drone_sphere_marker)
    // {
    //     holo_position_rviz.publish(drone_sphere_marker);
    // }


     void publish_mesh_point_cloud_to_rviz(vector<geometry_msgs::Point> triangles)
    {
        // iterate on the pc vector and publish each pc independently
        // for (int ii = 0; ii < point_cloud_realsense_W.size(); ii++)
        //{
        // pcl::PointCloud<pcl::PointXYZ> PointCloud;

        PointCloud::Ptr msg(new PointCloud);
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "world"; //base_link
        msg->height = 1;
        msg->width = triangles.size();  //point_cloud_realsense_W
        msg->points.clear();
        
        cout << msg->width << endl;
        for (int j = 0; j < msg->width; j++)
        {
            geometry_msgs::Point p;

            p.x = triangles[j].x;
            p.y = triangles[j].y;
            p.z = triangles[j].z;
            //cout << "point: " << p << endl;
           
            msg->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
          
        
        }
        holo_mesh_pointcloud.publish(msg);
        triangles.clear();
     
    }



   

void publish_world_to_sim()
{    
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
  
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();
    geometry_msgs::Quaternion quat_orientation;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap";
    odom_trans.child_frame_id = "world";

    quat_orientation.w = 1.0;
    odom_trans.transform.rotation = quat_orientation;


    sim_base_link_broadcaster.sendTransform(odom_trans);
}


void publish_holo_to_sim_tf()
{
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
  
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();
    geometry_msgs::Quaternion quat_orientation;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap"; //Mettere il frame che dipende da reale a mocap depending on the flag 
    odom_trans.child_frame_id = "hololens"; //Per mappa hololens necessario riferirla ad un frame fisso che viene istanziato quando l'applicazione vine lanciata 
    //Il frame fisso rappresenta il mondo di unity
    // Il fram unity va trasformato rispetto al frame mocap. 
    //La mappa va rappresentata rispetto al frame world_unity legata al frame mocap 

    quat_orientation.x = Holo_orientation_quat(0);
    quat_orientation.y = Holo_orientation_quat(1);
    quat_orientation.z = Holo_orientation_quat(2);

    quat_orientation.w = 1.0;

    odom_trans.transform.translation.x = Holo_position(0);
    odom_trans.transform.translation.y = Holo_position(1);
    odom_trans.transform.translation.z = Holo_position(2);
    odom_trans.transform.rotation = quat_orientation;


    holo_to_sim_broadcaster.sendTransform(odom_trans);
}







    /*
    void publish_cube_pose()
    {
        unity_robotics_demo_msgs::PosRot pose;

        pose.pos_x = x_coo_Holo;
        pose.pos_y = y_coo_Holo;
        pose.pos_z = z_coo_Holo;
        pose.rot_x = x_rot_Holo;
        pose.rot_y = y_rot_Holo;
        pose.rot_z = z_rot_Holo;
        pose.rot_w = w_rot_Holo;
        
        Cube_pose.publish(pose);

    }
   */
};

#endif
