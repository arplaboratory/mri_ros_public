#ifndef _KIMERA_VIO_DATA_H
#define _KIMERA_VIO_DATA_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <visualization_msgs/Marker.h>
#include </home/luca/luca_ws/devel/include/scene_understanding_pkg_msgs/MeshVertexPosition.h>

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
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>

// Image conversion from RosMsgs to OpenCV
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_msgs/Vertices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/filter.h>
#include <shape_msgs/Mesh.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <voxblox_msgs/Mesh.h>

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

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <voxblox/mesh/mesh_utils.h>
#include <scene_understanding_pkg_msgs/waypointArray.h>
#include <scene_understanding_pkg_msgs/waypointMsg.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Kimera_vio_data
{

    ros::NodeHandle nh_;

    // Place Declaration Service here
    // Place Declaration Subscriber Here
    // Place Declaration Publisher Here
    ros::Subscriber kimera_mesh;
    ros::Subscriber kimera_point_cloud;
    ros::Subscriber kimera_vio_odometry;
    ros::Subscriber Realsense_color_image;
    ros::Subscriber realsense_point_cloud;
    ros::Subscriber image_proc_point_cloud; //Use only with euroc datset
    ros::Subscriber dragonfly_imu;
    ros::Subscriber dragonfly_vio_pose;
    ros::Subscriber dragonfly_pointcloud;
    ros::Subscriber quadrotor_sim_pose;
    //Voxl3 Subscribe
    ros::Subscriber voxl3_vio_pose;
    ros::Subscriber voxl3_mocap_pose;

    //From Unity
    ros::Subscriber start_stop_mapping_bool;
    ros::Subscriber restart_mapping;
    
    // Python Flag Subscriber feature matching
    ros::Subscriber python_check_feature_matching;
    // voxblox sub
    ros::Subscriber voxblox_mesh;

    ros::Subscriber mocap_scene_root_in_unity_world; 
    ros::Subscriber holoPosition;

    ros::Publisher marker_pub_vertices_rviz;
    ros::Publisher marker_pub_old_vertices_rviz;
    ros::Publisher marker_pub_arrows_vertices_rviz;
    //Publish Mesh arrays to unity
    ros::Publisher mesh_to_unity;
    ros::Publisher rs_pointcloud_w;
    ros::Publisher mesh_pointcloud_to_unity;
    ros::Publisher dragonfly_odom;
    ros::Publisher holo_position_rviz;




    ros::ServiceClient clear_map_service;
    tf::TransformBroadcaster dragonfly_odom_broadcaster;
    tf::TransformBroadcaster voxl3_odom_broadcaster;
    tf::TransformBroadcaster from_mocap_to_world;
    tf::TransformBroadcaster mocap_to_mocap_sr_broadcaster;
    tf::TransformBroadcaster mocap_to_sim_broadcaster;
    tf::TransformBroadcaster unity_mocap_to_mocap_sr_broadcaster;


    // tf::TransformBroadcaster camera_link_to_voxl;
    // tf::Transform transform_camera_link_to_voxl;
    // tf::TransformListener from_camera_link_to_mocap_sr;
    // tf::StampedTransform from_camera_link_to_mocap_sr_;
    

public:
    //Case from where obtain the odometry
    int voxl3_odometry = 0; 

    typedef vector<double> row_t;
    typedef vector<row_t> points_t;
    vector<double> mesh_point_x;
    vector<double> mesh_point_y;
    vector<double> mesh_point_z;

    vector<double> point_x;
    vector<double> point_y;
    vector<double> point_z;

    vector<geometry_msgs::Point> point_cloud_W;

    vector<vector<geometry_msgs::Point>> pc_W_final;
    vector<geometry_msgs::Point> point_cloud_realsense_W;
    vector<geometry_msgs::Point> image_proc_pointcloud;
    // Save Normals
    vector<geometry_msgs::Point> triangles_normals;

    vector<geometry_msgs::Point> voxblox_mesh_vertices;
    geometry_msgs::Vector3 camera_odometry_position;
    // vector<double> camera_odometry_orientation;
    geometry_msgs::Vector3 camera_odometry_orientation;
 
    shape_msgs::Mesh shape_msg_vertices_world_frame;

    vector<shape_msgs::Mesh> mesh_vector_CF;
    vector<shape_msgs::Mesh> mesh_vector_GF;
    // Initialize Visualization Marker Data to display Meshes in Rviz
    shape_msgs::Mesh shape_msg_mesh;
    visualization_msgs::Marker triangles, arrows, triangles_old, triangles_difference, triangles_old_old;
    
    Eigen::MatrixXf distance_GF;
    vector<geometry_msgs::Point> link_vertices_v;

    cv::Mat new_color_frame;
    cv::Mat bw_resized_frame;
     
     //Dragonfly params
     geometry_msgs::Point dragonfly_imu_angular_vel;
     geometry_msgs::Point dragonfly_imu_lin_acc;
     geometry_msgs::Point dragonfly_position;
     geometry_msgs::Point dragonfly_euler_orientation;
     geometry_msgs::Quaternion dragonfly_quat_orientation;

     //Voxl3 Params
     geometry_msgs::Point voxl3_position;
     geometry_msgs::Point voxl3_euler_orientation;
     geometry_msgs::Quaternion voxl3_quat_orientation;
     
    //Realsense Params
    geometry_msgs::Quaternion camera_quat_orientation;
    
    //MOcap frame in Unity coordinates 
    
    geometry_msgs::Point mocap_sr_unity_world_position;
    geometry_msgs::Vector3 mocap_sr_euler_angles_unity_world;
    geometry_msgs::Quaternion mocap_sr_orientation_unity_world;

    //Hololens position respect world frame 
    geometry_msgs::Point hololens_position;
    geometry_msgs::Quaternion hololens_orientation;

    //Array of pose to synchronize with the pointcloud rotations 
    vector<nav_msgs::Odometry> voxl_pose_array;
    vector<geometry_msgs::Point> voxl_euler_array;
    double last_pointcloud_stamp = 0.0;
  

    geometry_msgs::Point voxl_position_GF_for_rotation;
    geometry_msgs::Vector3 euler_angles_for_pc_rotation;

    string stringpath;
    int counter_triangle_published = 0;

    //Test
    scene_understanding_pkg_msgs::waypointMsg waypoint_msg_to_unity;
    scene_understanding_pkg_msgs::waypointArray waypoint_array_msg_to_unity;

    //from Unity 
    bool start_mapping = false;
    bool restart_mapping_flag = false;
    bool with_holo = false;

//Parameters
   
    bool use_voxblox_pc = false;
    bool pc_from_image_proc = false; // if true the sub choosen is from topic /stereo_gray?dense/pointcloud
    bool data_published_to_rviz = true;
    // Flag from python
    bool feature_matching_flag = false;
    bool wait_for_next_baseline_image = true;

    bool flagKimeraMesh = false;
    bool flagKimeraPointCloud = false;
    bool flagKimeraOdometry = false;
    bool flagRealSenseNewColorFrame = false;
    bool flagboolFeatureMatching = false;
    bool flagKimeraRealsensePointCloud = false;
    bool flagKimeraImageProcPointCloud = false;
    bool flagDragonflyImu = false;
    bool flagDragonflyPose = false;
    bool flagVoxl3Pose = false;
    bool flagboolStartMapping = false; 
    bool flagboolRestartMapping = false;
    bool flagVoxl3VioPose = false;
    bool flagQuadrotorSimPose = false;
    bool flagHoloPosition = false;

    Kimera_vio_data()
    {

        // Services
        clear_map_service = nh_.serviceClient<std_srvs::Empty>("/voxblox_node_voxl_3/clear_map");

        // Subscribers
        kimera_mesh = nh_.subscribe("/kimera_vio_ros/mesh", 50000, &Kimera_vio_data::kimera_vio_mesh_callback, this);
        kimera_point_cloud = nh_.subscribe("/kimera_vio_ros/time_horizon_pointcloud", 50000, &Kimera_vio_data::kimera_vio_pointcloud_callback, this);
        kimera_vio_odometry = nh_.subscribe("/kimera_vio_ros/odometry", 50000, &Kimera_vio_data::kimera_vio_odometry_callback, this);
        Realsense_color_image = nh_.subscribe("/camera/color/image_raw", 50000, &Kimera_vio_data::realsense_color_image_callback, this);
        python_check_feature_matching = nh_.subscribe("/feature_detector_py/feature_matching_flag", 50000, &Kimera_vio_data::python_feature_detector_matching_callback, this);
        //subscribe to Realsense
        voxblox_mesh = nh_.subscribe("/voxblox_node_voxl_3/mesh", 50000, &Kimera_vio_data::voxblox_mesh_block_callback, this);
        realsense_point_cloud = nh_.subscribe("/voxl3/dfs_point_cloud_GF", 50000, &Kimera_vio_data::realsense_pointcloud_callback, this); /// camera/depth/color/points  ///voxl3/dfs_point_cloud
        image_proc_point_cloud =  nh_.subscribe("/stereo_gray/dense_stereo/pointcloud", 50000, &Kimera_vio_data::image_proc_pointcloud_callback, this);
        //subscribe to Dragonfly Topic 
        dragonfly_imu = nh_.subscribe("/dragonfly8/imu", 50000, &Kimera_vio_data::dragonfly_imu_callback, this); 
        dragonfly_vio_pose = nh_.subscribe("/qvio/odometry", 50000, &Kimera_vio_data::dragonfly_vio_pose_callback, this); //dragonfly8/vio_pose
        
        //voxl3_vio_pose =  nh_.subscribe("/voxl3/quadrotor_ukf/control_odom", 50000, &Kimera_vio_data::voxl3_vio_pose_callback, this); //dragonfly8/vio_pose
        voxl3_mocap_pose = nh_.subscribe("/voxl3/odom", 10, &Kimera_vio_data::voxl3_mocap_pose_callback, this); ///voxl3/odom  ///voxl3/quadrotor_ukf/control_odom
        //quadrotor_sim_pose = nh_.subscribe("/quadrotor/odom", 10, &Kimera_vio_data::quadrotor_vio_pose_callback, this);
  
        start_stop_mapping_bool = nh_.subscribe("/from_Unity/start_stop_mapping", 50000, &Kimera_vio_data::from_unity_start_stop_mapping_callback, this);
        restart_mapping = nh_.subscribe("/from_Unity/restart_mapping", 50000, &Kimera_vio_data::from_unity_restart_mapping_callback, this);
        mocap_scene_root_in_unity_world = nh_.subscribe("/from_unity/mocap_frame_in_unity_world_coo", 50000, &Kimera_vio_data::mocap_scene_root_in_unity_frame_callback, this);
        holoPosition = nh_.subscribe("/hololens_position", 50000, &Kimera_vio_data::holo_position_callback, this);

        //deve essere ripubblicata come tf frame riferita al mondo per essere visibile da voxblox
        // dragonfly_pointcloud =  nh_.subscribe("/s/dense_stereo/pointcloud", 50000, &Kimera_vio_data::dragonfly_pointcloud_callback, this);

        /// stereo_gray/dense_stereo/pointcloud
        marker_pub_vertices_rviz = nh_.advertise<visualization_msgs::Marker>("kimera_mesh", 10);
        marker_pub_old_vertices_rviz = nh_.advertise<visualization_msgs::Marker>("kimera_mesh_old", 10);
        marker_pub_arrows_vertices_rviz = nh_.advertise<visualization_msgs::Marker>("surface_normals", 10);
        rs_pointcloud_w = nh_.advertise<sensor_msgs::PointCloud2>("rs_pointcloud_world", 1);
        mesh_to_unity = nh_.advertise<scene_understanding_pkg_msgs::MeshVertexPosition>("drone_mesh_to_unity", 1);
        mesh_pointcloud_to_unity = nh_.advertise<sensor_msgs::PointCloud2>("drone_mesh_to_unity_as_pointcloud", 1);
        holo_position_rviz = nh_.advertise<visualization_msgs::Marker>("hololens_frame", 10);
     

        //dragonfly_odom =  nh_.advertise<nav_msgs::Odometry>("dragonfly/odom", 50);
    }

    ~Kimera_vio_data()
    {
    }

    // Point cloud sent in the mesh topic is ordered folliwing the list of vertices.
    // To obtain the mesh from the data contained in the mesh topic is sufficient? to ordereing in triple
    // the list of vertices
    void kimera_vio_mesh_callback(const pcl_msgs::PolygonMesh &msg) // const pcl_msgs::PolygonMesh &msg)
    {

        // Obtain points position xyz from the point cloud
        sensor_msgs::PointCloud2 pointcloud_msg;
        pointcloud_msg = msg.cloud;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(pointcloud_msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *pt_cloud);

        for (int i = 0; i < pt_cloud->points.size(); ++i)
        {

            float x = pt_cloud->points[i].x;
            float y = pt_cloud->points[i].y;
            float z = pt_cloud->points[i].z;

            mesh_point_x.push_back(x);
            mesh_point_y.push_back(y);
            mesh_point_z.push_back(z);
        }
        // Convert pcl_msg PolygonMesh in pcl::PolygonMesh
        pcl::PolygonMesh pcl_mesh;
        pcl_conversions::toPCL(msg, pcl_mesh);

        // Reconvert to ROS format

        pcl_msgs::PolygonMesh pcl_msg_mesh;
        pcl_conversions::fromPCL(pcl_mesh, pcl_msg_mesh);
        sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

        size_t size = pcd_modifier.size();

        shape_msg_mesh.vertices.resize(size); // shape msg is a ros format to contain th emesh

        // std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";

        sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

        // obtain the position xyz of the points relative to the triplets described by vertices
        for (size_t i = 0; i < size; i++, ++pt_iter)
        {
            shape_msg_mesh.vertices[i].x = pt_iter[0];
            shape_msg_mesh.vertices[i].y = pt_iter[1];
            shape_msg_mesh.vertices[i].z = pt_iter[2];
        }

        shape_msg_mesh.triangles.resize(pcl_mesh.polygons.size());

        for (size_t i = 0; i < pcl_mesh.polygons.size(); ++i)
        {
            if (pcl_mesh.polygons[i].vertices.size() < 3)
            {
                ROS_WARN("Not enough points in polygon. Ignoring it.");
                continue;
            }

            // shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
            // boost::arra

            // triangle.vertex_indices = xyz;

            // mesh.triangles.push_back(shape_msgs::MeshTriangle());
            // mesh.triangles[i].vertex_indices.resize(3);

            for (int j = 0; j < 3; ++j)
                shape_msg_mesh.triangles[i].vertex_indices[j] = pcl_mesh.polygons[i].vertices[j]; // associate a ros mesh triqangles to each polygons
        }
        if (pcl_mesh.polygons.size() > 3)
        {
            mesh_vector_CF.push_back(shape_msg_mesh);
        }

        flagKimeraMesh = true;
    }

    void kimera_vio_pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
    {
        /*
         pcl::PCLPointCloud2 pcl_pc2;
         pcl_conversions::toPCL(*input,pcl_pc2);
         pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
         pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);



          for(int i = 0 ; i < pt_cloud->points.size(); ++i)
         {
             float x = pt_cloud->points[i].x;
             float y = pt_cloud->points[i].y;
             float z = pt_cloud->points[i].z;

             point_x.push_back(x);
             point_y.push_back(y);
             point_z.push_back(z);
        }point_y
         */
        /*


         /*
        sensor_msgs::PointCloud2 data = msg;
        cout<< data.points[0].x << endl;
        */
        flagKimeraPointCloud = true;
    }

    void realsense_pointcloud_callback(const sensor_msgs::PointCloud2 &input) //const boost::shared_ptr<const sensor_msgs::PointCloud2>
    {
        

        pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(input, *cloud_pass);

        // std::cerr << "PointCloud Before filtering: " << input->width * input->height << std::endl;
        // Filter out and discretizing point cloud
        pcl::PCLPointCloud2::Ptr cloud_filtered_pcl2(new pcl::PCLPointCloud2());
        
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud_pass);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*cloud_filtered_pcl2);
       

        // std::cerr << "PointCloud after filtering: " << cloud_filtered_pcl2->width * cloud_filtered_pcl2->height << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*input,pcl_pc2);
        //pcl::fromPCLPointCloud2(*cloud_filtered_pcl2, *pt_cloud);
       pcl::fromPCLPointCloud2(*cloud_filtered_pcl2, *pt_cloud);
       
        point_x.clear();
        point_y.clear();
        point_z.clear();
       
        for (int i = 0; i < pt_cloud->points.size(); ++i)
        {

            double x = pt_cloud->points[i].x;
            double y = pt_cloud->points[i].y;
            double z = pt_cloud->points[i].z;
             //Evaluate distance frm the drone position 
            
            double distance = sqrt(pow(x -voxl3_position.x, 2 ) + pow(y -voxl3_position.y, 2 ) + pow(z -voxl3_position.z, 2 ));
            if (distance > 2.0)
            {
                continue;
            }

            //Check if there are noisy points under the ground 
            if (z < - 0.2)
            {
                z = 0;
            }
            point_x.push_back(x);
            point_y.push_back(y);
            point_z.push_back(z); 
        }
       
        last_pointcloud_stamp = double(input.header.stamp.sec) +  double(input.header.stamp.nsec)*1e-9;
    
        //publish_transformed_pointcloud(output_pc_mocap_sr);
        flagKimeraRealsensePointCloud = true;

          //cout << scientific << setprecision(15)<< " PC CALLBACK: " << last_pointcloud_stamp <<  endl;
    }
    



     void voxl3_mocap_pose_callback(const nav_msgs::Odometry &msg)
  {
    
    
     voxl_pose_array.push_back(msg);
     
      //Position
      voxl3_position.x = msg.pose.pose.position.x;
      voxl3_position.y = msg.pose.pose.position.y;
      voxl3_position.z = msg.pose.pose.position.z;
    
     //orientation
     
    // Covert to euler angles
    voxl3_quat_orientation = msg.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    voxl3_euler_orientation.x = roll;
    voxl3_euler_orientation.y = pitch;
    voxl3_euler_orientation.z = yaw;
    voxl_euler_array.push_back(voxl3_euler_orientation);

    flagVoxl3Pose = true;

   
  }



    void image_proc_pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
    {
         pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*input, *cloud_pass);

        // std::cerr << "PointCloud Before filtering: " << input->width * input->height << std::endl;
        // Filter out and discretizing point cloud
        pcl::PCLPointCloud2::Ptr cloud_filtered_pcl2(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud_pass);
        sor.setLeafSize(0.001f, 0.001f, 0.001f);
        sor.filter(*cloud_filtered_pcl2);

        // std::cerr << "PointCloud after filtering: " << cloud_filtered_pcl2->width * cloud_filtered_pcl2->height << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*input,pcl_pc2);
       // pcl::fromPCLPointCloud2(*cloud_filtered_pcl2, *pt_cloud);
       pcl::fromPCLPointCloud2(*cloud_pass, *pt_cloud);
        /*

        tf::StampedTransform transform;
        tf::TransformListener listener;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);


        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

        //Transform the stored pc from msg to temp_cloud
        //Transform the point cloud from temp_cloud in base_link to world
         try{
            listener.lookupTransform("/world", "/left_cam", ros::Time()-ros::Duration(14.0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);


        //Publish the new point cloud
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_transformed,cloud_publish);
        cloud_publish.header = input->header;


        rs_pointcloud_w.publish(cloud_publish);
       */
        // call the transformed point cloud

        // cout << "Size Pointcloud: " <<  pt_cloud->points.size() << endl;
        geometry_msgs::Point p; 
        for (int i = 0; i < pt_cloud->points.size(); ++i)
        {
           
            p.x = pt_cloud->points[i].x;
            p.y = pt_cloud->points[i].y;
            p.z = pt_cloud->points[i].z;

            //image_proc_pointcloud.push_back(p);
        }
        

        
       
        flagKimeraImageProcPointCloud = true;

    }

    void kimera_vio_odometry_callback(const nav_msgs::Odometry &msg)
    {

        
        camera_odometry_position.x = msg.pose.pose.position.x;
        camera_odometry_position.y = msg.pose.pose.position.y;
        camera_odometry_position.z = msg.pose.pose.position.z;
        
        

        camera_quat_orientation = msg.pose.pose.orientation;
        // Covert to euler angles
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // the found angles are written in a geometry_msgs::Vector3

        camera_odometry_orientation.x = roll;
        camera_odometry_orientation.y = pitch;
        camera_odometry_orientation.z = yaw;
        
        flagKimeraOdometry = true;
    }

    void voxblox_mesh_block_callback(const voxblox_msgs::Mesh::ConstPtr &msg)
    {
        geometry_msgs::Point p;
        geometry_msgs::Point p_new;
       
        for (const voxblox_msgs::MeshBlock &mesh_block : msg->mesh_blocks)
        {
            const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);

            size_t vertex_index = 0u;
            voxblox::Mesh mesh;
            mesh.vertices.reserve(mesh_block.x.size());
            mesh.indices.reserve(mesh_block.x.size());

            // translate vertex data from message to voxblox mesh
           
            for (size_t i = 0; i < mesh_block.x.size(); ++i)
            {
                // Each vertex is given as its distance from the blocks origin in units of
                // (2*block_size), see mesh_vis.h for the slightly convoluted
                // justification of the 2.
                constexpr float point_conv_factor =
                    2.0f / std::numeric_limits<uint16_t>::max();
                const float mesh_x =
                    (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
                     static_cast<float>(index[0])) *
                    msg->block_edge_length;
                const float mesh_y =
                    (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
                     static_cast<float>(index[1])) *
                    msg->block_edge_length;
                const float mesh_z =
                    (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
                     static_cast<float>(index[2])) *
                    msg->block_edge_length;

                p_new.x = mesh_x;
                p_new.y = mesh_y;
                p_new.z = mesh_z; 
                
                // p.x = -1*mesh_x;
                // p.y = -1*mesh_y;
                // p.z = -1*mesh_z;

                // //ROtate all the points of 180° around the zed axis
                // p_new.x= p.x * cos(M_PI/2) - p.y*sin(M_PI/2);
                // p_new.y= p.x * sin(M_PI/2) + p.y*cos(M_PI/2);
                // // cout <<" x: " <<  connected_mesh.vertices[i].x() << " y: " << connected_mesh.vertices[i].y() << " z: " << connected_mesh.vertices[i].z() << endl;
                // p_new.z = p.z + 1.3;
                if (feature_matching_flag == false)
                {
                   voxblox_mesh_vertices.push_back(p_new);
                }
                
                
                mesh.indices.push_back(vertex_index++);
                mesh.vertices.emplace_back(p_new.x, p_new.y, p_new.z); //mesh_x, mesh_y, mesh_z
            }
            
            //
            // calculate normals
            // it is represented as a point. to have the direction of the vector the two points tol consider
            // are dir0 and normal
            mesh.normals.reserve(mesh.vertices.size());
            geometry_msgs::Point p_normal;
            for (size_t i = 0; i < mesh.vertices.size(); i += 3)
            {
                const voxblox::Point dir0 = mesh.vertices[i] - mesh.vertices[i + 1];
                const voxblox::Point dir1 = mesh.vertices[i] - mesh.vertices[i + 2];
                const voxblox::Point normal = dir0.cross(dir1).normalized();

                mesh.normals.push_back(normal);
                mesh.normals.push_back(normal);
                mesh.normals.push_back(normal);

                // Evaluate the triangle baricenter
                float bar_x = (mesh.vertices[i].x() + mesh.vertices[i + 1].x() + mesh.vertices[i + 2].x()) / 3;
                float bar_y = (mesh.vertices[i].y() + mesh.vertices[i + 1].y() + mesh.vertices[i + 2].y()) / 3;
                float bar_z = (mesh.vertices[i].z() + mesh.vertices[i + 1].z() + mesh.vertices[i + 2].z()) / 3;

                /*
                  //Add Starting point to the vector
                  p_normal.x = bar_x;
                  p_normal.y = bar_y;
                  p_normal.z = bar_y;
                  triangles_normals.push_back(p_normal);

                  //Add Second Points
                  p_normal.x = bar_x + normal(0);
                  p_normal.y = bar_y + normal(1);
                  p_normal.z = bar_z + normal(2);
                  triangles_normals.push_back(p_normal);


                  cout << "Normal: " << p_normal << endl;
                */
            }

            /*
           // add color information
           mesh.colors.reserve(mesh.vertices.size());
           const bool has_color = mesh_block.x.size() == mesh_block.r.size();
           for (size_t i = 0; i < mesh_block.x.size(); ++i)
           {
             voxblox::Color color;
             if (has_color) {
               color.r = mesh_block.r[i];
               color.g = mesh_block.g[i];
               color.b = mesh_block.b[i];

             } else {
               // reconstruct normals coloring
               color.r = std::numeric_limits<uint8_t>::max() *
                         (mesh.normals[i].x() * 0.5f + 0.5f);
               color.g = std::numeric_limits<uint8_t>::max() *
                         (mesh.normals[i].y() * 0.5f + 0.5f);
               color.b = std::numeric_limits<uint8_t>::max() *
                         (mesh.normals[i].z() * 0.5f + 0.5f);
             }
             color.a = std::numeric_limits<uint8_t>::max();
             mesh.colors.push_back(color);
           }
           */
           /*
            
             // connect mesh
            voxblox::Mesh connected_mesh;
            voxblox::createConnectedMesh(mesh, &connected_mesh, 0.01);
             cout << "connected_mesh.vertices SIZE: " << connected_mesh.vertices.size() << endl;
            geometry_msgs::Point p;

            for (size_t i = 0; i < connected_mesh.vertices.size(); ++i)
            {
                p.x = connected_mesh.vertices[i].x();
                p.y = connected_mesh.vertices[i].y();
                p.z = connected_mesh.vertices[i].z();
                //cout <<" x: " <<  connected_mesh.vertices[i].x() << " y: " << connected_mesh.vertices[i].y() << " z: " << connected_mesh.vertices[i].z() << end
                voxblox_mesh_vertices.push_back(p);
            }
             cout << "Vvoxblox_mesh_vertices SIZE: " << voxblox_mesh_vertices.size() << endl;  

             */
        }
        /*
                cout <<"Mesh Block: " <<  msg.mesh_blocks.size() << endl;
                for (int i = 0; i<msg.mesh_blocks.size(); ++i)
                {

                    //take the index in each block
                    int index1 = msg.mesh_blocks[i].index[0];
                    int index2 = msg.mesh_blocks[i].index[1];
                    int index3 = msg.mesh_blocks[i].index[2];
                    for (int j = 0; j<msg.mesh_blocks[i].x.size(); ++j)
                    {
                        float point_conv_factor =  2.0f / std::numeric_limits<uint16_t>::max();
                        p.x = msg.mesh_blocks[i].x[j] * point_conv_factor + index1 * msg.block_edge_length;
                        p.y = msg.mesh_blocks[i].y[j]  * point_conv_factor + index2 * msg.block_edge_length;
                        p.z = msg.mesh_blocks[i].z[j] * point_conv_factor + index3 * msg.block_edge_length;
                        voxblox_mesh_vertices.push_back(p);



                        //cout << "x: " << p.x << "y: " << p.y << "z: " << p.z << endl;

                    }
                }
               */
    }

    void realsense_color_image_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        new_color_frame = cv_ptr->image;

        flagRealSenseNewColorFrame = true;
    }

  //Dragonfly callback
  void dragonfly_imu_callback(const sensor_msgs::Imu &msg)
  {
      dragonfly_imu_angular_vel.x = msg.angular_velocity.x;
      dragonfly_imu_angular_vel.y = msg.angular_velocity.y;
      dragonfly_imu_angular_vel.z = msg.angular_velocity.z;

      dragonfly_imu_lin_acc.x = msg.linear_acceleration.x;
      dragonfly_imu_lin_acc.y = msg.linear_acceleration.y;
     dragonfly_imu_lin_acc.z = msg.linear_acceleration.z;

      flagDragonflyImu = true;
     
  }

  void dragonfly_vio_pose_callback(const nav_msgs::Odometry &msg)
  {
      //Position
      dragonfly_position.x = msg.pose.pose.position.x;
      dragonfly_position.y = -1.0*msg.pose.pose.position.y;
      dragonfly_position.z = -1.0*msg.pose.pose.position.z;
    
     //orientation
     
    // Covert to euler angles
    dragonfly_quat_orientation = msg.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    dragonfly_euler_orientation.x = roll;
    dragonfly_euler_orientation.y = pitch;
    dragonfly_euler_orientation.z = yaw;

    flagDragonflyPose = true;


  }



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
     


    }


    

    void holo_position_callback(const geometry_msgs::Pose &msg)
    {

        hololens_position = msg.position;
        hololens_orientation = msg.orientation;

        
        flagHoloPosition = true;
    }



// void voxl3_vio_pose_callback(const nav_msgs::Odometry &msg)
//   {
//      voxl_pose_array.push_back(msg);
     
//       //Position
//       voxl3_position.x = msg.pose.pose.position.x;
//       voxl3_position.y = msg.pose.pose.position.y;
//       voxl3_position.z = msg.pose.pose.position.z;
    
//      //orientation
     
//     // Covert to euler angles
//     voxl3_quat_orientation = msg.pose.pose.orientation;
//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
//     // the tf::Quaternion has a method to acess roll pitch and yaw
//     double roll, pitch, yaw;
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
//     voxl3_euler_orientation.x = roll;
//     voxl3_euler_orientation.y = pitch;
//     voxl3_euler_orientation.z = yaw;
//     voxl_euler_array.push_back(voxl3_euler_orientation);

//     flagVoxl3VioPose = true;


//   }
  

  
// void quadrotor_vio_pose_callback(const nav_msgs::Odometry &msg)
//   {
//      voxl_pose_array.push_back(msg);
     
//       //Position
//       voxl3_position.x = msg.pose.pose.position.x;
//       voxl3_position.y = msg.pose.pose.position.y;
//       voxl3_position.z = msg.pose.pose.position.z;
    
//      //orientation
     
//     // Covert to euler angles
//     voxl3_quat_orientation = msg.pose.pose.orientation;
//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
//     // the tf::Quaternion has a method to acess roll pitch and yaw
//     double roll, pitch, yaw;
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
//     voxl3_euler_orientation.x = roll;
//     voxl3_euler_orientation.y = pitch;
//     voxl3_euler_orientation.z = yaw;
//     voxl_euler_array.push_back(voxl3_euler_orientation);

//     flagQuadrotorSimPose = true;


//   }

 


    void from_unity_start_stop_mapping_callback(const std_msgs::Bool &msg)
    {
        start_mapping = msg.data;
        flagboolStartMapping = true;
        cout << "MAPPING: " <<    start_mapping << endl;
    }


    void from_unity_restart_mapping_callback(const std_msgs::Bool &msg)
    {
        restart_mapping_flag = msg.data;
        flagboolRestartMapping = true;   
    }


    // From Python Feature detector
    void python_feature_detector_matching_callback(const std_msgs::Bool &msg)
    {
        feature_matching_flag = msg.data;
        flagboolFeatureMatching = true;
    }

    void publish_mesh_vertices_to_rviz(visualization_msgs::Marker triangles)
    {
        marker_pub_vertices_rviz.publish(triangles);
    }

    void publish_old_mesh_vertices_to_rviz(visualization_msgs::Marker triangles_old)
    {
        marker_pub_old_vertices_rviz.publish(triangles_old);
    }

    void publish_arrows_vertices_to_rviz(visualization_msgs::Marker arrows)
    {
        marker_pub_arrows_vertices_rviz.publish(arrows);
    }

    void publish_rs_pointcloud_world(vector<geometry_msgs::Point> triangles)
    {
        // iterate on the pc vector and publish each pc independently
        // for (int ii = 0; ii < point_cloud_realsense_W.size(); ii++)
        //{
        // pcl::PointCloud<pcl::PointXYZ> PointCloud;
        PointCloud::Ptr msg(new PointCloud);
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "mocap_sr"; //world
        msg->height = 1;
        msg->width = triangles.size();  //point_cloud_realsense_W
        msg->points.clear();
      
        for (int j = 0; j < msg->width; j++)
        {
            geometry_msgs::Point p;

            p.x = triangles[j].x;
            p.y = triangles[j].y;
            p.z = triangles[j].z;
            //Check teh distance between the points and the drone and discard points too far away 
            float threshold_distance = 3;
            float distance = sqrt(pow(voxl3_position.x - p.x, 2) +pow(voxl3_position.y - p.y, 2) + pow(voxl3_position.z - p.z, 2) );
            if (distance > threshold_distance)
            { 
               
                continue;
            }
            p.z = p.z - 0.15;
            //cout << "point: " << p << endl;
            if (pc_from_image_proc == true)
            {
                msg->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
            }
            else
            {
               
                msg->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
              
            }

           
        }
        
         cout << "points size: " << msg->points.size() << endl;
        if (msg->points.size() > 100)
        {
            msg->width = msg->points.size();
            rs_pointcloud_w.publish(msg);
        }
       
        triangles.clear();
        //}
    }

    

    //Publish Mesh Vertices to Unity 
    void publish_mesh_to_unity(visualization_msgs::Marker triangles)
    {
        //Iterate on the triangles and republish them as geometry messsage array
        
        scene_understanding_pkg_msgs::MeshVertexPosition msg_point;
        /*
        vector<float32_t> x_verteces;
        vector<float32_t> y_verteces;
        vector<float32_t> z_verteces;
        */
        geometry_msgs::Point p;
        
        cout << "Points Published to Unity: " << triangles.points.size() << endl;
        for (int ii = 0; ii < triangles.points.size(); ii++)
        {
            p = triangles.points[ii];
             msg_point.point.push_back(p);
             /*
            x_verteces.push_back(triangles.points[ii].x);
            y_verteces.push_back(triangles.points[ii].y);
            z_verteces.push_back(triangles.points[ii].z);
            */
        }
        
        if (msg_point.point.size() > 0)
        {
             mesh_to_unity.publish(msg_point);
        }
       


    }

    void publish_mesh_to_unity_as_pointcloud(visualization_msgs::Marker triangles)
    {
        PointCloud::Ptr msg(new PointCloud);
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "world"; //base_link
        msg->height = 1;
        msg->width = triangles.points.size();  //point_cloud_realsense_W
        msg->points.clear();

        for (int j = 0; j < msg->width; j++)
        {
            geometry_msgs::Point p;

            p = triangles.points[j];
        
            //cout << "point: " << p << endl;
          
            msg->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));

        }

        mesh_pointcloud_to_unity.publish(msg);

    }





    //Publish Dragonfly Odom
    void publish_dragonfly_tf()
    {
        //TF Publisher valuse 
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
    
    
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(-1*dragonfly_euler_orientation.z,dragonfly_euler_orientation.y,dragonfly_euler_orientation.x);
    
    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);

    geometry_msgs::Quaternion quat;
    quat.x =  q_tf.getX();
    quat.y = q_tf.getY();
    quat.z = q_tf.getZ();
    quat.w = q_tf.getW();
    
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = dragonfly_position.x;
    odom_trans.transform.translation.y = dragonfly_position.y;
    odom_trans.transform.translation.z = dragonfly_position.z;
    odom_trans.transform.rotation = quat;

    //send the transform
    //dragonfly_odom_broadcaster.sendTransform(odom_trans);
    


    }



        //Publish Dragonfly Odom
    void publish_voxl3_tf()
    {
        //TF Publisher valuse 
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
    


   
    
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap_sr";
    odom_trans.child_frame_id = "voxl3";

    odom_trans.transform.translation.x = voxl3_position.x;
    odom_trans.transform.translation.y = voxl3_position.y;
    odom_trans.transform.translation.z = voxl3_position.z;
    odom_trans.transform.rotation = voxl3_quat_orientation;

    //send the transform
    voxl3_odom_broadcaster.sendTransform(odom_trans);
    


    }


    void publish_mocap_sim_tf()
    {
             //TF Publisher valuse 
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap";
    odom_trans.child_frame_id = "simulator";

    odom_trans.transform.rotation.w = 1.0;

    //send the transform
    mocap_to_sim_broadcaster.sendTransform(odom_trans);
    


    }


         //Publish Dragonfly Odom
    void publish_realsense_tf()
    {
        //TF Publisher valuse 
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
    

     

   
    
    // INS_msg.orientation.x = q_tf.getX();
    // INS_msg.orientation.y = q_tf.getY();
    // INS_msg.orientation.z = q_tf.getZ();
    // INS_msg.orientation.w = q_tf.getW();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = camera_odometry_position.x;
    odom_trans.transform.translation.y = camera_odometry_position.y;
    odom_trans.transform.translation.z = camera_odometry_position.z;
    odom_trans.transform.rotation = camera_quat_orientation;

    //send the transform
    voxl3_odom_broadcaster.sendTransform(odom_trans);

    }



  void publish_holo_pose_to_RVIZ(visualization_msgs::Marker arrow_x)
    {
        holo_position_rviz.publish(arrow_x);
    }




    void publish_from_mocap_to_world_tf()
{
   ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap"; //simulator
    odom_trans.child_frame_id = "world";

    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion quat; 
    quat.w = 1.0;

    odom_trans.transform.rotation = quat;
     //send the transform
    from_mocap_to_world.sendTransform(odom_trans);

}



//Mocap related to the fram Mocap_sr in Unity (Only when Hololens is used)
void publish_unity_mocap_to_mocap_sr_tf()
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
    odom_trans.child_frame_id = "mocap_sr";
    

   
    tf::Quaternion quat;
    tf::quaternionMsgToTF(mocap_sr_orientation_unity_world, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    geometry_msgs::Vector3 voxl3_euler_orientation;
    voxl3_euler_orientation.x = roll;
    voxl3_euler_orientation.y = pitch;
    voxl3_euler_orientation.z = yaw;


 
    
     quat_orientation = mocap_sr_orientation_unity_world; //tf2::toMsg(q_new);

 
    odom_trans.transform.translation.x = mocap_sr_unity_world_position.x;
    odom_trans.transform.translation.y = mocap_sr_unity_world_position.y;
    odom_trans.transform.translation.z = mocap_sr_unity_world_position.z;
    odom_trans.transform.rotation = quat_orientation;


    unity_mocap_to_mocap_sr_broadcaster.sendTransform(odom_trans);
}


void publish_mocap_to_mocap_sr_tf()
{
    
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();

    geometry_msgs::Quaternion quat_orientation;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap";
    odom_trans.child_frame_id = "mocap_sr";
    

   
    tf::Quaternion quat;
 

    odom_trans.transform.rotation.w = 1.0;


    mocap_to_mocap_sr_broadcaster.sendTransform(odom_trans);
     
}

  

void publish_hololens_pose_in_RVIZ()
{
  visualization_msgs::Marker sphere;
    sphere.points.clear();

     sphere.header.frame_id = "/world";
    sphere.header.stamp  = ros::Time::now();
    //Markers Namespace 
     sphere.ns = "holo_position_in_unity";
    // Assign a type of action to each marker
     sphere.action =  visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important

     sphere.pose.position =  hololens_position;
     sphere.pose.orientation = hololens_orientation;
     sphere.pose.orientation.w = 1.0;

    // Marker Type (Arrow, Sphere, points )
     sphere.type = visualization_msgs::Marker::SPHERE;   
     
     sphere.id = 0;
  
    // POINTS markers use x and y scale for width/height respectively
     sphere.scale.x = 0.3;
     sphere.scale.y = 0.3;
     sphere.scale.z = 0.3;
    
     sphere.color.b = 1.0f;
     sphere.color.a = 1.0;

    publish_holo_pose_to_RVIZ(sphere);

}


    //Call Clear Map Service 
    bool call_clear_map_service()
    {
        bool flag = false;
        std_srvs::Empty srv;
         clear_map_service.waitForExistence();
        

        if (clear_map_service.call(srv))
        {
        ROS_ERROR("Successfully called service clear_map");
        flag = true;
        }
        else
        {
        ROS_ERROR("Failed to call service clear_map");
        flag = false;
        }
      
       
        return flag;

    }
    


};

#endif
 












