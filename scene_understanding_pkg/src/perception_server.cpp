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


#include "perception_server.h"

#include "SU_Unity_comm.h"
#include "Mesh_constructor.h"
#include <shape_msgs/Mesh.h>
#include <typeinfo>

//Voxblox Libraries
#include <voxblox/core/esdf_map.h>


namespace perception {

    PerceptionServer::PerceptionServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      in_simulation(true),
      horizon_force_perception(1.5),
      lamda(1.0),
      Fs(5.0),
      avg_stamp(10),
      mesh_publish_period(3),
      perception_utils(nh, nh_private)
      {//in sec
      //transformer_(nh, nh_private) { // transformer e una classe che va inizializzata e definita (se necessario)
    //Publisher
    triangles_mesh = nh_.advertise<visualization_msgs::Marker>("/surface_mesh", 1);
    marker_pub_force_arrow_rviz = nh_.advertise<visualization_msgs::Marker>("f_obs_vector", 10);
    mesh_to_unity = nh_.advertise<scene_understanding_pkg_msgs::MeshVertexPosition>("drone_mesh_to_unity", 1);
    cloud_in_rep = nh_.advertise<sensor_msgs::PointCloud2>("cloud_in_rep_", 1);
    pub_occupied_pc_to_unity = nh_.advertise<sensor_msgs::PointCloud2>("/voxblox_node/occupied_pc_to_unity", 1);
    mesh_pointcloud_to_unity = nh_.advertise<sensor_msgs::PointCloud2>("drone_mesh_to_unity_as_pointcloud", 1); //verificare se utile
    holo_position_rviz = nh_.advertise<visualization_msgs::Marker>("hololens_frame", 1);
    obst_force = nh_.advertise<scene_understanding_pkg_msgs::ObstacleRepForce>("obstacles_force_field", 1);
  
    //Subscribers
    cloud_in = nh_.subscribe("/cloud_in", 5, &PerceptionServer::cloud_in_callback, this);
    quadrotor_pose = nh_.subscribe("/odom", 10, &PerceptionServer::quadrotor_pose_callback, this); //define the topic from the remapping
    start_stop_mapping = nh_.subscribe("/from_Unity/start_stop_mapping", 5, &PerceptionServer::from_unity_start_stop_mapping_callback, this);
    restart_mapping = nh_.subscribe("/from_Unity/restart_mapping", 5, &PerceptionServer::from_unity_reset_map_callback, this);
    voxblox_mesh = nh_.subscribe("/voxblox_node/mesh", 5, &PerceptionServer::voxblox_mesh_callback, this);
    voxblox_pc_surface = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &PerceptionServer::voxblox_surface_cloud_callback, this);
    mocap_scene_root_in_unity_world = nh_.subscribe("/from_unity/mocap_frame_in_unity_world_coo", 5, &PerceptionServer::mocap_scene_root_in_unity_frame_callback, this);
    holoPose = nh_.subscribe("/hololens_position", 5, &PerceptionServer::holo_position_callback, this);
    virtual_obstacles = nh_.subscribe("/rrt/virtual_obstacles_coo", 5, &PerceptionServer::virtual_obs_callback, this);
    enable_assistive_mode = nh_.subscribe("/rrt/start_assistive_guidance", 1, &PerceptionServer::planner_start_assistive_guidance_callback, this);
    //add the marker for the visualization of the virtual obstacles in rviz keep it out from drone teleop

     // Services
    clear_map_service = nh_.serviceClient<std_srvs::Empty>("/voxblox_node/clear_map");
    
    //Parameters 
    nh_private_.param("/tele_control_params/scenario", in_simulation, in_simulation);
    nh_private_.param("/perception_params/horizon", horizon_force_perception, horizon_force_perception);
    nh_private_.param("/perception_params/lamda", lamda, lamda);
    nh_private_.param("/perception_params/Fs", Fs, Fs);
    nh_private_.param("/perception_params/avg_stamps", avg_stamp, avg_stamp);
    nh_private_.param("/perception_params/mesh_publish_period", mesh_publish_period, mesh_publish_period);
   

    //Initialize visualization messages
    initialize_visualization_messages();

    if (mesh_publish_period > 0.0) {
    publish_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(mesh_publish_period*1000),
                                &PerceptionServer::publishMesh, this);

    publish_occupied_pc_to_unity = 
        nh_private_.createTimer(ros::Duration(mesh_publish_period),
                                &PerceptionServer::publishOccupiedPC_to_unity, this);
  }

  perception_utils.avg_stamp = avg_stamp;
  bool call_restart_mapping_service = false;
 
  }


void PerceptionServer::initialize_visualization_messages()
{
  triangles.points.clear();
  obs_magn_force_b_frame.points.clear();

  triangles.header.frame_id = mesh_vis_msgs_frame;//"/mocap_sr";
  obs_magn_force_b_frame.header.frame_id = force_vis_msgs_frame;
  triangles.header.stamp = ros::Time::now();
  obs_magn_force_b_frame.header.stamp = ros::Time::now();

  // Markers Namespace
  triangles.ns = "triangle_list";
  obs_magn_force_b_frame.ns = "force_arrow";
  // Assign a type of action to each marker
  triangles.action = visualization_msgs::Marker::ADD;
  obs_magn_force_b_frame.action = visualization_msgs::Marker::ADD;

  // Pose -- Since are points the orientation is not important
  triangles.pose.orientation.w = 1.0;
  obs_magn_force_b_frame.pose.orientation.w = 1.0;

  // Marker Type (Arrow, Sphere, points )
  triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  obs_magn_force_b_frame.type = visualization_msgs::Marker::ARROW;

  triangles.id = 0;
  obs_magn_force_b_frame.id = 0;

  // POINTS markers use x and y scale for width/height respectively
  triangles.scale.x = 1;
  triangles.scale.y = 1;
  triangles.scale.z = 1;

  obs_magn_force_b_frame.scale.x = 1;
  obs_magn_force_b_frame.scale.y = 0.05;
  obs_magn_force_b_frame.scale.z = 0.05;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

  // Points are green
  triangles.color.g = 1.0f;
  triangles.color.a = 0.3;
  
  obs_magn_force_b_frame.color.g = 1.0f;
  obs_magn_force_b_frame.color.a = 1.0;

}

void PerceptionServer::publishMesh(const ros::TimerEvent& /*event*/) {
    if (triangles.points.size() > 0)
    {
      this->triangles_mesh.publish(triangles);
      publish_mesh_to_unity(triangles);
      
      cout << "-----------------------------Data Published-------------------------------------------" << endl;
    }
    
    triangles.points.clear();      
  //Clear the voxblox Vector otherwise the same triamngles are added to the Visualization 
  //Marker Message 
    voxblox_mesh_vertices.clear();

}


void PerceptionServer::publishOccupiedPC_to_unity(const ros::TimerEvent& /*event*/) {

  bool point_existing = false;  
  pc_to_unity_new.points.clear();
  cout << "pc_surface_ size: " << pc_surface_.points.size() << endl;
  cout << "pc_to_unity size: " << pc_to_unity.points.size() << endl;
//Every time a pointcloud is received its required to filtering it publishing only the added points before republishing to unity 
  if (pc_surface_.points.size() > pc_to_unity.points.size())
  {
   //iterate over the pointcloud and ublishing to unity only the different points 
    pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
    pcl::PointCloud<pcl::PointXYZ>::const_iterator item2;

   //Iterate on the new pointcloud 
    for (item = pc_surface_.begin(); item != pc_surface_.end(); item++) 
    {
      point_existing = false;
      //iterate on the old pointcloud 
      for (item2 = pc_to_unity.begin(); item2 != pc_to_unity.end(); item2++) 
      {
      float dx = item->x - item2->x; 
      float dy = item->y - item2->y; 
      float dz = item->z - item2->z; 
      float distance_3D = sqrt(pow((double)dx, 2) + pow((double)dy, 2) + pow((double)dz, 2) );
       
       if (distance_3D == 0){
        point_existing = true;
        break;
        }
       }
     
      //check if the program arrives here with point_existing = false so i can add the point to the pointcloud to publish 
      if (point_existing == false)
      {
          //add the point to the new pointcloud

          pc_to_unity_new.points.push_back(pcl::PointXYZ(item->x, item->y, item->z));
      }
  
     }
   pc_to_unity.points.clear();
   pc_to_unity = pc_surface_;
  }
 
  //create the pointcloud to publish
  pcl::PCLPointCloud2 pointCloud;
  sensor_msgs::PointCloud2 pointCloud_out;

  std_msgs::Header header_msg;
  header_msg.frame_id = frame_pc;
  header_msg.stamp = ros::Time::now();

  /* prepare pointCloud msg to publish */
  pointCloud_out.header = header_msg;

  pcl::toPCLPointCloud2(pc_to_unity_new, pointCloud);
  pcl_conversions::moveFromPCL(pointCloud, pointCloud_out);
  pointCloud_out.header = header_msg;
   
  cout << "###########################################################" << endl;
  cout << "pc_to_unity_new size: " << pc_to_unity_new.points.size() << endl;
  pub_occupied_pc_to_unity.publish(pointCloud_out);


}




void PerceptionServer::publish_hololens_pose_in_RVIZ()
{
    visualization_msgs::Marker sphere;
    sphere.points.clear();

    sphere.header.frame_id = "mocap";
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

    holo_position_rviz.publish(sphere);

}

void PerceptionServer::evaluate_obstacle_force_field()
{
  //The pointcloud received from Voxblox is expressed on the frame Map which is aliigne with the frame world.
  //Take the robot position and the surface pointcloud position 
  float horizon = horizon_force_perception; 
  float theta = 0.0;

  float dx_sum = 0.0;
  float dy_sum = 0.0;
  float sat_x_value = Fs;
  float sat_y_value = Fs;
  vector<float> theta_vec;


  //Looking for voxels insde the robot horizon 
  pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
  /*
  if (pc_surface_.size() > 0)
  {
    distance_vec.clear();
    F_magn = 0.0;
  }
  */

 distance_vec.clear();
 F_magn = 0.0;
  for (item = pc_surface_.begin(); item != pc_surface_.end(); item++) 
  {
   
    float dx = item->x - (mocap_sr_unity_world_position.x + quad_position.x); 
    float dy = item->y - (mocap_sr_unity_world_position.y + quad_position.y); 
    float dz = item->z - quad_position.z; 
    float distance_2D = sqrt(pow((double)dx, 2) + pow((double)dy, 2));
     
    //cout << "distance_2D: " <<distance_2D << endl;
    
    if (distance_2D < horizon && abs(dz) < 0.1) 
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
      theta = perception_utils.align_theta_with_yaw(theta, dx, dy);
     
     // //Add all the d

      float k = 1 - exp(horizon);
      F_magn = (Fs/k)*exp(-lamda*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      
      //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
      float F_magn_squared = pow((double)F_magn, 2);
      float dy_ = F_magn * sin(theta); 
      float dy_squared = pow((double)dy_, 2);
      float dx_ = sqrt(F_magn_squared - dy_squared);
      
      

      //Change properly the sign of the x component depending 
      if (dx > 0)
      {
        dx_ = -1*dx_;
      }

      dx_sum += dx_;
      dy_sum += dy_;

     
      
      distance_vec.push_back(distance_2D);
      theta_vec.push_back(theta);

    }
  }

  //add to the force also the force generated by the virtual obstacles if presents 
    int counter_obs = 0;
    for (int ii = 0; ii <  virtual_obstacles_vec.size()/2; ii++)
    {
        float min_x = virtual_obstacles_vec[counter_obs].x;
        float min_y = virtual_obstacles_vec[counter_obs].y;

        float max_x = virtual_obstacles_vec[counter_obs + 1].x;
        float max_y = virtual_obstacles_vec[counter_obs + 1].y;

        float x_center = (max_x - min_x)/2;
        x_center = max_x -x_center;

        float y_center = (max_y - min_y)/2;
        y_center = max_y -y_center;


       float dx = x_center - quad_position.x; 
       float dy = y_center - quad_position.x; 

      
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
    //   F_magn = 0.3*(Fs/k)*exp(-lamda*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      
    //   //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
    //   float F_magn_squared = pow((double)F_magn, 2);
    //   float dy_ = F_magn * sin(theta); 
    //   float dy_squared = pow((double)dy_, 2);
    //   float dx_ = sqrt(F_magn_squared - dy_squared);
      
    //   //Change properly the sign of the x component depending 
    //   if (dx > 0)
    //   {
    //     dx_ = -1*dx_;
    //   }
      
    //   dx_sum += dx_;
    //   dy_sum += dy_;

    //   distance_vec.push_back(distance_2D);
    //   theta_vec.push_back(theta);
    
    // }
    counter_obs = counter_obs + 2; 
   
    }
 virtual_obstacles_vec.clear();
    //  //Normalize the resultant vector
  float ip = sqrt(pow((double)dx_sum, 2) + pow((double)dy_sum, 2));
  float dx_sum_norm = dx_sum/ip;
  float dy_sum_norm = dy_sum/ip;
  float norm_ip = sqrt(pow(dx_sum_norm,2) + pow(dy_sum_norm,2));

  //Evaluate theta of the current sum vector 
  theta_res = acos(dx_sum_norm/norm_ip);
 
  if (dy_sum_norm <= 0)
  {
    theta_res = -theta_res;
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

  if (distance_vec.size() > 0 || F_magn > 0.01)
   {

    //average the value of theta_res to avoid big oscillation 
    //theta_res = perception_utils.average_theta_val(theta_res, &theta_vec);
   
      // //Compute average of the distances 
    float sum = accumulate(distance_vec.begin(), distance_vec.end(), 0);
    float d_vec_avg = sum/distance_vec.size();

   
    min_dist = *min_element(distance_vec.begin(), distance_vec.end());
    // //Find the index related to the minimum distance element 
    index_min_distance = perception_utils.getIndex(&distance_vec, min_dist);

     //Copute and check the derivative of theta to avoid big oscillation where in wall cavity or near corners 
    theta_dot = perception_utils.compute_theta_dot(theta_res, theta_min_distance_old, dt);

    
    //The new Magnitude of the vector is given by the exponetial decay relationship evaluated to the element in the surface with the ckoser distance 
    float k = 1 - exp(horizon);
    F_magn = (Fs/k)*exp(-lamda*min_dist)*(1-exp(horizon - min_dist)); 

    dx_new = F_magn * cos(theta_res);
    dy_new = F_magn * sin(theta_res);
    theta_min_distance_ = theta_res;
    
    
    if(APVI)
    {
      dx_new = 0.2*dx_new;
      dy_new = 0.2*dy_new;
      F_magn = sqrt(pow(dx_new,2) + pow(dy_new, 2));
    }
    //FIlter the components of the force to avoid big change in the direction of the force 
    perception_utils.average_marker_postions_on_multiple_stamps(&dx_fil, &dy_fil, dx_new, dy_new);

    // cout << "theta_min_distance: " << theta_res << endl;
    // cout << "dx_new: " << dx_fil << endl;
    // cout << "dy_new: " << dy_fil << endl;

    theta_min_distance_old = theta_res;

  }
  else
  {
    F_magn = 0.0;
    distance_vec.clear();

  }

  float dx_sum_sat = 0.0;
  float dy_sum_sat = 0.0;
  perception_utils.saturate_force_value(&dx_sum_sat, &dy_sum_sat, dx_fil,dy_fil, sat_x_value, sat_y_value );
  
  //Publosh the arrow representing the magnitude of the onstacle ofrce field centered on the robot frame 
  publish_obs_force_array_marker(theta_res);
 
  //This is fine, try with the sqaured of the cubic of the distance. 
  //If still to high, normalie and put a gain or saturate the Force up to a certain value on dx and dy 
  cout << "F_magn: " << F_magn << endl;
  cout << "dx_sum: " << dx_sum_sat << " dy_sum: " << dy_sum_sat <<  endl;
  publish_obstacle_force_resultant(dx_sum_sat, dy_sum_sat, F_magn);
  
//   kF_mag = kimera_data->F_magn;
  
}

/*
*  Services
*/



/* 
* Callbacks 
*/
void PerceptionServer::voxblox_surface_cloud_callback(const sensor_msgs::PointCloud2 &input){
     pc_surface_.points.clear();
     pcl::PCLPointCloud2 pc_surface;//(new pcl::PCLPointCloud2);
     pcl_conversions::toPCL(input, pc_surface);      
     pcl::fromPCLPointCloud2(pc_surface, pc_surface_);
     frame_pc = input.header.frame_id;
    
}

void PerceptionServer::cloud_in_callback(const sensor_msgs::PointCloud2 &input) //const boost::shared_ptr<const sensor_msgs::PointCloud2>
{
    start_mapping = true;
    if (start_mapping == true)
    {
      sensor_msgs::PointCloud2 pointCloud_out;
      pointCloud_out = input;
      pointCloud_out.header.stamp = ros::Time::now();
      cloud_in_rep.publish(input);
      cout << "START MAPPING" << endl;
    }
      else
    {
      cout << "STOP MAPPING" << endl;
    }
}



void PerceptionServer::from_unity_start_stop_mapping_callback(const std_msgs::Bool &msg)
{
    /*Provide a way to pause the map */
    start_mapping = msg.data;
    
}


void PerceptionServer::mocap_scene_root_in_unity_frame_callback(const geometry_msgs::PoseStamped &msg)
{
     /*
     Callback to receive the position of the MSR Unity frame respect the world frame.
     */
     //!!!!! REMEBER
     mocap_sr_unity_world_position.x = 0; //msg.pose.position.x;
     mocap_sr_unity_world_position.y = 0; //msg.pose.position.y;
     mocap_sr_unity_world_position.z = 0; //msg.pose.position.z;
      
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


void PerceptionServer::holo_position_callback(const geometry_msgs::Pose &msg)
{
    hololens_position = msg.position;
    hololens_orientation = msg.orientation;
}

void PerceptionServer::virtual_obs_callback(const scene_understanding_pkg_msgs::RRTObstaclesCoo &msg)
{
  /*
  Obtain the pose of the virtual obstacle if required 
  */
  for (int i = 0; i < msg.point.size(); i++)
 {
   //Take obstacles data
   virtual_obstacles_vec.push_back(msg.point[i]);
 }

}

void PerceptionServer::planner_start_assistive_guidance_callback(const std_msgs::Bool flag)
{
    APVI = flag.data;
}

void PerceptionServer::voxblox_mesh_callback(const voxblox_msgs::Mesh::ConstPtr &msg){
  /* Function Description
  *
  */
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

            voxblox_mesh_vertices.push_back(p_new);
            
            mesh.indices.push_back(vertex_index++);
            mesh.vertices.emplace_back(p_new.x, p_new.y, p_new.z); //mesh_x, mesh_y, mesh_z
        }
    }
  //This part should be inside the visualizer function populate_RVIZ_triangle_list_with_voxblox_pc
    int counter = 0;
    int counter_clock_wise =0;
    for (int tri_index = 0; tri_index < voxblox_mesh_vertices.size() / 3; ++tri_index)  
    {
      geometry_msgs::Point p;
      p = voxblox_mesh_vertices[counter]; 
      triangles.points.push_back(p);
      p = voxblox_mesh_vertices[counter + 1]; 
      triangles.points.push_back(p);
      p = voxblox_mesh_vertices[counter + 2]; //counter_clock_wise
      triangles.points.push_back(p);
      
      counter = counter + 3;
      counter_clock_wise = counter;
    }

}

void  PerceptionServer::quadrotor_pose_callback(const nav_msgs::Odometry &msg)
{
   
    quad_position.x = msg.pose.pose.position.x;
    quad_position.y = msg.pose.pose.position.y;
    quad_position.z = msg.pose.pose.position.z;
    //orientation
     
    // Covert to euler angles
    quad_quat_orientation = msg.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    quad_euler_orientation.x = roll;
    quad_euler_orientation.y = pitch;
    quad_euler_orientation.z = yaw;

    //Computing the Obstacle Force repulsion every time a new pose is received 
    evaluate_obstacle_force_field();

    //Set up the tf frames depending if running in simulation or real environment
    if (in_simulation)
    {
      publish_mocap_sim_tf();
    }
    else
    {
      publish_hololens_pose_in_RVIZ();
    }
}

void PerceptionServer::from_unity_reset_map_callback(const std_msgs::Bool &msg)
{
    bool reset_map_flag = msg.data;

    if (reset_map_flag)
    {
    std_srvs::Empty srv;
    clear_map_service.waitForExistence();
    
    if (clear_map_service.call(srv))
    {
    ROS_ERROR("Successfully called service clear_map");
    }
    else
    {
    ROS_ERROR("Failed to call service clear_map");
    }
     pc_to_unity.points.clear();
    }
   
}

/*
* Publisher
*/

 void PerceptionServer::publish_mesh_to_unity(visualization_msgs::Marker triangles) {
    //Iterate on the triangles and republish them as geometry messsage array
    scene_understanding_pkg_msgs::MeshVertexPosition msg_point;
    geometry_msgs::Point p;
    
    cout << "Points Published to Unity: " << triangles.points.size() << endl;
    for (int ii = 0; ii < triangles.points.size(); ii++)
    {
        p = triangles.points[ii];
        msg_point.point.push_back(p);
    }
    
    if (msg_point.point.size() > 0)
    {
        this->mesh_to_unity.publish(msg_point);
    }
       
   }

void PerceptionServer::publish_obs_force_array_marker(float theta)
{
   float roll = 0.0;
   float pitch = 0.0;
   float yaw = theta;

   Eigen::Vector4f quat;
   perception_utils.euler_to_quat(&quat, roll, pitch, yaw);
  //Convert theta from radians orientation (0,0, theta) only on the yaw to a quaternion value. 

  obs_magn_force_b_frame.pose.orientation.x = quat(0);
  obs_magn_force_b_frame.pose.orientation.y = quat(1);
  obs_magn_force_b_frame.pose.orientation.z = quat(2);
  obs_magn_force_b_frame.pose.orientation.w = quat(3);

 
  
  //Position is the position of the robot
  obs_magn_force_b_frame.pose.position.x  = ( mocap_sr_unity_world_position.x + quad_position.x); 
  obs_magn_force_b_frame.pose.position.y  = ( mocap_sr_unity_world_position.y + quad_position.y); 
  obs_magn_force_b_frame.pose.position.z  = quad_position.z; 

  obs_magn_force_b_frame.scale.x = 0.2 * F_magn;
  marker_pub_force_arrow_rviz.publish(obs_magn_force_b_frame);


}

void PerceptionServer::publish_obstacle_force_resultant(float fx, float fy, float Fm)
{
    scene_understanding_pkg_msgs::ObstacleRepForce msg;
    msg.Fx = fx;
    msg.Fy = fy;
    msg.F_magn = Fm;
    obst_force.publish(msg);
}


/*
* Publish TF tranform
*/
void PerceptionServer::publish_mocap_sim_tf()
{
    //TF Publisher  
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

} //namespace