#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <cmath>

using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher marker_pub_mesh_perimeter = n.advertise<visualization_msgs::Marker>("visualization_marker_perimeter", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker triangle, perimeter_line;
    //Unique Id Assigned to each marker
    triangle.header.frame_id = perimeter_line.header.frame_id = "/map";
    triangle.header.stamp  =  perimeter_line.header.stamp = ros::Time::now();
    //Markers Namespace 
    triangle.ns = perimeter_line.ns = "triangle_list";
    // Assign a type of action to each marker
    triangle.action = perimeter_line.action = visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important
    triangle.pose.orientation.w = perimeter_line.pose.orientation.w = 1.0;


    // Marker Type (Arrow, Sphere, points )
    triangle.type = visualization_msgs::Marker::TRIANGLE_LIST;   
    perimeter_line.type = visualization_msgs::Marker::LINE_LIST;
    
    triangle.id = 0;
    perimeter_line.id = 0;

    // POINTS markers use x and y scale for width/height respectively
    triangle.scale.x = 1;
    triangle.scale.y = 1;
    triangle.scale.z = 1;
    
    perimeter_line.scale.x = 0.05;
    perimeter_line.scale.y = 0.05;
    perimeter_line.scale.z = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
   


    // Points are green
    triangle.color.g = 1.0f;
    triangle.color.a = 0.5;

    perimeter_line.color.g = 0.0f;
    perimeter_line.color.r = 0.0f;
    perimeter_line.color.b = 0.0f;
    perimeter_line.color.a = 1.0;


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 1; ++i)
    {
 
      float y = i; //5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 0; //5 * cos(f + i / 100.0f * 2 * M_PI);
      
      //  Vertex 1
      geometry_msgs::Point p;
      p.x = 0; //(int32_t)i - 50;
      p.y = 0;
      p.z = 0;

     
      //Triangle list needs three vertex
      triangle.points.push_back(p);
      perimeter_line.points.push_back(p);
      // Vertex 2
      p.x = 0; //(int32_t)i - 50;
      p.y = 1;
      p.z = 0;
      
      triangle.points.push_back(p);
      perimeter_line.points.push_back(p);

      // Vertex 3 
      p.x = 0; //(int32_t)i - 50;
      p.y = 0;
      p.z = 1;

      triangle.points.push_back(p);
      
      p.x = 2; //(int32_t)i - 50;
      p.y = 0;
      p.z = 0;

     
      //Triangle list needs three vertex
      triangle.points.push_back(p);
      
      // Vertex 2
      p.x = 0; //(int32_t)i - 50;
      p.y = 1.8;
      p.z = 0;
      
      triangle.points.push_back(p);

      // Vertex 3 
      p.x = 0; //(int32_t)i - 50;
      p.y = 0;
      p.z = 1.7;

      triangle.points.push_back(p);
      cout << "Points Published!" << endl;
    }


    //marker_pub.publish(points);
    marker_pub.publish(triangle);
    marker_pub_mesh_perimeter.publish(perimeter_line);

    r.sleep();

    
  }
}