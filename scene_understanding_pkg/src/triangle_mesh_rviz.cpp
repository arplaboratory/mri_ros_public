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
using namespace std;

#include </home/luca/luca_ws/src/scene_understanding_pkg/include/SU_Unity_comm.h>


using namespace std;
using namespace std::chrono;

// vectors where vertices from file are stored 
typedef vector <double> record_t;
typedef vector <record_t> vertices_t;

struct Mesh_data
{
  
  vector <vector<double>> vertices;
  vector <int> indices;
  vector <int> vertices_n;
  vector <int> indices_n;
  vector<vector<double>> new_vertices; //List of vertices after reordering themm

} mesh_data;

/* Read Vertices File */
istream& operator >> ( istream& ins, record_t& record )
  {
  // make sure that the returned record contains only the stuff we read now
  record.clear();

  // read the entire line into a string (a CSV record is terminated by a newline)
  string line;
  getline( ins, line );
 
  // now we'll use a stringstream to separate the fields out of the line
  stringstream ss( line );
  string field;
  while (getline( ss, field, ',' ))
    {
    // for each field we wish to convert it to a double
    // (since we require that the CSV contains nothing but floating-point values)
    stringstream fs( field );
    double f = 0.0;  // (default value is 0.0)
    fs >> f;
    
    // add the newly-converted field to the end of the record
    record.push_back( f );
    }

  // Now we have read a single line, converted into a list of fields, converted the fields
  // from strings to doubles, and stored the results in the argument record, so
  // we just return the argument stream as required for this kind of input overload function.
  return ins;
  }

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.
istream& operator >> ( istream& ins, vertices_t& data )
  {
  // make sure that the returned data only contains the CSV data we read here
  data.clear();

  // For every record we can read from the file, append it to our resulting data
  record_t record;
  while (ins >> record)
    {
    data.push_back( record );

    }

  // Again, return the argument stream as required for this kind of input stream overload.
  return ins;  
  }

// Used To read from file indices list, indices number and vertices nu,ber for each mesh
void read_list_of_int_from_txt(string path, vector<int> &vec)
{
  vec.clear();

  ifstream infile(path);
  string line = "a";
  while (line.length() != 0)
  {
  getline( infile, line, '\n' );
 
  stringstream fs( line );
  double f = 0.0;  // (default value is 0.0)
  fs >> f;

  vec.push_back( f );
  }


}



void initialize_visualization_message(SU_Unity_comm *unity_data)
{
  unity_data-> triangles.header.frame_id =  unity_data-> perimeter_lines.header.frame_id = "/map";
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
  unity_data-> triangles.color.a = 0.9;

  unity_data-> perimeter_lines.color.g = 0.0f;
  unity_data->  perimeter_lines.color.r = 0.0f;
  unity_data-> perimeter_lines.color.b = 0.0f;
  unity_data-> perimeter_lines.color.a = 1.0;
}

void populate_vertices_vector(Mesh_data *mesh_data,  vertices_t vertices)
{
for (int i = 0; i < vertices.size(); i++)
  {
    vector<double> record;
    for (int j = 0; j < 3; j++)
    {
      record.push_back(vertices[i][j]);
      //cout << "record: " <<record[j] << endl;
    }
    mesh_data->vertices.push_back(record);
   
  }
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
void reordering_vertices_list_with_indices(Mesh_data *mesh_data)
{
  int row_indices_reached = 0;
  int row_vertices_reached = 0;
  
  for (int i = 0; i < mesh_data->indices_n.size(); i++) //mesh_data->indices_n.size()
  {
    // Create list of indices related to the considered mesh 
    vector<int> new_indices;
    int index_n = mesh_data->indices_n[i];
    
    for (int ii = 0; ii < index_n; ii++)
    {
      int row_indices_list = row_indices_reached + ii;
      new_indices.push_back(mesh_data->indices[row_indices_list]);
      //cout << "new_indices i: " <<  new_indices[ii] << endl;
      }

    for  (int j = 0; j < new_indices.size(); j++)
    {
      //Vertices row to consider and to add to the new vector
      int row = row_vertices_reached + new_indices[j]; //mesh_data->vertices[row_vertices_reached + new_indices[j]];
       mesh_data->new_vertices.push_back(mesh_data->vertices[row]);
       //cout << "vertices considered x: " <<  mesh_data->vertices[row][0] << mesh_data->vertices[row][1] << endl;
    }
    row_vertices_reached = row_vertices_reached + mesh_data->vertices_n[i];
    //cout << "row_vertices_reached : " << row_vertices_reached << endl;
  
    row_indices_reached = row_indices_reached + index_n;
    //cout << "row_indices_reached : " <<  row_indices_reached << endl; 
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "publish_mesh_to_rviz");
  ros::NodeHandle n;
  
  SU_Unity_comm unity_data;
  Mesh_data mesh_data;

  bool load_data_from_txt_file = false;
  
  
     /* Load Vertices From TXT File */
  vertices_t vertices;
  ifstream infile( "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/Exp_4/vertices_coo.txt" );
  infile >> vertices;
  
  string path = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/Exp_4/vertices_indices.txt";
  read_list_of_int_from_txt(path, mesh_data.indices);
  
  path = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/Exp_4/number_of_vertices_in_each_mesh.txt";
  read_list_of_int_from_txt(path, mesh_data.vertices_n);
  
  path = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Mesh_Vertices_data/Exp_4/number_of_indices_in_each_mesh.txt";
  read_list_of_int_from_txt(path, mesh_data.indices_n);

   //Populate Struct Vector -- Required Only Because done with ifstream operator 
  populate_vertices_vector(&mesh_data, vertices);


  
 


  // Reoreder list of vertices following indices list to be correctly displayed by the mesh generator in rviz
  reordering_vertices_list_with_indices(&mesh_data);

  auto start = high_resolution_clock::now();
  auto last_published_mesh =  high_resolution_clock::now();
  auto elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
  auto mesh_publish_period = 10000000.0;
  bool init_flag = true;
  ros::Rate r(30);
 
  while (ros::ok())
  {
    initialize_visualization_message(&unity_data);
    
   
     //the number of triangles in mesh is 1/3 the number of vertices 
     int n_triangles = mesh_data.new_vertices.size()/3;
     int count = 0;
    
     if (init_flag == true || elapesed_time.count()  > mesh_publish_period)
     {
          // Create the vertices for the points and lines
        for (uint32_t i = 0; i < n_triangles; i++)
        {
     
          geometry_msgs::Point p;
          //vertex 1
          p.x = mesh_data.new_vertices[count][0]; //(int32_t)i - 50;  // 2, 0, 1
          p.y = mesh_data.new_vertices[count][2];
          p.z = mesh_data.new_vertices[count][1];
          unity_data.triangles.points.push_back(p);
    
          // Vertex 2
          p.x = mesh_data.new_vertices[count + 1][0]; //(int32_t)i - 50;
          p.y = mesh_data.new_vertices[count + 1][2];
          p.z = mesh_data.new_vertices[count + 1][1];
          unity_data.triangles.points.push_back(p);
    
          // Vertex 3
          p.x = mesh_data.new_vertices[count + 2][0]; //(int32_t)i - 50;
          p.y = mesh_data.new_vertices[count + 2][2];
          p.z = mesh_data.new_vertices[count + 2][1];
          unity_data.triangles.points.push_back(p);
          
          count = count + 3;
          
         
        }
    
    
        //marker_pub.publish(points);
         unity_data.publish_mesh_vertices_to_rviz(unity_data.triangles);
         cout << "Points Published!" << endl;
        last_published_mesh = high_resolution_clock::now();
        init_flag = false;
     }
    
    //unity_data.publish_mesh_triangles_perimeter_to_rviz(unity_data.perimeter_lines);
    start = high_resolution_clock::now();
    elapesed_time = duration_cast<microseconds>(start - last_published_mesh);
    r.sleep();

    
  }
}
