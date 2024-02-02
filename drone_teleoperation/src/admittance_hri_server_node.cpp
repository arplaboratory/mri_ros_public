#include "drone_teleoperation/admittance_hri_server.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "admittance_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  string path;
  string folder_name;
  string global_path;
  nh.getParam("/tele_control_params/log_files_directory", path );
  nh.getParam("/tele_control_params/desired_txt_folder_name", folder_name );
  global_path = path + folder_name;
  boost::filesystem::create_directories(global_path.c_str());
  hri_admittance::HRIControlServer node(nh, nh_private, global_path);

  ros::spin();
  return 0;
}