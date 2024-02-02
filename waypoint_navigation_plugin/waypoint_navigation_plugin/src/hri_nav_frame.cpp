/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "hri_nav_frame.h"
//#include "waypoint_nav_tool.h"


//#include "waypoint_nav_frame.h"
//#include "waypoint_nav_frame.h"

#include <tf/tf.h>

#include <QFileDialog>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace waypoint_nav_plugin
{

HriFrame::HriFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent, HriNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::HRI_coworkerWidget())
  , sn_map_ptr_(map_ptr)
{
  scene_manager_ = context_->getSceneManager();

  // set up the GUI
  ui_->setupUi(this);

 
  path_clear_pub_ = nh_.advertise<std_msgs::Bool>("/clear", 1);
  rqt_change_mode = nh_.advertise<std_msgs::Int32>("/rqt_input/case_switcher", 1);
  rqt_input_start_FPVI_task = nh_.advertise<std_msgs::Bool>("/rqt_input/start_FPVI_task", 1);
  rqt_input_start_APVI_task = nh_.advertise<std_msgs::Bool>("/rqt_input/start_APVI_task", 1);
  rqt_input_rotate_180_yaw = nh_.advertise<std_msgs::Bool>("/rqt_input/rotate_180_yaw", 1);
  //connect the Qt signals and slots
  




  //ROSRUN RQT Mav Manager Line topics 
  connect(ui_->robot_name_line_edit, SIGNAL(editingFinished()), this, SLOT(robotChanged()));
  connect(ui_->node_name_line_edit, SIGNAL(editingFinished()), this, SLOT(serviceChanged()));
  //Buttons
  connect(ui_->motors_on_push_button, SIGNAL(clicked()), this, SLOT(motors_on_push_button()));
  connect(ui_->motors_off_push_button, SIGNAL(clicked()), this, SLOT(motors_off_push_button()));
  connect(ui_->land_push_button, SIGNAL(clicked()), this, SLOT(land_push_button()));
  connect(ui_->takeoff_push_button, SIGNAL(clicked()), this, SLOT(takeoff_push_button()));
  connect(ui_->goto_push_button, SIGNAL(clicked()), this, SLOT(goto_push_button()));
  connect(ui_->relative_checkbox, SIGNAL(stateChanged(int)), this, SLOT(relativeChanged(int)));
  connect(ui_->go_home_button, SIGNAL(clicked()), this, SLOT(goHome_push_button()));
  connect(ui_->hover, SIGNAL(clicked()), this, SLOT(hover_push_button()));
  connect(ui_->clear_path, SIGNAL(clicked()), this, SLOT(clear_path()));
  connect(ui_->reset_map, SIGNAL(clicked()), this, SLOT(clear_map()));
  connect(ui_->take_control_button, SIGNAL(clicked()), this, SLOT(rqt_change_mode_()));
  connect(ui_->start_FPVI_task_button, SIGNAL(clicked()), this, SLOT(start_FPVI_task()));
  connect(ui_->start_APVI_task_button, SIGNAL(clicked()), this, SLOT(start_APVI_task()));
  connect(ui_->exit_task_button, SIGNAL(clicked()), this, SLOT(exit_task()));
  connect(ui_->rotate180, SIGNAL(clicked()), this, SLOT(rotate_180_yaw()));
/*
  connect(ui_->bern_pl, SIGNAL(valueChanged(double)), this, SLOT(pl_ineqChanged(double)));
  connect(ui_->bern_xl, SIGNAL(valueChanged(double)), this, SLOT(xl_ineqChanged(double)));
  connect(ui_->bern_yl, SIGNAL(valueChanged(double)), this, SLOT(yl_ineqChanged(double)));
  connect(ui_->bern_zl, SIGNAL(valueChanged(double)), this, SLOT(zl_ineqChanged(double)));

  connect(ui_->bern_pu, SIGNAL(valueChanged(double)), this, SLOT(pu_ineqChanged(double)));
  connect(ui_->bern_xu, SIGNAL(valueChanged(double)), this, SLOT(xu_ineqChanged(double)));
  connect(ui_->bern_yu, SIGNAL(valueChanged(double)), this, SLOT(yu_ineqChanged(double)));
  connect(ui_->bern_zu, SIGNAL(valueChanged(double)), this, SLOT(zu_ineqChanged(double)));

  connect(ui_->ineq_enable, SIGNAL(valueChanged(double)), this, SLOT(en_ineqChanged(double)));
  connect(ui_->deriv_order, SIGNAL(valueChanged(double)), this, SLOT(do_ineqChanged(double)));
*/

}

HriFrame::~HriFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}




void HriFrame::enable()
{
  // activate the frame
  show();
}

void HriFrame::disable()
{

  hide();
}

void HriFrame::clear_path()
{
   std_msgs::Bool thing;
   path_clear_pub_.publish(thing);
}




  //Clear Map
void HriFrame::clear_map(){
	ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/voxblox_node/clear_map");
	std_srvs::Empty srv;
  //client.waitForExistence();
  if (client.call(srv)){
      ROS_INFO("Successfully called service clear_map");
  }else{
      ROS_ERROR("Failed clear_map");
  }
}

void HriFrame::rqt_change_mode_(){
    int mode = 1;
	std_msgs::Int32 mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_change_mode.publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::start_FPVI_task(){
    bool mode = true;
	std_msgs::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_start_FPVI_task.publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::start_APVI_task(){
    bool mode = true;
	std_msgs::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_start_APVI_task.publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::exit_task(){
    int mode = 2;
	std_msgs::Int32 mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_change_mode.publish(mode_);
        sleep(0.01);
    }
}

void HriFrame::rotate_180_yaw(){
    bool mode = true;
	std_msgs::Bool mode_;
    mode_.data = mode;
    for (int i = 0; i < 100; i++)
    {
        rqt_input_rotate_180_yaw.publish(mode_);
        sleep(0.01);
    }
}



  //Buttons RQT MAV MANAGER
void HriFrame::motors_on_push_button(){
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(srvs_name);
	std_srvs::SetBool srv;
	srv.request.data = true;
	if (client.call(srv))
	{
		ROS_INFO("MOTORS STARTED");
	}
	else
	{
		ROS_ERROR("FAILED TO START MOTORS");
	}	
}

void HriFrame::motors_off_push_button(){
	ros::NodeHandle nh;
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/motors";
	ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(srvs_name);
	std_srvs::SetBool srv;
	srv.request.data = false;
	if (client.call(srv))
	{
		ROS_INFO("MOTORS STOPPED");
	}
	else
	{
		ROS_ERROR("FAILED TO STOP MOTORS");
	}	
}

void HriFrame::hover_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	ros::NodeHandle nh;
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/hover";
	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	std_srvs::Trigger srv;
	if (client.call(srv))
	{
		ROS_INFO("Hover Success");
	}
	else
	{	
		ROS_ERROR("Failed Hover ");
	}		
}

void HriFrame::land_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	ros::NodeHandle nh;
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/land";
	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	std_srvs::Trigger srv;
	if (client.call(srv))
	{
		ROS_INFO("Land Success");
	}
	else
	{	
		ROS_ERROR("Failed Land ");
	}		
}
void HriFrame::takeoff_push_button(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
	ros::NodeHandle nh;
	std::string srvs_name = "/"+ robot_name+"/"+mav_node_name+"/takeoff";
	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(srvs_name);
	std_srvs::Trigger srv;
	if (client.call(srv))
	{
		ROS_INFO("Takeoff Success");
	}
	else
	{	
		ROS_ERROR("Failed takeoff ");
	}		
}

void HriFrame::goto_push_button(){
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
	ros::NodeHandle nh;
	std::string srvs_name;
	if(relative_){
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goToRelative";
	}
	else{
		srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	}
	ros::ServiceClient client = nh.serviceClient<mav_manager::Vec4>(srvs_name);
	mav_manager::Vec4 srv;
  	srv.request.goal [0] = ui_->x_doubleSpinBox_gt->value();
 	srv.request.goal [1] = ui_->y_doubleSpinBox_gt->value();
  	srv.request.goal [2] = ui_->z_doubleSpinBox_gt->value();
  	srv.request.goal [3] = ui_->yaw_doubleSpinBox_gt->value();
	if (client.call(srv))
	{
		ROS_INFO("GoTo Success");
	}
	else
	{	
		ROS_ERROR("Failed GoTo ");
	}		

}

void HriFrame::relativeChanged(int b){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  if (b ==2){
	  relative_ = true;
  }
  else{
	 relative_ = false;
  }
}


void HriFrame::robotChanged(){ 
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->robot_name_line_edit->text();
  robot_name =  new_frame.toStdString();
 




}

void HriFrame::serviceChanged(){
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->node_name_line_edit->text();
  mav_node_name =  new_frame.toStdString();
}

void HriFrame::goHome_push_button(){
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
	ros::NodeHandle nh;
	std::string srvs_name;
	srvs_name = "/"+ robot_name+"/"+mav_node_name+"/goTo";
	ros::ServiceClient client = nh.serviceClient<mav_manager::Vec4>(srvs_name);
	mav_manager::Vec4 srv;
  	srv.request.goal [0] = 0;
 	srv.request.goal [1] = 0;
  	srv.request.goal [2] = 0.5;
  	srv.request.goal [3] = 0;
	if (client.call(srv))
	{
		ROS_INFO("Go Home Success");
	}
	else
	{	
		ROS_ERROR("Failed Go Home ");
	}		
  }

}//namespace

/*
  void HriFrame::pl_ineqChanged(double val){
    ineqChanged(val, 0,3);
  }
  void HriFrame::xl_ineqChanged(double val){
    ineqChanged(val, 0,0);

  }
  void HriFrame::yl_ineqChanged(double val){
    ineqChanged(val, 0,1);

  }  
  void HriFrame::zl_ineqChanged(double val){
    ineqChanged(val, 0,2);

  }  
  void HriFrame::pu_ineqChanged(double val){
    ineqChanged(val, 1,3);

  }  
  void HriFrame::xu_ineqChanged(double val){
    ineqChanged(val, 1,0);
  }

  void HriFrame::yu_ineqChanged(double val){
    ineqChanged(val, 1,1);
  }

  void HriFrame::zu_ineqChanged(double val){
    ineqChanged(val, 1,2);
  }  

  void HriFrame::en_ineqChanged(double val){
    ineqChanged(val, 2,0);
  }

  void HriFrame::do_ineqChanged(double val){
    ineqChanged(val, 3,0);
  }
} // namespace*/
