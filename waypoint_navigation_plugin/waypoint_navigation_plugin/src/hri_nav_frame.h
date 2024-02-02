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

#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include <QWidget>
#include <iostream>
#include "gnuplot.h"
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <mav_manager/Vec4.h>
#include <unistd.h>
#include <Eigen/Sparse>
#include "ui_HRI_coworker.h"
#include <visualization_msgs/MarkerArray.h>


typedef struct {
    int derivOrder, vertexNum;
    Eigen::Vector4d lower, upper;
    bool enable; //Declares wether this constraint is active or not
} waypoint_ineq_const;

  static const std::string Deriv_title[5] = { "'Pos'", "'Vel'", "'accel'", "'jerk'" , "'snap'"};
  static const std::string append[5] = { "Pos", "Vel", "accel", "jerk" , "snap"};

namespace Ogre
{
class SceneNode;
class Vector3;
class SceneManager;
class Quaternion;
}

namespace rviz
{
class DisplayContext;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace Ui
{
class HRI_coworkerWidget;
}

namespace waypoint_nav_plugin
{
class HriNavTool;
}

namespace waypoint_nav_plugin
{

class HriFrame : public QWidget
{
  friend class HriNavTool;
  Q_OBJECT

public:
  HriFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent = 0, HriNavTool* wp_tool=0);
  ~HriFrame();

  void enable();
  void disable();

  std::vector<waypoint_ineq_const> ineq_list;


protected:

  Ui::HRI_coworkerWidget *ui_;
  rviz::DisplayContext* context_;

private Q_SLOTS:


  //void rollChanged(double val);
  //void pitchChanged(double val);


  //Buttons RQT MAV MANAGER
  void motors_on_push_button();
  void motors_off_push_button();
  void land_push_button();
  void takeoff_push_button();
  void goto_push_button();
  void relativeChanged(int b);
  void robotChanged();
  void serviceChanged();
  void goHome_push_button();
  void hover_push_button();
  void clear_map();
  void clear_path();
  void rqt_change_mode_();
  void start_FPVI_task();
  void start_APVI_task();
  void exit_task();
  void rotate_180_yaw();
   //Bernstein Check boxes


  //Inequality cahgned for each double box
  /*
  void pl_ineqChanged(double val);
  void xl_ineqChanged(double val);
  void yl_ineqChanged(double val);
  void zl_ineqChanged(double val);
  void pu_ineqChanged(double val);
  void xu_ineqChanged(double val);
  void yu_ineqChanged(double val);
  void zu_ineqChanged(double val);
  void en_ineqChanged(double val);
  void do_ineqChanged(double val);*/

private:

  ros::NodeHandle nh_;

  ros::Publisher path_clear_pub_;
  ros::Publisher rqt_change_mode;
  ros::Publisher rqt_input_start_FPVI_task;
  ros::Publisher rqt_input_start_APVI_task;
  ros::Publisher rqt_input_rotate_180_yaw;



  HriNavTool* hri_nav_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;

  interactive_markers::InteractiveMarkerServer* server_;

  //default height the waypoint must be placed at
  bool relative_ = true;


  // The current name of the output topic.
  std::string robot_name = "quadrotor";
  std::string mav_node_name = "mav_services";

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;


};

}

#endif
