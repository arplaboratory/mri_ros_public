#include "drone_teleoperation/utils/log_files_writing.h"


namespace hri_admittance {

  LOGFilesWriting::LOGFilesWriting(string path)
                        
                        : ofs1(path + "/x_des.txt"),
                        ofs2(path + "/y_des_GF.txt"),
                        ofs3(path + "/z_des_GF.txt"),
                        ofs4(path + "/x_GF.txt"),
                        ofs5(path + "/y_GF.txt"),
                        ofs6(path + "/z_GF.txt"),
                        ofs7(path + "/x_dot_des_target_GF.txt"),
                        ofs8(path + "/y_dot_des_target_GF.txt"),
                        ofs9(path + "/z_dot_des_GF.txt"),
                        ofs10(path + "/x_dot_GF.txt"),
                        ofs11(path + "/y_dot_GF.txt"),
                        ofs12(path + "/z_dot_GF.txt"),
                        ofs13(path + "/yaw.txt"),
                        ofs15(path + "/F_x.txt"),
                        ofs16(path + "/x_c.txt"),
                        ofs17(path + "/x_c_dot.txt"),
                        ofs20(path + "/y_c.txt"),
                        ofs21(path + "/y_c_dot.txt"),
                        ofs22(path + "/F_y.txt"),
                        ofs23(path + "/KF_x_pos_est.txt"),
                        ofs24(path + "/KF_x_vel_est.txt"),
                        ofs25(path + "/KF_y_pos_est.txt"),
                        ofs26(path + "/KF_y_vel_est.txt"),
                        ofs27(path + "/KF_x_user_case_3_pos_est.txt"),
                        ofs28(path + "/KF_y_user_case_3_pos_est.txt"),
                        ofs29(path + "/KF_x_user_case_3_vel_est.txt"),
                        ofs30(path + "/KF_y_user_case_3_vel_est.txt"),
                        ofs31(path + "/user_target_pos_x_case_3.txt"),
                        ofs32(path + "/user_target_pos_y_case_3.txt"),
                        ofs33(path + "/user_target_vel_x_case_3.txt"),
                        ofs34(path + "/user_target_vel_y_case_3.txt"),
                        ofs35(path + "/damping_output.txt"),
                        ofs39(path + "/fx_obs.txt"),
                        ofs40(path + "/fy_obs.txt"),
                        ofs41(path + "/F_obs_magn.txt"),
                        ofs42(path + "/task_counters.txt"),
                        ofs43(path + "/drone_line_distance_APVI.txt"),
                       ofs44(path + "/FPVI_target_pos_assignment_and_reaching_for_drone_line_distance.txt"){

    }
    


    void LOGFilesWriting::writing()
    {
        ofs1 << marker_pos(0) << "\n";
        ofs2 << marker_pos(1) << "\n";
        ofs3 << marker_pos(2) << "\n";
        ofs4 << quad_pose(0) << "\n";
        ofs5 << quad_pose(1) << "\n";
        ofs6 << quad_pose(2) << "\n";
        ofs7 << marker_vel(0) << "\n";
        ofs8 << marker_vel(1) << "\n";
        ofs9 << marker_vel(2) << "\n"; 
        ofs10 << quad_vel(0) << "\n";
        ofs11 << quad_vel(1) << "\n";
        ofs12 << quad_vel(2) << "\n";
        ofs13 << quad_orientation(2) << "\n";
        ofs15 <<   F_admittance(0)  << "\n";
        ofs16 <<   pos_comm(0)  << "\n";
        ofs17 <<   vel_comm(0)  << "\n";
        ofs20 <<   pos_comm(1)  << "\n";
        ofs21 <<   vel_comm(1)  << "\n";
        ofs22 <<   F_admittance(1)  << "\n";
        ofs23 << kf_x_state.x << "\n";
        ofs24 << kf_x_state.y << "\n";
        ofs25 << kf_y_state.x << "\n";
        ofs26 << kf_y_state.y << "\n";
        //Only in case 3
        ofs27 << kf_user_x_state.x << "\n";
        ofs28 << kf_user_y_state.x << "\n";
        ofs29 << kf_user_x_state.y << "\n";
        ofs30 << kf_user_y_state.y << "\n";
        ofs31 << marker_proj_pos(0) << "\n";
        ofs32 << marker_proj_pos(1) << "\n";
        ofs33 << marker_proj_vel(0) << "\n";
        ofs34 << marker_proj_vel(1) << "\n";
        ofs35 << d << "\n";
        // outFile36 << drone.gamma_output << "\n";
        // outFile37 << mission.setpoint_old.x << ", " << mission.setpoint_old.y << endl;
        // outFile38 << mission.setpoint.x << ", " << mission.setpoint.y << endl;
        ofs39 << f_vec(0) << "\n";
        ofs40 << f_vec(1) << "\n";
        float force_obs_Magn = sqrt(pow(f_vec(0),2) + pow(f_vec(1),2));
        ofs41 << force_obs_Magn << "\n";
        ofs42 << FPVI_int_goals_ <<", " << FPVI_final_goal_ << ", " << APVI_final_goal_ << ", " << FPVI_start_ << ", " << APVI_start_ << ", " << APVI_end_ << ", " << FPVI_end_ << "\n";
        ofs43 << distance_ << "\n";
        ofs44 << FPVI_int_goal1_pos_x_ <<", " << FPVI_int_goal1_pos_y_ << ", " << FPVI_int_goal2_pos_x_ << ", " << FPVI_int_goal2_pos_y_ << ", " << FPVI_final_goal_pos_x_ << ", " << FPVI_final_goal_pos_y_ << ", " << FPVI_int_goal1_reached_ << ", " <<FPVI_int_goal2_reached_ <<", " << FPVI_final_goal_reached_ <<"\n";
    }

    void LOGFilesWriting::get_des_pose(Eigen::Vector3f marker_pos_)
    {
       marker_pos = marker_pos_;
    }
    void LOGFilesWriting::get_des_pose_projected(Eigen::Vector3f marker_proj_pos_)
    {
      marker_proj_pos = marker_proj_pos_;
    }
    void LOGFilesWriting::get_quad_pose(Eigen::Vector3f quad_pose_, Eigen::Vector3f quad_orientation_)
    {
       quad_pose = quad_pose_;
       quad_orientation = quad_orientation_;
    }
    void LOGFilesWriting::get_target_vel(Eigen::Vector3f marker_vel_)
    {
        marker_vel = marker_vel_;
    }
    void LOGFilesWriting::get_quad_vel(Eigen::Vector3f quad_vel_)
    {
        quad_vel = quad_vel_;
    }
    void LOGFilesWriting::get_Admittance_Force(Eigen::Vector3f F_admittance_)
    {
        F_admittance = F_admittance_;
    }
    void LOGFilesWriting::get_commanded_position(Eigen::Vector3f pos_comm_)
    {
       pos_comm = pos_comm_;
    }
    void LOGFilesWriting::get_commanded_velocity(Eigen::Vector3f vel_comm_)
    {
       vel_comm = vel_comm_;
    }
    void LOGFilesWriting::get_commanded_kfx_state(geometry_msgs::Vector3 kf_x_state_)
    {
      kf_x_state = kf_x_state_;
    }
    void LOGFilesWriting::get_commanded_kf_userx_state(geometry_msgs::Vector3 kf_user_x_state_)
    {
      kf_user_x_state = kf_user_x_state_;
    }
    void LOGFilesWriting::get_commanded_kfy_state(geometry_msgs::Vector3 kf_y_state_)
    {
      kf_y_state = kf_y_state_;
    }
    void LOGFilesWriting::get_commanded_kf_usery_state(geometry_msgs::Vector3 kf_user_y_state_)
    {
      kf_user_y_state = kf_user_y_state_;
    }
    void LOGFilesWriting::get_target_user_projected_vel(Eigen::Vector3f marker_proj_vel_)
    {
      marker_proj_vel = marker_proj_vel_;
    }
    void LOGFilesWriting::get_damping(float d_)
    {
      d = d_;
    }
    void LOGFilesWriting::get_obstacle_force_vector(Eigen::Vector3f f_vec_)
    {
      f_vec = f_vec_;
    }
    
    void LOGFilesWriting::get_task_counters(int FPVI_int_goals, int FPVI_final_goal, int APVI_final_goal, int FPVI_start, int APVI_start, int APVI_end, int FPVI_end)
    {
      FPVI_int_goals_ = FPVI_int_goals;
      FPVI_final_goal_ = FPVI_final_goal;
      APVI_final_goal_ = APVI_final_goal;
      FPVI_start_ = FPVI_start;
      APVI_start_ = APVI_start;
      FPVI_end_ = FPVI_end;
      APVI_end_ = APVI_end;
    }
    
    void LOGFilesWriting::get_target_position_FPVI(Eigen::Vector3f FPVI_int_goal1_pos, Eigen::Vector3f FPVI_int_goal2_pos,  Eigen::Vector3f FPVI_final_goal_pos, int FPVI_int_goal1_reached, int FPVI_int_goal2_reached, int FPVI_final_goal_reached)
    {
      FPVI_int_goal1_pos_x_ = FPVI_int_goal1_pos(0);
      FPVI_int_goal1_pos_y_ = FPVI_int_goal1_pos(1);
      FPVI_int_goal2_pos_x_ = FPVI_int_goal2_pos(0);
      FPVI_int_goal2_pos_y_ = FPVI_int_goal2_pos(1);
      FPVI_final_goal_pos_x_ = FPVI_final_goal_pos(0);
      FPVI_final_goal_pos_y_ = FPVI_final_goal_pos(1);
      FPVI_int_goal1_reached_ = FPVI_int_goal1_reached; //1 if reached, define the order how the number change from zero to one
      FPVI_int_goal2_reached_ = FPVI_int_goal2_reached;
      FPVI_final_goal_reached_ = FPVI_final_goal_reached;
      
    }


    void LOGFilesWriting::get_drone_line_distance(float distance)
    {
       distance_ = distance;
    }
    

}