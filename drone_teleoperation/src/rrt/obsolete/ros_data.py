#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Mon Jul 13 15:18:34 2020

@author: lucamora
"""


import rospy 
import time
from geometry_msgs.msg import Twist,PoseWithCovariance, Quaternion, Point, Pose, Vector3, Vector3Stamped, PoseStamped, PoseWithCovarianceStamped,PointStamped
from std_msgs.msg import String, Header, Bool, Float32, Int32
from std_msgs.msg import Empty 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage, PointCloud2
from scene_understanding_pkg_msgs.msg  import RRTPathPoints2D, RRTObstaclesCoo
import sensor_msgs.point_cloud2 as pc2

#from keras.layers.core import Dense, Dropout, Activation, Flatten
# import matplotlib.pyplot as plt
import numpy as np
import time
import random
import math
import pdb
import sys
#import cv2
from cv_bridge import CvBridge, CvBridgeError


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians



def obtain_drone_odometry():
    poseData = None
    count_for_exit_while = 1
    roll = 0
    pitch = 0
    yaw = 0
    while poseData is None :
        try:
            poseData = rospy.wait_for_message('quadrotor/odom', Odometry, timeout = 10) #voxl3
            orientation_q = poseData.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        except:
            rospy.loginfo('Unable to reach the drone quadrotor/odom topic. Try to connect again')
            if count_for_exit_while > 100:
               break
            count_for_exit_while = count_for_exit_while +1
	

        return poseData.pose.pose.position, yaw


def obtain_final_goal():
    point = None
    count_for_exit_while = 1
    goal = Point()
    while point is None :
        try:
            point = rospy.wait_for_message('/rrt/final_goal', Point, timeout = 10)
            goal = point.data
            
        except:
            rospy.loginfo('Unable to reach the drone /rrt/final_goal topic. Try to connect again')
            if count_for_exit_while > 1:
               break
            count_for_exit_while = count_for_exit_while +1
        return goal



def obtain_voxels_positions():
   
    count_for_exit_while = 1
    pointcloud = PointCloud2()
    surface_pc = []
    terminal_flag = False
    while pointcloud.width == 0:
        try:
            pointcloud = rospy.wait_for_message('/voxblox_node/surface_pointcloud', PointCloud2, timeout = 10)
            #print("pointcloud_: ", pointcloud)
            for p in pc2.read_points(pointcloud, field_names = ("x", "y", "z"), skip_nans=True):
                surface_pc.append([p[0],p[1],p[2]])

            
        except:
            rospy.loginfo('Unable to reach the drone /voxblox_node_holo/surface_pointcloud topic. Try to connect again')
            if count_for_exit_while > 0:
                terminal_flag = True
                return surface_pc, terminal_flag
            count_for_exit_while = count_for_exit_while +1
        return surface_pc, terminal_flag





def take_flag_to_exit_rrt():
    flag_obtained = None
    count_for_exit_while = 1
    while flag_obtained is None :
        try: 
            flag_obtained = rospy.wait_for_message('/rrt/exit_search_mode', Bool, timeout = 5)
            
        except:
            rospy.loginfo('Unable to reach the topic /rrt/exit_search_mode. Try to connect again')
           
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
    return flag_obtained.data
    


def take_flag_inside_manouver():
    flag_obtained = None
    count_for_exit_while = 1
    while flag_obtained is None :
        try: 
            flag_obtained = rospy.wait_for_message('/rrt/inside_bypass_manouver', Bool, timeout = 5)
            
        except:
            rospy.loginfo('Unable to reach the topic /rrt/inside_bypass_manouver. Try to connect again')
           
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
    return flag_obtained.data
    
    

def publish_path_array_to_goal(path):
    """
    Publish a point array containing the coordinates oif the segnment evaliate to the goal
    """
    pub = rospy.Publisher('/rrt/path', RRTPathPoints2D, queue_size=1)
    msg = RRTPathPoints2D()
    counter = 0
    for i in path:
        point_ = Point(i[0], i[1], 0)
        msg.point.append(point_)
        counter = counter + 1
    pub.publish(msg)








#Publish flag final goal reached to the telecontrol script
def publish_final_goal_reached(goal_reached):
    pub = rospy.Publisher('/rrt/final_goal_reached', Bool, queue_size=1)
    msg = Bool()
    msg.data = goal_reached
    pub.publish(msg)


#Publish to RRT script the goal selected by the user 
def publish_rrt_goal(goal):
    pub =  rospy.Publisher('/rrt/final_goal', Point, latch=True, queue_size=1)  
    message = Point()
    message = goal
    pub.publish(message)




def publish_flag_to_enter_in_assistive_rrt_guidance(flag):
    pub =  rospy.Publisher('/rrt/start_assistive_guidance', Bool, latch=True, queue_size=1)  
    message = Bool()
    message.data = flag
    pub.publish(message)

def publish_obstacles_positions(obstacles, bounds):
    #define a geometry message array of points with a identifier string (obs_1, obs_2).
    #Each obstacle is identified in the worlframe by 4 vertices (min_x, min_y, max_x, max_y) 
    pub =  rospy.Publisher('/rrt/virtual_obstacles_coo', RRTObstaclesCoo, latch=True, queue_size=1) 
    name = "obstacle_"
    counter = 0
    obstacle_num = len(obstacles)
   
    th_length = bounds[0]
    th_height = bounds[1]
    
    
    for obs in obstacles:
        x_min = obs[0] + th_length
        y_min = obs[1] + th_height
        x_max = obs[2] - th_length
        y_max = obs[3] - th_height

        msg = RRTObstaclesCoo()
        msg.number = obstacle_num
        coo1 = Point(x_min, y_min, 0)
        coo2 = Point(x_max, y_max, 0)
        msg.point.append(coo1)
        msg.point.append(coo2)
        msg.name = name + str(counter)
        pub.publish(msg)
        time.sleep(0.2)
        counter = counter + 1

    
    

def publish_final_goal_position(goal):
    pub =  rospy.Publisher('/rrt/final_goal', Point, latch=True, queue_size=1) 
    point = Point()
    point.x = goal[0]
    point.y = goal[1]
    point.z = 0.9
    pub.publish(point) 





# def take_color_frame_Realsense_camera():
#     camera_frame = None
#     count_for_exit_while = 1
#     while camera_frame is None :
#         try:
#             camera_frame = rospy.wait_for_message('/camera/color/image_raw', Image, timeout = 5)
#             bridge = CvBridge()
#             cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
#         except:
#             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
#             if count_for_exit_while > 100:
#                 break
#             count_for_exit_while = count_for_exit_while +1
#     return cv_image

# #Take images from Kimera 
# def take_kimera_frame_image_euroc():
#     camera_frame = None
#     count_for_exit_while = 1
#     while camera_frame is None :
#         try:
#             camera_frame = rospy.wait_for_message('/cam0/image_raw', Image, timeout = 5)
#             bridge = CvBridge()
#             cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
#         except:
#             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
#             if count_for_exit_while > 100:
#                 break
#             count_for_exit_while = count_for_exit_while +1
#     return cv_image

# def take_kimera_frame_realsense():
#     camera_frame = None
#     count_for_exit_while = 1
#     while camera_frame is None :
#         try: 
#             camera_frame = rospy.wait_for_message('/camera/infra1/image_rect_raw', Image, timeout = 5)
#             bridge = CvBridge()
#             cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
#         except:
#             rospy.loginfo('Unable to reach the topic /camera/infra1/image_rect_raw. Try to connect again')
           
#             if count_for_exit_while > 100:
#                 break
#             count_for_exit_while = count_for_exit_while +1
#     return cv_image
    

def publish_matching_flag(flag):
    """
    Publish a flag that indicates when the two images presents matching features 
    """
    pub = rospy.Publisher('/feature_detector_py/feature_matching_flag', Bool, queue_size=1)
    msg = Bool()
    msg.data = flag
    pub.publish(msg)



#Publisher Related to the teleoperation package 

def publish_desired_mode(mode):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/case_switcher', Int32, latch=True, queue_size=1)  
    message = Int32()
    message.data = mode
    pub.publish(message)

