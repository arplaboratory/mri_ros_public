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
from sensor_msgs.msg import Imu, Image, CompressedImage

#from keras.layers.core import Dense, Dropout, Activation, Flatten
import matplotlib.pyplot as plt
import numpy as np
import time
import random
import math
import pdb
import cv2
import sys

from cv_bridge import CvBridge, CvBridgeError


def take_color_frame_Realsense_camera():
    camera_frame = None
    count_for_exit_while = 1
    while camera_frame is None :
        try:
            camera_frame = rospy.wait_for_message('/camera/color/image_raw', Image, timeout = 5)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
    return cv_image

#Take images from Kimera 
def take_kimera_frame_image_euroc():
    camera_frame = None
    count_for_exit_while = 1
    while camera_frame is None :
        try:
            camera_frame = rospy.wait_for_message('/cam0/image_raw', Image, timeout = 5)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
    return cv_image

def take_kimera_frame_realsense():
    camera_frame = None
    count_for_exit_while = 1
    while camera_frame is None :
        try: 
            camera_frame = rospy.wait_for_message('/camera/infra1/image_rect_raw', Image, timeout = 5)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
        except:
            rospy.loginfo('Unable to reach the topic /camera/infra1/image_rect_raw. Try to connect again')
           
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
    return cv_image
    




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

def publish_increasing_decreasing_yaw(val):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/change_yaw', Int32, latch=True, queue_size=1)  
    message = Int32()
    message.data = val
    pub.publish(message)

def publish_take_off(val):
#publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/take_off', Int32, latch=True, queue_size=1)  
    message = Int32()
    message.data = val
    pub.publish(message)




# #Subscribe to compressed images 
# def take_voxl_frame_compressed():
#       voxl_frame= rospy.wait_for_message("/stereo/left/compressed",
#                CompressedImage, timeout = 2)
      
#        #### direct conversion to CV2 ####
#           np_arr = np.fromstring(ros_data.data, np.uint8)
#           image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
     
