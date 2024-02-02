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
from scene_understanding_pkg_msgs.msg  import RRTPathPoints2D
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



#Publish to RRT script the goal selected by the user 
def publish_rrt_goal(goal):
    pub =  rospy.Publisher('/rrt/final_goal', Point, latch=True, queue_size=10)  
    message = Point()
    message = goal
    pub.publish(message)



