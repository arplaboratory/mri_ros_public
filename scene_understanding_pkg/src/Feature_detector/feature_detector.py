#!/usr/bin/python
from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse

import matplotlib
import operator

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import rospy 

import numpy as np

from scipy import ndimage
import cv2
import time
import os
import math

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread

from ros_data import take_color_frame_Realsense_camera , take_kimera_frame_image_euroc, take_kimera_frame_realsense, take_voxl3_frame
from feature_detector_class import feature_detector_class

def resize_image(img):
    scale_percent = 40 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
      
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized


def listener():
    global image_saving_path 
    rospy.init_node('Feature_detector', anonymous=True)
    
    FD = feature_detector_class()
    path_baseline_image = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/Feature_detector/images/baseline_image.png"
    FD.load_baseline_image(path_baseline_image)
    
    #if false false take data from euroc
    from_voxl3 = True
    from_kimera_realsense = False
    condition = 10000
    count = 0
    while(count < condition):
        if (from_voxl3 == True and from_kimera_realsense ==  False):
            gray_frame = take_voxl3_frame()
            #gray_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            gray_frame = resize_image(gray_frame)
        elif (from_voxl3 == False and from_kimera_realsense ==  True):
            #Take realsense camera infrared images
            gray_frame = take_kimera_frame_realsense()
            gray_frame = resize_image(gray_frame)
        else:
            gray_frame = take_kimera_frame_image_euroc()
            gray_frame = resize_image(gray_frame)
            #gray_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        
        cv2.imshow('gray', gray_frame)
        cv2.waitKey(1)

        FD.feature_detector(gray_frame)
        #FD.check_similarities()

        FD.prev_image = gray_frame
        count = count +1 
    


if __name__ == '__main__':
    listener()


