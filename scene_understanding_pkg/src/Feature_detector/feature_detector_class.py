#!/usr/bin/env python
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import cv2
import math
import time
from ros_data import take_color_frame_Realsense_camera, publish_matching_flag

#from solar_project.src.solar_project.RGB_visual_ros_sim_reordered import drone



class feature_detector_class:
    """
    Class for feature detection and matchin between different frames 
    """
    def __init__(self):
        
        #Initialize Detection Parameters 
        self.baseline_image_gray = []
        self.knn_matches = []
        self.good_matches = []
        self.ratio_thresh = 0.7
        self.threshold_gm = 70  #Se numero cresce rendo difficile match e acquisisco piu data nella mesh
        self.image_minibatch = []
        self.image_minibatch_size = 0
        self.prev_image = []
        self.desc_prev_image = []
        self.desc_curr_image = []
        self.theta_x = 0.0
        self.theta_y = 0.0
        self.theta_z = 0.0
        self.theta_x_prec = 0.0 
        self.theta_y_prec = 0.0 
        self.theta_z_prec = 0.0 
        self.R = 0
        self.T = 0
        self.same_image = False
        self.init = True
        pass

    def load_baseline_image(self, path):
        self.baseline_image_gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    
    def feature_detector(self, curr_frame):
        if  self.baseline_image_gray is None or curr_frame is None:
            print('Could not open or find the images!')
            exit(0)

        if (self.init == True or self.same_image == False):
            #take baseline image 
            self.baseline_image_gray = curr_frame
            #add baseline image to the minibatch of images 
            self.add_minibatch(self.baseline_image_gray )
            self.init = False
        

        #-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        minHessian = 400
        detector = cv2.xfeatures2d_SURF.create(hessianThreshold=minHessian)
        #Define a vector of matches
        self.good_matches_minibatch = []
        #check similarities with all the previous baseline images stored in the minibatch
        #this permits to recognize if the drone has scanned before this area and avoiding to have new scans
        counter = 0
        for image in self.image_minibatch:
            baseline_image = image
            keypoints1, descriptors1 = detector.detectAndCompute(baseline_image , None)
            keypoints2, descriptors2 = detector.detectAndCompute(curr_frame, None)
            
            

            #-- Step 2: Matching descriptor vectors with a FLANN based matcher
            # Since SURF is a floating-point descriptor NORM_L2 is used
            matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
            #Matrix Output [m,n]
            knn_matches = matcher.knnMatch(descriptors1, descriptors2, 2)
            #-- Filter matches using the Lowe's ratio test
        
            self.good_matches = []
        
            for m,n in knn_matches:
                if m.distance < self.ratio_thresh * n.distance:
                    self.good_matches.append(m)
            
            self.good_matches_minibatch.append(self.good_matches)
            counter = counter + 1
        #Evaluate rotation and translation from the fundamentakl and essential matrix
        #if (len(self.prev_image) > 0):
        #    self.obtain_rotation_and_translation_matrices(curr_frame)
        #Draw the match with the image that have more matches in common stored in the minibatch
        max_value_list = []
        for match in self.good_matches_minibatch:
            max_value = len(match)
            max_value_list.append(max_value)
        
     
        #print("Len Minibatch: " ,len(self.good_matches_minibatch) )
        #find the best match 
        max_value = max(max_value_list)
        max_index = max_value_list.index(max_value) 
        image_to_show = self.image_minibatch[max_index]
       #The rotation error will be not considered if a match is found in the minibatch 
        matching_batch_flag = False
        if (max_index < len(max_value_list) -1):
           matching_batch_flag = True
              
       
        #-- Draw matches
        #img_matches = np.empty((max(self.baseline_image_gray.shape[0], curr_frame.shape[0]), self.baseline_image_gray.shape[1]+curr_frame.shape[1], 3), dtype=np.uint8)
        
        #cv2.drawMatches(self.baseline_image_gray, keypoints1, curr_frame, keypoints2, self.good_matches, img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        #img_matches = np.empty((max(image_to_show.shape[0], curr_frame.shape[0]), image_to_show.shape[1]+curr_frame.shape[1], 3), dtype=np.uint8)
        #cv2.drawMatches(image_to_show, keypoints1, curr_frame, keypoints2, self.good_matches, img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
       
        #-- Show detected matches
        
        #cv2.imshow('Good Matches', img_matches)
       # cv2.waitKey(1)
        
        self.check_similarities(matching_batch_flag)
        self.theta_x_prec = self.theta_x
        self.theta_y_prec = self.theta_y
        self.theta_z_prec = self.theta_z
        matching_batch_flag = False

    def check_similarities(self,matching_batch_flag):
        '''
        This function check the similarities between the baseline frame and  new frame.
        Two frames are defined similar if the number of good_matches is above a certain threshold.
        Output: flag true or false depending if the images are considered similar or not
        '''
        x_rot_diff = abs(self.theta_x -  self.theta_x_prec)
        y_rot_diff = abs(self.theta_y -  self.theta_y_prec)
        z_rot_diff = abs(self.theta_z -  self.theta_z_prec)
        
        #print("x_rot_diff: ", x_rot_diff)
        #print("y_rot_diff: ", y_rot_diff)
        #print("z_rot_diff: ", z_rot_diff)
        
        #10 degree in rad : 0.17
        
        num_gm = len(self.good_matches)
        #self.threshold_gm = 70

        if (num_gm > self.threshold_gm):
            #if (x_rot_diff < 0.10 and y_rot_diff < 0.10 and z_rot_diff < 0.10 or matching_batch_flag == True):
            self.same_image = True
            print("Match")
            #else:
            #    self.same_image = False
            #    #if not match sent flag to c++ code
            #    print("Not Match")

        else:
            self.same_image = False
            #if not match sent flag to c++ code
            print("Not Match")
        
        publish_matching_flag(self.same_image)

    def add_minibatch(self, baseline_image):
        self.image_minibatch.append(baseline_image)
        self.image_minibatch_size = len(self.image_minibatch)
        if (len(self.image_minibatch)> 20):
            self.image_minibatch.pop(0)

    def obtain_rotation_and_translation_matrices(self, curr_frame):
        
        fx = 385.097656
        cx = 320.960998
        fy = fx 
        cy = 239.11816406
        K = np.float64([[fx, 0, cx], 
                [0, fy, cy], 
                [0, 0, 1]])

        D = np.float64([0.0, 0.0, 0.0, 0.0])
        
        detector = cv2.ORB_create(nfeatures=25000, edgeThreshold=15, patchSize=125, nlevels=32, 
                     fastThreshold=20, scaleFactor=1.2, WTA_K=2,
                     scoreType=cv2.ORB_HARRIS_SCORE, firstLevel=0)

        # find the keypoints and descriptors with ORB
        kp1, des1 = detector.detectAndCompute(curr_frame,None)
        kp2, des2 = detector.detectAndCompute(self.prev_image,None)
        
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        matches = bf.match(des1,des2)
        kp1_match = np.array([kp1[mat.queryIdx].pt for mat in matches])
        kp2_match = np.array([kp2[mat.trainIdx].pt for mat in matches])
        
        kp1_match_ud = cv2.undistortPoints(np.expand_dims(kp1_match,axis=1),K,D)
        kp2_match_ud = cv2.undistortPoints(np.expand_dims(kp2_match,axis=1),K,D)
        
        E, mask_e = cv2.findEssentialMat(kp1_match_ud, kp2_match_ud, focal=1.0, pp=(0., 0.), 
                                       method=cv2.RANSAC, prob=0.999, threshold=0.001)
        
        #print ("Essential matrix: used ",np.sum(mask_e) ," of total ",len(matches),"matches")

        points, R, t, mask_RP = cv2.recoverPose(E, kp1_match_ud, kp2_match_ud, mask=mask_e)
       # print("points:",points,"\trecover pose mask:",np.sum(mask_RP!=0))
        self.R = R
        self.T = t
        #print("R:",R,"t:",t.T)

        #Obtain angles from rotational matrix
        self.theta_x = math.atan2(self.R[2][1], self.R[2][2])
        #print("theta_x: ", theta_x)
        
        val = math.sqrt(math.pow(self.R[2][1], 2) + math.pow(self.R[2][2], 2))
        self.theta_y = math.atan2(-self.R[2][0], val)
        #print("theta_y: ", theta_y)
        self.theta_z = math.atan2(self.R[1][0], self.R[0][0])
       # print("theta_z: ", theta_z)
        





      