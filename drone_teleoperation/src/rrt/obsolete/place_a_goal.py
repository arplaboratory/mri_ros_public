


#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:09:43 2020

@author: lucamora
"""


""" 
Script that recognize if a key is pressed and it send a message to the drone,
in particular to the cpp script requiring the drone to stop or to start along the PV arrays
"""


import rospy 

import numpy as np

from scipy import ndimage
import time

from threading import Thread

from ros_data import publish_desired_mode
from ros_data_voxl import obtain_drone_odometry, publish_path_array_to_goal, publish_final_goal_reached

class KEY_pressed:
    def __init__(self):
        self.key_pressed = 0
        self.counter_key = 0
        self.counter_key_old = 0
        self.flag = False
        self.user_control = False 
        self.case = 0

def listen_keyboard_input(key, condition):
    while (key.counter_key < condition):
        
        try:
            key.key_pressed = raw_input("[KEYBOARD INPUT] CASE SWITCHER: H -> Hovering, T -> Trajectory Tracker, M -> Manual Control")
        except:
            print("[KEYBOARD INPUT] PLEASE SELECT ONE OF THE LISTED OPTIONS")

        if ( key.key_pressed == ''):
            print("[KEYBOARD INPUT] PLEASE SELECT ONE OF THE LISTED OPTIONS: {}".format(key.key_pressed))
            continue

        
        if ( key.key_pressed == 'h'):
            print("[KEYBOARD INPUT] HOVERING: {}".format(key.key_pressed))
            key.case = 1
            

        if ( key.key_pressed == 't'):
            print("[KEYBOARD INPUT] START TRAJECTORY TRACKING: {}".format(key.key_pressed))
            key.case = 2
           

        if ( key.key_pressed == 'm'):
            print("[KEYBOARD INPUT] MANUAL TELEOPERATION: {}".format(key.key_pressed))
            key.case = 3
        
        

	   
        publish_desired_mode(key.case)
            
         
        
        key.counter_key = key.counter_key + 1
        key.flag = True
    return 




def listener():
    rospy.init_node('Keyboard_Inputs_to_dron_control', anonymous=True)
    
    key = KEY_pressed()
   
    counter_key_old = 0
    count_end = 0
    condition = 10000000000000000
    start = 0
    end = 0
   
    #Listen Keyboard input 
    count = 0
    flag = False
    while(count < condition):
        if count == 1:
             #create Thread to listen Keyboard input
            try:
                thread = Thread(target = listen_keyboard_input, args = (key, condition))
                thread.daemon = True
                thread.start()
                #thread.join()
            except:
                print("Impossible to create the thread to listen the user input")

        start = time.time()

        #receive Flag = True if the drone is under user control 
       
        # print("############################################",  key.user_control  )

        time.sleep(0.5)
      
        count = count + 1

   




if __name__ == '__main__':
    listener()
  