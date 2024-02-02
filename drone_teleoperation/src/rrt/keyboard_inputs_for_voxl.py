#!/usr/bin/env python3
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
from utilities.publishers import publish_desired_mode, publish_goals, publish_increasing_decreasing_yaw
from threading import Thread
import signal


class KEY_pressed:
    def __init__(self):
        self.key_pressed = 0
        self.counter_key = 0
        self.counter_key_old = 0
        self.flag = False
        self.user_control = False 
        self.case = 0
        self.count_goal = 0

def listen_keyboard_input(key, condition):
    while (key.counter_key < condition):
        
        try:
            print("#####################################################################################")
            print("To change the yaw press the key A to move left, D to move right")
            key.key_pressed = input("[KEYBOARD INPUT] CASE SWITCHER: H -> Hovering, T -> Trajectory Tracker, M -> Manual Control")
            print(" float(key.key_pressed): ",  float(key.key_pressed))
        except:
            print("[KEYBOARD INPUT] PLEASE SELECT ONE OF THE LISTED OPTIONS")

        if ( str(key.key_pressed) == ''):
            print("[KEYBOARD INPUT] PLEASE SELECT ONE OF THE LISTED OPTIONS: {}".format(key.key_pressed))
            continue

        
        if ( str(key.key_pressed) == 'h'):
            print("[KEYBOARD INPUT] FPVI: {}".format(key.key_pressed))
            key.case = 1
            for i in range (0, 100):
                publish_desired_mode(key.case)
                time.sleep(0.01)
                
            

        if ( str(key.key_pressed) == 't'):
            print("[KEYBOARD INPUT] EXIT APVI : {}".format(key.key_pressed))
            key.case = 2
            for i in range (0, 100):
                publish_desired_mode(key.case)
                time.sleep(0.01)
        
        if ( str(key.key_pressed) == 'g'):
            print("[KEYBOARD INPUT] Publish Intermediate Goals for APVI : {}".format(key.key_pressed))
            
            val = False
            if (key.count_goal == 0):
                val = True
                key.count_goal = key.count_goal + 1
            else:
                 val = False
                 key.count_goal = 0
            for i in range (0, 100):
                publish_goals(val)
                time.sleep(0.01)
            
        if ( str(key.key_pressed) == 'a'):
            print("[KEYBOARD INPUT] Increasing yaw of 10 degrees: {}".format(key.key_pressed))
            value = 0
            publish_increasing_decreasing_yaw(value)
            

        if ( str(key.key_pressed) == 'd'):
            print("[KEYBOARD INPUT] Decreasing yaw of 10 degrees: {}".format(key.key_pressed))
            value = 1
            publish_increasing_decreasing_yaw(value)
 
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
    try:
        thread = Thread(target = listen_keyboard_input, args=(key, condition))
        thread.daemon = True
        thread.start()
        #thread.join()
    except:
        print("Impossible to create the thread to listen the user input")

    #listen_keyboard_input(key, condition)
    
    rospy.spin()


if __name__ == '__main__':
    listener()
  