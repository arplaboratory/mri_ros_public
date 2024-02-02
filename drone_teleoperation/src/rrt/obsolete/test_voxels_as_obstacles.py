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
import math
import random

import numpy as np
from geometry_msgs.msg import Point
from scipy import ndimage
import time

from ros_data import obtain_voxels_positions

class RRT_search:
    def __init__(self):
        self.sample_points_list = []
        self.n_surface_points_in_range_list = []
        self.zero_value_positions_list = []
     

def sample_random_point_inside_horizon(r, drone):
    circle_x = drone[0]
    circle_y = drone[1]

    # random angle
    alpha = 2*math.pi* random.random()
    # random radius
    r = r * math.sqrt(random.random())
    x = r * math.cos(alpha) + circle_x
    y = r * math.sin(alpha) + circle_y
    point = (x, y)
    return point



def check_voxels_in_horizon(rrt_search):
    surface_pointcloud, terminal_flag = obtain_voxels_positions() #ruotare i punt nel BF in futuro 
    print("surface_pointcloud: ", len(surface_pointcloud))
    #define imaginary drone position 
    drone_pos_GF = (0.0, 12.0)
    #define a radius 
    r = 1

    #select random points insde the circle radius
    point = sample_random_point_inside_horizon(r, drone_pos_GF) 
    rrt_search.sample_points_list.append(point)
    #search for points inside the radius 
    counter = 0
    for i in range(0, len(surface_pointcloud)):
        #evaluate disatnce form drone position 
        a = pow(point[0] - surface_pointcloud[i][0], 2)
        b = pow(point[1] - surface_pointcloud[i][1], 2)
        d = math.sqrt(a + b)

        if (d < 1):
            counter = counter + 1
          
    
    rrt_search.n_surface_points_in_range_list.append(counter)
    return terminal_flag


def listener():
    rospy.init_node('check_voxels_as_obstacles', anonymous=True)
  
   
    rrt_search = RRT_search()
    counter_key_old = 0
    count_end = 0
    condition = 10000000000000
    start = 0
    end = 0
   
    #Listen Keyboard input 
    count = 0
    flag = False
    check_counter = 0
    while(count < condition):
        terminal_flag = check_voxels_in_horizon(rrt_search)
        if (check_counter > 10):
            zero_values = True
            #check which point is better between the one sampled 
            while (zero_values == True):
                #find the index of the zero value inside
                try: 
                    pos = rrt_search.n_surface_points_in_range_list.index(0)
                    print("pos: ", pos)
                   
                except: 
                    zero_values = False
                    rrt_search.n_surface_points_in_range_list = []
                    rrt_search.sample_points_list = []
                    check_counter = 0
                    continue
                #find the vertex position that correspond to the index found below
                rrt_search.zero_value_positions_list.append(rrt_search.sample_points_list[pos])
                #delete the elemnt from the list
                del rrt_search.sample_points_list[pos]
                del rrt_search.n_surface_points_in_range_list[pos]
                
            print("number of points obstacle free: ", len( rrt_search.zero_value_positions_list))
            print("Free Points list: ",   rrt_search.zero_value_positions_list)
            
        if (terminal_flag == True):
            break
        time.sleep(0.3)
        count = count + 1
        check_counter = check_counter + 1

      




if __name__ == '__main__':
    listener()
  