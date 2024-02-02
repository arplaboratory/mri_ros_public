#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
from operator import itemgetter
from random import randint
import random
from utilities.geometry import point_from_world_to_body, rotation_from_BF_to_GF, generate_ellipsoids, evaluate_drone_goal_distance, dist_between_points
from heuristics_with_voxels import cost_to_go
from threading import Thread
from heuristics_with_voxels import segment_cost, path_cost, distance_drone_goal, replan_horizon_goal_position
from nav_msgs.msg import Odometry
#from keyboard_inputs_for_voxl import KEY_pressed
from geometry_msgs.msg import Point
from sensor_msgs.msg  import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Header, Bool, Float32, Int32
from rrt import RRT
from utilities.plotting import Plot
from utilities.plotting_v2 import Plot_v2
from utilities.geometry import euler_from_quaternion, align_drone_yaw_to_next_goal, find_goal_hor_proj
from utilities.keyboard_input import set_goal
from utilities.publishers import publish_APVI_ON, publish_final_goal_reached, publish_path_array_to_goal, publish_final_goal_position
# import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import numpy as np
import time
import rospy


class RRTStar(RRT):
    def __init__(self, X, Q, x_drone, obstacles,  max_samples,horizon_ray, r, prc, rewire_count = None):
        """
        RRT* Search
        :param X: Search Space
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        :param rewire_count: number of nearby vertices to rewire
        """
        super().__init__(X, Q,x_drone, obstacles, max_samples,horizon_ray, r, prc)
        self.rewire_count = rewire_count if rewire_count is not None else 0
        #Subscriber 
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size = rospy.get_param("~queue_size", 1))
        self.surface_pc_sub = rospy.Subscriber("/voxblox_node/surface_pointcloud", PointCloud2, self.surface_pc_callback, queue_size = rospy.get_param("~queue_size", 1))
        self.manouver_flag_sub = rospy.Subscriber('/rrt/planner_paused',  Bool, self.flag_manouver_callback, queue_size = rospy.get_param("~queue_size", 1))
        self.exit_planner_flag_sub = rospy.Subscriber('/rrt/exit_search_mode', Bool, self.exit_flag_callback, queue_size = rospy.get_param("~queue_size", 1))
        self.exit_planner_flag_sub = rospy.Subscriber('/rqt_input/start_APVI_task', Bool, self.start_APVI_mode_callback, queue_size = rospy.get_param("~queue_size", 1))
        
        #Import Planner Final Goal 
        scenario = rospy.get_param("tele_control_params/scenario")
        if (scenario==1 or scenario==2):
            self.P1_final_goal_x_coo_init= rospy.get_param("tele_control_params/final_goal_sim/P1/x")
            self.P1_final_goal_y_coo_init = rospy.get_param("tele_control_params/final_goal_sim/P1/y")
        else:
            self.P1_final_goal_x_coo_init = rospy.get_param("tele_control_params/final_goal/P1/x")
            self.P1_final_goal_y_coo_init = rospy.get_param("tele_control_params/final_goal/P1/y")
        self.P1_final_goal_x_coo = self.P1_final_goal_x_coo_init
        self.P1_final_goal_y_coo = self.P1_final_goal_y_coo_init
        

        self.c_best = float('inf')  # length of best solution thus far
        self.c_near = []
        self.quad_pos = []
        self.quad_euler = []
        self.yaw_des = []
        self.x_drone = [] #drne discretized position in the planner used to update it
        self.x_near_BF = []
        self.x_goal_BF = [] #goal in the drone horizon
        self.x_goal_GF = [] #goalin drone horizon in GF
        self.goal_coo_GF = [] #Final goal coordinate in GF
        self.x_goal_GF_old = (0.0, 0.0)
        self.x_goal_final_BF= []
        self.path_GF = []
        self.mid_goal_counter_update = 0
        self.drone_next_step_projection = (0.0, 0.0)
        self.drone_height = 0.0
        self.condition = 0 
        self.counter_pub_path = 0
        self.counter_update = 0
        self.init = True
        self.flag_update_horizon = False
        self.horizon_goal_reached = False 
        self.flag_inside_manouver = False
        self.exit_planner = False
        self.final_goal = False
        self.counter_start_threading = 0
        self.counter_callback = 0

    
    def odom_callback(self, odom_msg):
        self.quad_pos = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)
        self.quad_height = odom_msg.pose.pose.position.z
        orientation_q = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.quad_euler = (roll, pitch, yaw)
        
        #print("[odom_callback] self.quad_pos: {}".format(self.quad_pos))
        if (self.condition == 0):
            self.waiting_for_goal()
        elif (self.condition == 1):
            self.planning_in_horizon()
        else:
            #Updating the planner horizon following the tree
            self.update_planner()
            print("[ODOM] Plese click in start task to activate the planner")
            #update_horizon()
        time.sleep(0.05)
    

    def start_APVI_mode_callback(self, value):
        if (value.data and self.counter_callback == 0):

            #Read the updated parameters from planner_utils.cpp in drone_teleoperation given the casuality of the target
            # time.sleep(0.2)
            # scenario = rospy.get_param("tele_control_params/scenario")
            # if (scenario==1 or scenario == 2):
            #     self.P1_final_goal_x_coo = rospy.get_param("tele_control_params/final_goal_sim/P1/x")
            #     self.P1_final_goal_y_coo = rospy.get_param("tele_control_params/final_goal_sim/P1/y")
            # else:
            #     self.P1_final_goal_x_coo = rospy.get_param("tele_control_params/final_goal/P1/x")
            #     self.P1_final_goal_y_coo = rospy.get_param("tele_control_params/final_goal/P1/y")
            # self.goal_coo_GF   = (float(self.P1_final_goal_x_coo),float(self.P1_final_goal_y_coo))
            self.counter_callback = 1
            self.final_goal = True
            
        print("Final Goal New param:  self.goal_coo_GF {}".format( self.goal_coo_GF))  
        time.sleep(2.0)   

    def surface_pc_callback(self, pc_msg):
        pointcloud = PointCloud2()
        surface_pc = []  
        for p in pc2.read_points(pc_msg, field_names = ("x", "y", "z"), skip_nans=True):
            surface_pc.append([p[0],p[1],p[2]])
        self.X.surface_pc = surface_pc
        return
    
    def flag_manouver_callback(self, flag):
        self.flag_inside_manouver = flag.data
    
    def exit_flag_callback(self, flag):
        self.exit_planner = flag.data
        if (self.exit_planner):
            for i in range(0, 100):
                print("Flag to exit rrt received")
                flag = False
                goal_reached = True
                publish_APVI_ON(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                publish_final_goal_reached(goal_reached)
                time.sleep(0.01)
            self.clear_all_the_trees()
            self.add_tree()
            #The first vertex and edge of the new tree is the drone position
            x_drone_BF = (0.0,0.0)
            self.add_vertex(0,x_drone_BF)
            self.add_edge(0,x_drone_BF, None)
            self.flag_update_horizon = True
            self.horizon_goal_reached = False
            self.init = False
            self.condition = 0
            self.counter_pub_path = 0
            self.counter_callback = 0
            self.final_goal = False
            self.waiting_for_goal()



    def waiting_for_goal(self):
        """
        Waiting fo the goal to be selected by the user
        """
        #print("[waiting_for_goal] self.x_drone: {}".format(self.x_drone))
        # print("[waiting_for_goal] self.quad_pos: {}".format(self.quad_pos))
        # if (self.final_goal == False):
        #     #This if permits to take the last updated odometry when a previous final goals is reached 
        #     print("[waiting_for_goal] self.x_drone inside if: {}".format(self.x_drone))
        #     self.final_goal, self.goal_coo_GF = set_goal()
        #     return
        
        # final_goal = True
      
        #check if the flag from the rqt enter APVI mode becomes true 
        if (self.final_goal):
            self.x_drone = (self.quad_pos[0], self.quad_pos[1])
            self.X.drone_GF = (self.quad_pos[0], self.quad_pos[1])
            drone_hor_goal_distance = evaluate_drone_goal_distance(self.x_drone, self.x_goal_GF_old)
            scenario = rospy.get_param("tele_control_params/scenario")
            random_float = random.random()
            
            #This part of code define where the final goal should appear during the task depeending if the 
            #system is in simulation or in real world. 
            # The final target location is decided before starting the planner and the coordinates uploaded in the serve as 
            # rosparam . Planner_Utils.cpp will get this coordinates from the serve to update the rviz/holo visualization. 

            #If the x_drone is positive, then the target is gonna appear with x negative (in the other semi-half of the plane)
            #If x_drone <0, then x_target >0
            #In addition, depending a random number, y_target could be bigger or lower then zero to add some randomness. 
            #In the real case,when x_target<0 and  y_target>0, the coordinates is decreased of 0.3, to avoid the drone proceding in the 
            #vicon less area.
            self.P1_final_goal_x_coo = self.P1_final_goal_x_coo_init
            self.P1_final_goal_y_coo = self.P1_final_goal_y_coo_init
            P1_final_goal_x_coo_inv =  self.P1_final_goal_x_coo 

            self.goal_coo_GF   = (float(self.P1_final_goal_x_coo),float(self.P1_final_goal_y_coo))
            if (self.quad_pos[0] < 0):
                if (self.goal_coo_GF[0] < 0):
                    P1_final_goal_x_coo_inv = -1*self.goal_coo_GF[0] 
                P1_final_goal_x_coo_inv = 0.7
                # In real exp the y_marker can appear in P1+0.2 (if P1 positive) or P1-0.3 (if P1 negative)
                # If random > 0.5 marker will be y_negative 
                # If random < 0.5 marker will be y_postive
                if (random_float > 0.5 and self.P1_final_goal_y_coo > 0):
                    self.P1_final_goal_y_coo = -1*self.P1_final_goal_y_coo - 0.5  
                elif (random_float > 0.5 and self.P1_final_goal_y_coo < 0):
                    self.P1_final_goal_y_coo = self.P1_final_goal_y_coo - 0.5
                else:
                    if (random_float <= 0.5 and self.P1_final_goal_y_coo > 0):
                        self.P1_final_goal_y_coo = self.P1_final_goal_y_coo + 0.1
                    elif(random_float <= 0.5 and self.P1_final_goal_y_coo < 0):
                        self.P1_final_goal_y_coo = -1*self.P1_final_goal_y_coo + 0.1

                #In dimulation the modification does not take effect
                if(scenario==1 or scenario ==2):
                    self.P1_final_goal_y_coo = self.P1_final_goal_y_coo_init
                self.goal_coo_GF   = (float(P1_final_goal_x_coo_inv),float(self.P1_final_goal_y_coo))
            else:
                #If x_robot > 0, the traget should appear in x < 0
                if (self.goal_coo_GF[0] > 0):
                    P1_final_goal_x_coo_inv = -1*self.goal_coo_GF[0]
                
                #if random float > 0.5, y_marker will be negative
                if (random_float > 0.5 and self.P1_final_goal_y_coo > 0):
                    self.P1_final_goal_y_coo = -1*self.P1_final_goal_y_coo - 0.25
                elif(random_float > 0.5 and self.P1_final_goal_y_coo < 0):
                    self.P1_final_goal_y_coo = self.P1_final_goal_y_coo - 0.25
                else:
                    if (random_float <= 0.5 and self.P1_final_goal_y_coo > 0):
                        self.P1_final_goal_y_coo = self.P1_final_goal_y_coo 
                    elif(random_float <= 0.5 and self.P1_final_goal_y_coo < 0):
                        self.P1_final_goal_y_coo = -1*self.P1_final_goal_y_coo 

                if(scenario==1 or scenario ==2):
                    self.P1_final_goal_y_coo = -1*self.P1_final_goal_y_coo_init
                self.goal_coo_GF   = (float(P1_final_goal_x_coo_inv),float(self.P1_final_goal_y_coo))
                 
            
            
            if (scenario==1 or scenario == 2):
                self.goal_coo_GF = (float(P1_final_goal_x_coo_inv),float(self.P1_final_goal_y_coo))
                rospy.set_param('tele_control_params/final_goal_sim/P1/x',  self.goal_coo_GF[0])   
                rospy.set_param('tele_control_params/final_goal_sim/P1/y',  self.goal_coo_GF[1])   
            else:
                rospy.set_param('tele_control_params/final_goal/P1/x', self.goal_coo_GF[0])   
                rospy.set_param('tele_control_params/final_goal/P1/y',  self.goal_coo_GF[1])   
            
           
            time.sleep(0.2)
            
            for i in range(0, 10):
               

                print("Goal Selected: x {}, y: {}".format(self.goal_coo_GF[0],  self.goal_coo_GF[1]))
                flag = True
                publish_APVI_ON(flag)
                publish_final_goal_reached(goal_reached = False)
                publish_final_goal_position(self.goal_coo_GF)
                time.sleep(0.1)

            #Yaw of the drne (selected a priori then imported from real world)
            self.yaw_des = align_drone_yaw_to_next_goal(self.goal_coo_GF, self.quad_pos) 
            self.clear_all_the_trees()
            self.add_tree()
            x_drone_BF = (0.0, 0.0)
            self.add_vertex(0, x_drone_BF)
            self.add_edge(0, x_drone_BF, None)
            self.X.yaw = self.yaw_des
            self.yaw_drone = self.yaw_des
            self.x_goal_GF_old = self.x_drone
            self.init = True
            self.condition = 2 #update planner final goal
            self.ccounter_pub_path = 0
            self.exit_planner = False
            self.final_goal = False
            self.counter_start_threading = 0
            print("self.X.yaw: {}".format(self.X.yaw))
            print("self.goal_coo_GF: {}".format(self.goal_coo_GF))
            print("self.x_drone: {}".format(self.x_drone))
            return
        
        self.counter_start_threading = self.counter_start_threading + 1

    def planning_in_horizon(self):
        self.x_drone = (self.quad_pos[0], self.quad_pos[1])
        self.X.drone_GF = (self.quad_pos[0], self.quad_pos[1])
        self.drone_height = self.quad_height
        self.X.drone_height =  self.drone_height

        #check distance for the goal 
        drone_hor_goal_distance = evaluate_drone_goal_distance(self.x_drone, self.x_goal_GF_old)
        drone_final_goal_distance = evaluate_drone_goal_distance(self.x_drone, self.goal_coo_GF)
        print("drone_hor_goal_distance: ", drone_hor_goal_distance)
        if (drone_hor_goal_distance < 0.20 and self.flag_inside_manouver == False):
            self.x_drone = self.x_goal_GF_old
            self.X.yaw = self.yaw_drone

            if (drone_final_goal_distance < 0.35): #Horizon goal reached
                #publish goal reached 
                for i in range(0, 100):
                    publish_final_goal_reached(goal_reached = True)
                    flag = False
                    publish_APVI_ON(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                    print("[RRT] Final Goal Reached")
                    time.sleep(0.01)
                self.horizon_goal_reached = False
                self.init = False
                self.condition = 0
                self.exit_planner = False
                self.counter_callback = 0
                return

            self.horizon_goal_reached = False
            self.init = False
            self.condition = 2
            self.counter_pub_path = 0
            
        else:
            print("Waiting to reach the Horizon Goal")
            if (drone_final_goal_distance < 0.35): #Horizon goal reached
                self.x_drone = self.x_goal_GF_old
                self.X.yaw = self.yaw_drone

                #publish goal reached 
                for i in range(0, 100):
                    publish_final_goal_reached(goal_reached = True)
                    flag = False
                    publish_APVI_ON(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                    print("[RRT] Final Goal Reached")
                    time.sleep(0.01)
                self.horizon_goal_reached = False
                self.init = False
                self.condition = 0
                self.exit_planner = False
                self.counter_callback = 0
                return

            
            if (self.counter_pub_path < 15):
                publish_path_array_to_goal(self.path_GF)
                self.counter_pub_path = self.counter_pub_path + 1

        return
    
    def update_planner(self):
        #self.X.drone_GF = self.x_drone #For the clear obstacle function
        self.X.yaw = self.yaw_des
        #Rotate Goal Position in Body frame 
        print("[update_planner] self.x_drone: {}".format(self.x_drone))
        print("[update_planner] self.goal_coo_GF: {}".format(self.goal_coo_GF))
        x_goal_final_BF = point_from_world_to_body(self.goal_coo_GF, self.x_drone, self.yaw_des)   #--> o

        #Find Final Goal Projection on drone horizon 
        self.x_goal_BF, self.x_goal_GF, self.yaw_des = find_goal_hor_proj(self.X, self.x_drone, self.yaw_des, self.goal_coo_GF, x_goal_final_BF,self.x_goal_GF_old, self.horizon_ray, self.init)
        #rotate goal BF in GF --> self.x_goal_GF is the projection of the final goal on drone horizon 
        if (self.init == True):
            self.x_goal_GF =  rotation_from_BF_to_GF(self.x_goal_BF , self.x_drone, self.yaw_des)
       
        print("[update_planner] self.x_goal_GF: {}".format(self.x_goal_GF))
        # self.yaw_des = align_drone_yaw_to_next_goal(self.x_goal_GF, self.x_drone)
        # self.x_goal_BF  = point_from_world_to_body(self.x_goal_GF , self.x_goal_GF_old, self.yaw_des)  #!!! Per
        self.yaw_drone = self.yaw_des
        self.X.yaw = self.yaw_des
        for q in self.Q:  # iterate over different edge lengths
            #Iterate q[1] time in on the edges of length q[0]
            for i in range(int(q[1])):  # iterate over number of edges of given length to add
                print("self.horizon_goal_reached: ", self.horizon_goal_reached)
                if (self.horizon_goal_reached == True):
                    break
                #publish_obstacles_positions(self.obstacles, self.X.safety_bounds)
                #Find a new random vertice and the closer vertice liyng on the existing tree to the new vertex randomly choosen
                x_new_BF, x_nearest_BF = self.new_and_near_with_voxels(0, q)
    
                if x_new_BF is None:
                    continue
                # get nearby vertices and cost-to-come
                x_drone_BF = (0.0,0.0)
                L_near = self.get_nearby_vertices(0, x_drone_BF, x_new_BF)
                # check nearby vertices for total cost and connect shortest valid edge
                self.connect_shortest_valid(0, x_new_BF, L_near,  self.x_goal_BF) #x_goal_final_BF
                #Check if the new vertex has been added to the main tree
                if x_new_BF in self.trees[0].E:
                    # rewire tree
                    self.rewire(0, x_new_BF, L_near)
                
                solution = self.check_solution()
                #If the drone reach the goal in the horizon, x_drone= x_goal_horizon and a new x_goal is settled on the surface
                self.path_GF = None
                path = None
               
                if solution[0]:
                    path = solution[1]
                    print("[update_planner] path: ", path)
                    self.path_GF = []
                    for i in path:
                        i =  rotation_from_BF_to_GF(i, self.x_drone, self.yaw_des)
                        self.path_GF.append(i)
                        
        
                    #check the disatnce between the drone and the final goal
                    # self.x_drone =  self.x_goal_final_GF
                    # distance = distance_drone_goal(self.quad_pos, self.goal_coo_GF)
                        
                    # if (distance > self.horizon_ray):
                    #     #convert  self.x_goal_BF in GF frame 
                    #     self.final_goal_reached = False
                    # else:
                    #     #goal Reached 
                    #     print("[RRT] Final Goal Reached")
                    #     self.final_goal_reached = True

                    self.x_goal_GF_old = rotation_from_BF_to_GF(self.x_goal_BF, self.x_drone,  self.yaw_des)

                    # Reset and reinitialize a new tree
                    self.clear_all_the_trees()
                    self.add_tree()
                    #The first vertex and edge of the new tree is the drone position
                    self.add_vertex(0,x_drone_BF)
                    self.add_edge(0,x_drone_BF, None)
                    self.mid_goal_counter_update = 0
                    self.flag_update_horizon = True

                    print("The Planner reached the Horizon Goal. Waiting")
                    print("########################################################################")
                    self.horizon_goal_reached = True
                    self.init = False
                    self.condition = 1
                    self.final_goal = False
                    #publish the path array to the admiyttance control script 
                    publish_path_array_to_goal(self.path_GF)
                    # if (goal_reached == False): #Final goal reached not horizon goal
                    #     # plt.ion()  # enable interactivity
                    #     # plot.update_plot(self.X,self.x_goal_GF_old, self.x_drone, self.trees, L_near, x_new_BF, self.x_near_BF, self.obstacles, path_GF, self.horizon_ray,  self.yaw_drone, goal_reached, self.X.safety_bounds, self.folder_plot_number )
                     
                    #     self.x_drone = self.x_goal_GF_old
                    self.counter_update = 0
                    break
        

            self.counter_update = self.counter_update + 1
            if (self.counter_update > 40):
                self.horizon_goal_reached = False
                self.init = False
                #self.waiting_for_goal()
                self.condition = 0
              
                break 
            
           
            

       
       
           

         

    def rewire(self, tree, x_new, L_near):
        """+
        This function permits to rewires the vertices that are in the closer set L to the new selected vertex in the free space. 
        In poractice, if the new vertex is added to the tree, all the previous closer the neighbours vertices present inthe set L are reaximed,
        to see if exist a new connection that minimize the overall cost.
        This becuse the new added node open new possibilties of connection in the graph 
        
        Rewire tree to shorten edges if possible
        Only rewires vertices according to rewire count
        :param tree: int, tree to rewire
        :param x_new: tuple, newly added vertex
        :param L_near: list of nearby vertices used to rewire
        :return:
        """

        drone_BF = (0.0, 0.0)
        #Iterating on the all close vertices in the L set
        for c_near, x_near in L_near:
            #Cost between x_init and nearest vertices to the new one lying on the tree
            curr_cost = path_cost(self.trees[tree].E, drone_BF, x_near)
            # cost between x_init and the new vertex plus  the segment from the near to the new vertex
            tent_cost = path_cost(self.trees[tree].E, drone_BF, x_new) + segment_cost(x_near, x_new)
            # if the cost to the new vertex is less than the cost tomthe nearest vertex and the edge connecting the new vertex to the 
            # closer one is ciollision free, then add the new vertex in the tree as child of x_near
            
            if tent_cost < curr_cost and self.X.collision_free(x_near, x_new, self.r):
                # print("[rewire] curr_cost: " , curr_cost)
                # print("[rewire] tent_cost: " , tent_cost)
                print("[rewire] tent_cost < curr_cost")
                self.trees[tree].E[x_near] = x_new

    def get_nearby_vertices(self, tree, x_init, x_new):
        """
        Get nearby vertices to new vertex and their associated path costs from the root of tree
        as if new vertex is connected to each one separately.

        :param tree: tree in which to search
        :param x_init: starting vertex used to calculate path cost
        :param x_new: vertex around which to find nearby vertices
        :return: list of nearby vertices and their costs, sorted in ascending order by cost

        : self.trees[tree].E Edges of the built tree
        """

        
        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree)) #Now is in BF
        #For all the near vertices lying on the tree to the new vertex i evaluate the distance between the initial vertex and the nearest one and the distance of the new segment (from the nearest to the new)
        # In this way i have the length of the entire path, taking into account also the new vertices.
        L_near = []

        for x_near in X_near:
            #Check tra i vertice che sono alla frontiera nel tree. --> in iterazioni sucessive alla prima è presnete ancge l'ultimo vertice aggiunto x_new
            L_near.append([path_cost(self.trees[tree].E, x_init, x_near) + segment_cost(x_near, x_new), x_near]) 
        L_near.sort(key=itemgetter(0))


        return L_near
    
    def current_rewire_count(self, tree):
        """
        Return rewire count
        :param tree: tree being rewired
        :return: rewire count
        """
        # if no rewire count specified, set rewire count to be all vertices
        if self.rewire_count is None:
            return self.trees[tree].V_count

        # max valid rewire count
        return min(self.trees[tree].V_count, self.rewire_count)


    def connect_shortest_valid(self, tree, x_new, L_near, x_goal_final_BF):
        """
        Connect to nearest vertex that has an unobstructed path
        :param tree: int, tree being added to
        :param x_new: tuple, vertex being added
        :param L_near: list of nearby vertices
        """
        self.c_near = []
        self.x_near = []
        x_near_ = 0
        
        c_near_ = []
        self.x_near_BF = []
        
        # c_near_list = []
        # distance_x_near_final_goal_list = []
        cost_function = []
        # check nearby vertices for total cost and connect shortest valid edge
        print("Lnear: ", L_near)
        for c_near, x_near in L_near:
            #c_near_list.append(c_near)

            #Evaluate distance 
           # distance_x_near_final_goal_list.append(cost_to_go(x_near, x_goal_final_BF))
            cost = 0
            if (self.connect_to_point(tree, x_near, x_new)):
                cost = 0.1
            else:
                cost = 10.0


            cost_function.append(0.3*c_near + 0.7*cost_to_go(x_near, x_goal_final_BF) + cost)
        #search the elemnt taht minimize the cost function
        m = min(cost_function)
        min_index = cost_function.index(m)
        
        c_near = L_near[min_index][0]
        x_near = L_near[min_index][1]
        self.c_near = c_near
        self.x_near_BF = x_near


  

    def rrt_star(self):
        """
        Based on algorithm found in: Incremental Sampling-based Algorithms for Optimal Motion Planning
        http://roboticsproceedings.org/rss06/p34.pdf
        :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        """

        rospy.init_node('RRT_Teleoperation', anonymous=True)
        #Initialize tree in BF 
        x_drone_BF = (0.0, 0.0)
        self.add_vertex(0, x_drone_BF)
        self.add_edge(0, x_drone_BF, None)

        counter = 0
        x = []
        y = []
        
        init = True  
        path_GF = None
        path = None
        goal_reached = False #flag to final goal reached 
        waiting_next_final_goal = False #The script wait tye user to select a goal 


        rospy.spin()
