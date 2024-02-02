# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
from operator import itemgetter
from random import randint

from utilities.geometry import point_from_world_to_body, rotation_from_BF_to_GF, generate_ellipsoids, evaluate_drone_goal_distance, dist_between_points
from heuristics import cost_to_go
from threading import Thread
from heuristics import segment_cost, path_cost, distance_drone_goal, replan_horizon_goal_position
from ros_data import obtain_drone_odometry, publish_path_array_to_goal,take_flag_inside_manouver, publish_final_goal_reached, obtain_final_goal, publish_flag_to_enter_in_assistive_rrt_guidance, publish_obstacles_positions, take_flag_to_exit_rrt, publish_final_goal_position
from keyboard_inputs_for_voxl import KEY_pressed
from geometry_msgs.msg import Point
from rrt import RRT
# from utilities.plotting import Plot
# from utilities.plotting_v2 import Plot_v2
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import math
import numpy as np
import time
import rospy


class RRTStar(RRT):
    def __init__(self, X, Q, x_drone, x_goal_final_GF, obstacles,  max_samples,horizon_ray,  r,  prc=0.01, rewire_count=None):
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
        super().__init__(X, Q, x_drone, x_goal_final_GF, obstacles, max_samples,horizon_ray, r, prc)
        self.rewire_count = rewire_count if rewire_count is not None else 0
        self.c_best = float('inf')  # length of best solution thus far
        self.c_near = []
        self.x_near_BF = []
        self.x_goal_BF = [] #goal in the drone horizon
        self.x_goal_GF = [] #goalin drone horizon in GF
        self.x_goal_GF_old = (0.0, 0.0)
        self.x_goal_final_BF= []
        self.mid_goal_counter_update = 0
        self.drone_next_step_projection = (0.0, 0.0)
        self.drone_height = 0.0
        self.folder_plot_number= 1

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

        #print("tree: ",  tree)
        
        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree)) #Now is in BF
        
            
        #     print("self.trees[tree].E: ", self.trees[tree].E)
        #     print("segment_cost: ",  segment_cost(x_near, x_new))
        #     print("path cost: ",  path_cost(self.trees[tree].E, x_init, x_near) + segment_cost(x_near, x_new), x_near)
        #For all the near vertices lying on the tree to the new vertex i evaluate the distance between the initial vertex and the nearest one and the distance of the new segment (from the nearest to the new)
        # In this way i have the length of the entire path, taking into account also the new vertices.
        L_near = []
        for x_near in X_near:
           #print("x_near: ",  x_near)
            #Check tra i vertice che sono alla frontiera nel tree. --> in iterazioni sucessive alla prima è presnete ancge l'ultimo vertice aggiunto x_new
           L_near.append([path_cost(self.trees[tree].E, x_init, x_near) + segment_cost(x_near, x_new), x_near]) 
           #print("[get_nearby_vertices] x_near in X_near: ",x_near)
        #L_near = [(path_cost(self.trees[tree].E, x_init, x_near) + segment_cost(x_near, x_new), x_near) for
         #         x_near in X_near]
        L_near.sort(key=itemgetter(0))
        #print("L_near: ", L_near)
        # noinspection PyTypeChecker
      

        return L_near

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

    def connect_shortest_valid(self, tree, x_new, L_near, x_goal_final_BF):
        """
        Connect to nearest vertex that has an unobstructed path
        :param tree: int, tree being added to
        :param x_new: tuple, vertex being added
        :param L_near: list of nearby vertices
        """
        self.c_near = []
        self.x_near_BF = []
        
        # c_near_list = []
        # distance_x_near_final_goal_list = []
        cost_function = []
        # check nearby vertices for total cost and connect shortest valid edge
        print("x_goal_final_BF: ", x_goal_final_BF)
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
            #break

          

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

    

    # def find_goal_projection_on_drone_horizon(self): #!!!! TO evaluate in the body frame
    #     #Define the interesection bewteen the drone goal line and the surface of the drone horizon
    #     y_goal_coo = self.x_goal_final_GF[1]
    #     x_goal_coo = self.x_goal_final_GF[0]
    #     x_drone_coo = self.x_drone[0]
    #     y_drone_coo = self.x_drone[1]
    
    #     m = 0
    #     if (x_goal_coo - x_drone_coo != 0):
    #         m = (y_goal_coo - y_drone_coo)/(x_goal_coo - x_drone_coo)
    #     else:
    #         m = (y_goal_coo - y_drone_coo)/0.01
    
    #     #obtain the angle theta between the x_axis GF and the angle between the drone and the goal
    #     theta = math.atan(m)
    
    #     #Knowing the horizon radius evaluateing the projection of the intersection between the horizn and the ine on the x and y axis
    #     x_rel_goal = 0.0
    #     y_rel_goal = 0.0
        
    #     if (x_drone_coo > x_goal_coo):
    #         x_rel_goal = self.horizon_ray*math.cos(theta) - x_drone_coo #il segno della somma va cambiato a seconda che il gol si trovi su asse x
    #     else:                                                           #positivo onegativo del drone. Se in body frame non necessario se yaw orientato verso il target
    #         x_rel_goal = self.horizon_ray*math.cos(theta) + x_drone_coo

    #     if (y_drone_coo > y_goal_coo):
    #         y_rel_goal = self.horizon_ray*math.sin(theta) - y_drone_coo
    #     else:
    #         y_rel_goal = self.horizon_ray*math.sin(theta) + y_drone_coo

    #     horizon_goal = (x_rel_goal, y_rel_goal)
             
    #     #check if horizon goal is inside an obstacle, otherwise replan another point on the circunferece until it is outside the obstacle
    #     print("self.X.obstacle_free(horizon_goal): ", self.X.obstacle_free(horizon_goal))
    #     if (self.X.obstacle_free(horizon_goal) == False):
    #         horizon_goal = replan_horizon_goal_position(self.X, self.horizon_ray, theta, self.x_drone, self.x_goal_final_GF, self.x_goal_GF_old, self.mid_goal_counter_update, self.yaw_drone)
    #        #Fare un check ance sulla posizione dei due goal per evitare che abbiano la stessa x GF

    #     return horizon_goal

    
    def find_goal_projection_on_drone_BF_horizon(self, final_goal_BF, init):
        x_goal_BF = final_goal_BF[0]
        y_goal_BF = final_goal_BF[1]
        
        # print("final_goal_BF: ", final_goal_BF)

        #Evaluate the equation of the line passing through the drone BF and the goal point
        
        if (x_goal_BF <= 0.01 and x_goal_BF >= 0):
            x_goal_BF = 0.00001
        
        if (x_goal_BF >= -0.01 and x_goal_BF < 0):
            x_goal_BF = -0.00001
        
        m = y_goal_BF/x_goal_BF
       
    
        theta = math.atan(m)
        x_rel_goal_BF = self.horizon_ray*math.cos(theta)
        y_rel_goal_BF = self.horizon_ray*math.sin(theta)
        # print("theta m: ", theta)
        # print("self.yaw_drone: ", self.yaw_drone)
       
       
        # if (-math.pi/2 - theta >-0.1 and -math.pi/2 - theta < 0.1):  # gola direction parallel to the y axis from the positive side
        #     #check if the distance with the goalis decreasing otherwise change the sign 
        #     y_rel_goal_BF = -1*y_rel_goal_BF
     
        
        rel_goal_BF = (x_rel_goal_BF, y_rel_goal_BF)

        #Evaluate the distance between the horizon goal and the goal in the body frame 
        dist_horizon_goal_final_goal_BF = dist_between_points(rel_goal_BF,final_goal_BF)

         #Evaluate the distance between the drone and the goal in the body frame 
        dist_drone_final_goal_BF = dist_between_points(0,final_goal_BF)
        flag = False
        if (dist_horizon_goal_final_goal_BF > dist_drone_final_goal_BF + self.horizon_ray/2):
            y_rel_goal_BF = m*(-1*x_rel_goal_BF) #Projecting the value on the line but with opposite sign, to avoi that the drone start to gomfar away from the onstacle surface 
            rel_goal_BF = (-1*x_rel_goal_BF, y_rel_goal_BF)
            theta = -np.pi + theta
            flag = True #if true the amgle must be only increased
            # print("@@@@@@@@@ rel_goal_BF: ", rel_goal_BF)
            # print("@@@@@@@@@ theta: ", theta)
        else:
            flag = False

       
        #Evaluate in GF for voxels 
        rel_goal_GF = rotation_from_BF_to_GF(rel_goal_BF, self.x_drone,  self.yaw_drone)
        #rel_goal_BF = replan_horizon_goal_position(self.X, self.horizon_ray, theta, self.x_drone, final_goal_BF,  self.x_goal_GF_old, self.mid_goal_counter_update, self.yaw_drone, init)
        print("[find_goal_projection_on_drone_BF_horizon] rel_goal_BF: ", rel_goal_BF, " rel_goal_GF: ", rel_goal_GF)
        #check if it is the first time that the drone impact an obstcle
        # if (self.X.obstacle_hit == False):
        #     self.X.init_replanning = True
        # else:
        #     self.X.init_replanning = False
        
       
        if (self.X.obstacle_free(rel_goal_BF) == False):

            print("Point inside obstacle")
            rel_goal_BF = replan_horizon_goal_position(self.X, self.horizon_ray, theta, self.x_drone, rel_goal_BF, final_goal_BF,  self.x_goal_GF_old, self.mid_goal_counter_update, self.yaw_drone, init, flag)
            #self.X.obstacle_hit = True
        # else:
        #     self.X.obstacle_hit = False
        return rel_goal_BF 


    def align_drone_yaw_to_next_goal(self, x_goal_GF):
        #Evaluate the angle between the goal in the BF and the and the drone body frame 
        a = x_goal_GF[1] - self.x_drone[1]
        b = x_goal_GF[0] - self.x_drone[0]
        d = math.sqrt(pow(a,2) + pow(b,2))
        if (a > 0):
            a = b/d
            yaw_des = math.acos(a)
        else:
            yaw_des = -1*math.acos(b/d)
        
        print("yaw_des: ", yaw_des)
        print("yaw drone: ", self.yaw_drone)
        return yaw_des
   
      


    def clear_ellipses_data(self):
        self.X.C = []
        self.X.semi_axes = []
        self.X.theta_BF = []
    


    def define_goal(self):
        x_goal = 0.0
        y_goal = 0.0 
        flag1 = False
        flag2 = False
        goal_defined = False
        try:
            flag1 = True
            x_goal = input('Please Insert the coordinates of the Goal in GF. First X coo\n')
          
        except:
            print("[KEYBOARD INPUT] Impossible to insert x coordinate")
        
        try:
            flag2 = True
            y_goal = input('Please Insert Y coo\n')
        except:
            print("[KEYBOARD INPUT] Impossible to insert Y coordinate")
        
        self.x_goal_final_GF = (float(x_goal),float(y_goal))

        for i in range(0, 10):
            print("Goal Selected: x {}, y: {}".format(self.x_goal_final_GF[0],  self.x_goal_final_GF[1] ))
            flag = True
            publish_flag_to_enter_in_assistive_rrt_guidance(flag)
            publish_final_goal_reached(goal_reached = False)
            publish_final_goal_position(self.x_goal_final_GF)
            time.sleep(0.1)
        
        if ( flag1 == True and flag1 == True):
            goal_defined = True

        return goal_defined


    def rrt_star(self, desired_folder_number):
        """
        Based on algorithm found in: Incremental Sampling-based Algorithms for Optimal Motion Planning
        http://roboticsproceedings.org/rss06/p34.pdf
        :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        """
        # self.add_vertex(0, self.x_drone)
        # self.add_edge(0, self.x_drone, None)
        rospy.init_node('RRT_Teleoperation', anonymous=True)
        #Initialize tree in BF 
        x_drone_BF = (0.0, 0.0)
        self.add_vertex(0, x_drone_BF)
        self.add_edge(0, x_drone_BF, None)
         
        #Initialize class to take the selected goal by the user
        key = KEY_pressed()

        counter = 0
        x = []
        y = []
         # run the animation
        # plot = Plot_v2("rrt_star_2d")
        # fig = plt.figure()  # make a figure    
        horizon_goal_reached = False 
        flag_update_horizon = False
        init = True  
        path_GF = None
        path = None
        goal_reached = False #flag to final goal reached 
        waiting_next_final_goal = False #The script wait tye user to select a goal 
        new_goal_published = False
        
    
        condition = 1000000
        counter_loop = 0
        drone_odometry = (0.0, 0.0)

        # traversable = self.X.voxels_obstacle_free(self.x_drone,0.5)
        # print("traversable: ", traversable)
        self.folder_plot_number = desired_folder_number
        while counter_loop < condition:
            #Publish Obstacles Positions 
            # if()
            # publish_obstacles_positions(self.obstacles, self.X.safety_bounds)
             #Obtain drone odometry at each iteration 
           
            
            
            if (waiting_next_final_goal or init):
                goal_defined = self.define_goal()
                # if (waiting_next_final_goal):
                #     break
                if (goal_defined == True):
                    drone, yaw = obtain_drone_odometry() #in GF
            
                    self.x_drone = (drone.x, drone.y)
                    self.drone_height = 1 #drone.z
                    self.X.drone_height =  self.drone_height
                    drone_odometry = self.x_drone

                     #Yaw of the drne (selected a priori then imported from real world)
                    self.yaw_drone = self.align_drone_yaw_to_next_goal(self.x_goal_final_GF) 
                    horizon_goal_reached = False 
                    flag_update_horizon = False
                    path_GF = None
                    path = None
                    goal_reached = False #flag to final goal reached 
                    waiting_next_final_goal = False #The script wait tye user to select a goal 
                    new_goal_published = False
                    x_drone_BF = (0.0, 0.0)
                    self.add_vertex(0, x_drone_BF)
                    self.add_edge(0, x_drone_BF, None)
                    self.X.drone_GF = self.x_drone #For the clear obstacle function
                    self.X.yaw = self.yaw_drone
                    self.x_goal_GF_old = self.x_drone

                    
                else:
                    if (counter_loop > 10000):
                        return
                    counter_loop = counter_loop + 1
                    #print("counter_loop: ", counter_loop)
                    continue
        
         
           
        
            if (horizon_goal_reached == True and init == False):
               
                drone, yaw = obtain_drone_odometry() #in GF
            
                self.x_drone = (drone.x, drone.y)
                self.drone_height = 1 #drone.z
                self.X.drone_height =  self.drone_height
                drone_odometry = self.x_drone
                    

                drone_hor_goal_distance = evaluate_drone_goal_distance(self.x_drone, self.x_goal_GF_old)
                
                print("Distance from Horizon goal: ", drone_hor_goal_distance)
                in_bypass_manouver = take_flag_inside_manouver()
                if (drone_hor_goal_distance < 0.20 and in_bypass_manouver == False):
                 
                    self.x_drone = self.x_goal_GF_old
                    self.X.drone_GF = self.x_drone #For the clear obstacle function
                    self.X.yaw = self.yaw_drone
                    if (goal_reached): #Horizon goal reached
                        #publish goal reached 
                        for i in range(0, 100):
                            publish_final_goal_reached(goal_reached)
                            flag = False
                            publish_flag_to_enter_in_assistive_rrt_guidance(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                            time.sleep(0.01)
                        waiting_next_final_goal = True
                        continue

                   

                    waiting_next_final_goal = False
                    horizon_goal_reached = False
                    init = False
                else:
                    print("Waiting for the user to reach the Horizon Goal before proceed")
                    waiting_next_final_goal = False
                    publish_path_array_to_goal(path_GF)
                     
                  
                    a = pow(drone.x - self.x_goal_final_GF[0], 2)
                    b = pow(drone.y - self.x_goal_final_GF[1], 2)

                    distance_drone_and_goal = np.sqrt(a + b)
                    if take_flag_to_exit_rrt() or distance_drone_and_goal < 0.3:
                        for i in range(0, 100):
                            print("Falg to exit rrt received")
                            flag = False
                            goal_reached = True
                            publish_flag_to_enter_in_assistive_rrt_guidance(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                            publish_final_goal_reached(goal_reached)
                            
                            time.sleep(0.01)
                            waiting_next_final_goal = True
                            
                        self.clear_all_the_trees()
                        self.add_tree()
                        #The first vertex and edge of the new tree is the drone position
                        self.add_vertex(0,x_drone_BF)
                        self.add_edge(0,x_drone_BF, None)
                        self.mid_goal_counter_update = 0
                        flag_update_horizon = True
                        
                        horizon_goal_reached = True
                        #publish the path array to the admiyttance control script 
                        publish_path_array_to_goal(path_GF)
                         #Publish Obstacles Positions 
                        publish_obstacles_positions(self.obstacles, self.X.safety_bounds)
                        waiting_next_final_goal = True
                 
                    continue
           
            self.X.drone_GF = self.x_drone #For the clear obstacle function
            self.X.yaw = self.yaw_drone
           
            #Rotate Goal Position in Body frame 
            
            x_goal_final_BF = point_from_world_to_body(self.x_goal_final_GF, self.x_drone, self.yaw_drone)   #--> ok
            print("x_goal_final_BF: ", x_goal_final_BF)
            #Set New Horizon Goal
                  # Find x_goal_B projection on the drone Horizon 
            self.x_goal_BF = self.find_goal_projection_on_drone_BF_horizon(x_goal_final_BF, init)
            #rotate goal BF in GF
            if (init == True):
                self.x_goal_GF =  rotation_from_BF_to_GF(self.x_goal_BF , self.x_drone, self.yaw_drone)
            else:
                self.x_goal_GF =  rotation_from_BF_to_GF(self.x_goal_BF , self.x_goal_GF_old, self.yaw_drone)
            
            print("[rrt_star] drone_GF: ", self.x_drone)
            print("[rrt_star] Horizon Goal GF: ", self.x_goal_GF)
            #align the yaw of teh drone to the next horizon goal 
            self.yaw_drone = self.align_drone_yaw_to_next_goal(self.x_goal_GF)
            self.x_goal_BF  = point_from_world_to_body(self.x_goal_GF , self.x_goal_GF_old, self.yaw_drone)  #!!! Per
            
            #self.x_goal = self.find_goal_projection_on_drone_horizon()
            
            horizon_goal_reached = False
            flag_update_horizon = False
            init = False
              
            
            # self.clear_ellipses_data()
                
            # C_BF, semi_axes_BF, theta_BF = generate_ellipsoids(self.obstacles, drone_odometry, self.yaw_drone)
            # # pass the information to the search space class
            # self.X.C = C_BF
            # self.X.semi_axes = semi_axes_BF
            # self.X.theta_BF = theta_BF 
            # self.X.drone_GF = self.x_drone
            # self.X.yaw = self.yaw_drone
            # select next Horizon Goal

                    
                 
            #This procedure is exact the same 
            for q in self.Q:  # iterate over different edge lengths
                if (horizon_goal_reached == True):
                    break
              #Iterate q[1] time in on the edges of length q[0]
               
                for i in range(int(q[1])):  # iterate over number of edges of given length to add
                    publish_obstacles_positions(self.obstacles, self.X.safety_bounds)
                    #Obtain drone odometry at each iteration                   
                    
                    
                    
                    #Find a new random vertice and the closer vertice liyng on the existing tree to the new vertex randomly choosen
                    x_new_BF, x_nearest_BF = self.new_and_near(0, q)
                    # print("x_new: ", x_new)
                    # print("x_nearest: ", x_nearest)
                   
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
                    path_GF = None
                    path = None
                   
                    if solution[0]:
                        path = solution[1]
                        path_GF = []
                        for i in path:
                            # drone, yaw = obtain_drone_odometry() #in GF
                            # drone_GF = (drone.x, drone.y)
                            i =  rotation_from_BF_to_GF(i, self.x_drone, self.yaw_drone)
                            path_GF.append(i)
                        
                        print("path_GF: ", path_GF)
                        #check the disatnce between the drone and the final goal
                        # self.x_drone =  self.x_goal_final_GF
                        distance = distance_drone_goal(self.x_drone, self.x_goal_final_GF)
                        
                        if (distance > self.horizon_ray):
                            #convert  self.x_goal_BF in GF frame 
                            goal_reached = False
                        else:
                            #goal Reached 
                            print("[RRT] Final Goal Reached")
                            goal_reached = True
                         
                        self.x_goal_GF_old = rotation_from_BF_to_GF(self.x_goal_BF, self.x_drone,  self.yaw_drone)
                        #Place the root position of the next tree in the horizon goal
                        
                        
                        
                        # Reset and reinitialize a new tree
                        self.clear_all_the_trees()
                        self.add_tree()
                        #The first vertex and edge of the new tree is the drone position
                        self.add_vertex(0,x_drone_BF)
                        self.add_edge(0,x_drone_BF, None)
                        self.mid_goal_counter_update = 0
                        flag_update_horizon = True
                           
                        print("Horizon Goal Reached. Selecting the next goal")
                        print("########################################################################")
                        horizon_goal_reached = True
                        #publish the path array to the admiyttance control script 
                        publish_path_array_to_goal(path_GF)
                         #Publish Obstacles Positions 
                        publish_obstacles_positions(self.obstacles, self.X.safety_bounds)


                        if (goal_reached == False): #Final goal reached not horizon goal
                            # plt.ion()  # enable interactivity
                            # plot.update_plot(self.X,self.x_goal_GF_old, self.x_drone, self.trees, L_near, x_new_BF, self.x_near_BF, self.obstacles, path_GF, self.horizon_ray,  self.yaw_drone, goal_reached, self.X.safety_bounds, self.folder_plot_number )
                          
                            self.x_drone = self.x_goal_GF_old
                        break
                    

                   
                    if take_flag_to_exit_rrt():
                        for i in range(0, 100):
                            self.x_goal_GF_old = rotation_from_BF_to_GF(self.x_goal_BF, self.x_drone,  self.yaw_drone)                       
                            # Reset and reinitialize a new tree
                            # self.clear_all_the_trees()
                            # self.add_tree()
                            # #The first vertex and edge of the new tree is the drone position
                            # self.add_vertex(0,x_drone_BF)
                            # self.add_edge(0,x_drone_BF, None)
                            self.mid_goal_counter_update = 0
                            flag = False
                            publish_flag_to_enter_in_assistive_rrt_guidance(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                            goal_reached = True
                            publish_final_goal_reached(goal_reached)
                            #time.sleep(0.01)
                            waiting_next_final_goal = True
                           
                        break
                    goal_reached = False
                    
                    print("[RRT STAR] Drone GF: x: {}, y: {}".format(x, y))
                    # plt.ion()  # enable interactivity
                    # plot.update_plot(self.X, self.x_goal_GF_old, self.x_drone, self.trees, L_near, x_new_BF, self.x_near_BF, self.obstacles, path_GF, self.horizon_ray, self.yaw_drone, goal_reached, self.X.safety_bounds, self.folder_plot_number )
                  

            counter = counter + 1

            if (counter > 100 and horizon_goal_reached == False):
                #If the code exited from the maximum number of iterations, re enter in the next goal waiting and 
                #ask to the tele-operation script to come back to the normal admittance mode (mission->state = 2)
                for i in range(0, 100):
                    flag = False
                    publish_flag_to_enter_in_assistive_rrt_guidance(flag)  #This put at false the flag to reenter in this modality. SO the drone will wait that a new goal will be given before re-entering in the manual assistive modality
                    goal_reached = True
                    publish_final_goal_reached(goal_reached)
                    print("[RRT] Maximum number of iteration reached. No solution Found. Come back to assistive rrt guidance")
                    waiting_next_final_goal = True

                

            