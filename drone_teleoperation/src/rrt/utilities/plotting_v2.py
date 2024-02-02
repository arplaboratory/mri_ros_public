# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from drawnow import drawnow
import numpy as np
import os
from math import pi
from utilities.geometry import rotation_from_BF_to_GF, rotate_ellipsoids_from_BF_to_GF


class Plot_v2():
    def __init__(self, filename):
        """
        Create a plot
        :param filename: filename
        """
        self.filename = "/home/arpl/luca_ws/src/scene_understanding_pkg/src/rrt-algorithms/src/rrt/Plots/" + filename + ".html"
        self.data = []
        self.layout = {'title': 'Plot',
                       'showlegend': False
                       }

        self.fig = {'data': self.data,
                    'layout': self.layout}
        
        self.X = []
        self.x_goal = []
        self.x_drone = []
        self.trees = []
        self.L_near_BF= []
        self.x_new_BF  = []      
        self.x_near_BF = []
        self.safety_bound = []

        self.x_coo_tree_line_list = []
        self.y_coo_tree_line_list = []
        self.x_coo_nearest_points = []
        self.y_coo_nearest_points = []
        self.x_coo_shortestvalid_edge_list = []
        self.y_coo_shortestvalid_edge_list = []

        self.obstacles = []
        self.x_coo_path =[]
        self.y_coo_path = []
        self.counter_plot = 0
        self.desired_folder_number = 1

        self.yaw_drone = 0.0

        self.horizon_ray = 0.0
    
    def draw_circle(self):
        theta = np.linspace( 0 , 2 * np.pi , 150 )
        radius =  self.horizon_ray
        a = self.x_drone[0] + radius * np.cos( theta )      
        b = self.x_drone[1] + radius * np.sin( theta )

        return a, b

    def make_fig(self):
        plt.axis([-2.5, 2.5, -2.5, 2.5])
        #Draw the Goal Position in the map  
        plt.scatter(self.x_goal[0], self.x_goal[1], s= 20, color =np.array([1, 0, 0]))  # I think you meant this
        #Draw the drone position ion the map 
        plt.scatter( self.x_drone[0], self.x_drone[1], s= 20, color = np.array([0, 0, 1]))  # I think you meant this
       # plt.plot( self.x_drone[0],  self.x_drone[1],  linewidth= 0.5)
        #Draw the freontiers points, closer to the new randomly sampled points 
        plt.scatter(self.x_coo_nearest_points,  self.y_coo_nearest_points, s= 5, color = np.array([0, 0, 0]))  # I think you meant this
        
        #Draw the nearest and the new points selected in the horizon with a diffrent color
        x_near_GF = rotation_from_BF_to_GF(self.x_near_BF, self.x_drone, self.yaw_drone)
        plt.scatter(x_near_GF[0], x_near_GF[1], s= 5,  color =np.array([0, 1, 0]))  # I think you meant this
        x_new_GF =  rotation_from_BF_to_GF(self.x_new_BF, self.x_drone, self.yaw_drone)
        plt.scatter(x_new_GF[0],  x_new_GF[1], s= 5, color =np.array([0, 0, 1])) 
        #Connect the shiortest valid edge 
        x_coo_shortest_valid  = [x_near_GF[0], x_new_GF[0]]
        y_coo_shortest_valid  = [x_near_GF[1], x_new_GF[1]]
        plt.plot(x_coo_shortest_valid,  y_coo_shortest_valid,  linewidth= 0.5) #the last added edge is blue 
        #plt.plot(self.x_coo_shortestvalid_edge_list,  self.y_coo_shortestvalid_edge_list,  linewidth= 0.5, color = 'black')
        a, b = self.draw_circle()
        plt.plot(a, b)
    

        
        #plt.grid(color='lightgray',linestyle='--')
        #plt.show()
    
        #Drawing obstacles 
        counter = 0
        th_length = self.safety_bound[0]
        th_height = self.safety_bound[1]
        
       
        counter = 0
        for obs in self.obstacles:
            x_min = obs[0] + th_length
            y_min = obs[1] + th_height
            x_max = obs[2] - th_length
            y_max = obs[3] - th_height
            
            ext_x = [x_min,x_max, x_max, x_min]
            ext_y = [y_min, y_min, y_max, y_max]
            plt.fill(ext_x, ext_y, color='blue')
            
            # #Drawing ellipses around obstacle
          
            # x_c, theta, semi_axes =  rotate_ellipsoids_from_BF_to_GF(self.X.C[counter], self.X.semi_axes[counter],  self.X.theta_BF[counter], self.x_drone,  self.yaw_drone)
            # u= x_c[0]    #x-position of the center
            # v= x_c[1]   #y-position of the center
            # a= semi_axes[0]   #radius on the x-axis
            # b= semi_axes[1]   #radius on the y-axis
            
            # t = np.linspace(0, 2*pi, 100)
            # plt.plot( u+a*np.cos(t) , v+b*np.sin(t) )
            counter = counter + 1

        #Drawing the optimal path 
        if (len(self.x_coo_path) > 0):
            plt.plot(self.x_coo_path, self.y_coo_path,  linewidth= 1.0, color = 'red')
        
        #Save the sequence of plots 
        try:
            path = "/home/arpl/luca_ws/DATA/TELE_CONTROL/rrt_plot/Exp_"+ str(self.desired_folder_number) 
            os.mkdir(path)
        except: 
            print("Directory exist")

        path = "/home/arpl/luca_ws/DATA/TELE_CONTROL/rrt_plot/Exp_"+ str(self.desired_folder_number) + "/path_"+str(self.counter_plot)+".png"
        plt.savefig(path)
        self.counter_plot = self.counter_plot + 1

    
    def fix_plot(self):
        plt.figure()
        plt.axis([0, 10, 0, 10])
        #Draw the Goal Position in the map  
        plt.scatter(self.x_goal[0], self.x_goal[1], s= 20, color =np.array([1, 0, 0]))  # I think you meant this
        #Draw the drone position ion the map 
        plt.scatter( self.x_drone[0], self.x_drone[1], s= 20, color = np.array([0, 0, 1]))  # I think you meant this
        #Draw the freontiers points, closer to the new randomly sampled points 
        plt.scatter(self.x_coo_nearest_points,  self.y_coo_nearest_points, s= 5, color = np.array([0, 0, 0]))  # I think you meant this
        
        #Draw the nearest and the new points selected in the horizon with a diffrent color
        x_near_GF = rotation_from_BF_to_GF(self.x_near_BF, self.x_drone, self.yaw_drone)
        plt.scatter(x_near_GF[0], x_near_GF[1], s= 5,  color =np.array([0, 1, 0]))  # I think you meant this
        x_new_GF =  rotation_from_BF_to_GF(self.x_new_BF, self.x_drone, self.yaw_drone)
        plt.scatter(x_new_GF[0],  x_new_GF[1], s= 5, color =np.array([0, 0, 1])) 
        #Connect the shiortest valid edge 
        x_coo_shortest_valid  = [x_near_GF[0], x_new_GF[0]]
        y_coo_shortest_valid  = [x_near_GF[1], x_new_GF[1]]
        plt.plot(x_coo_shortest_valid,  y_coo_shortest_valid,  linewidth= 0.5) #the last added edge is blue 
        #plt.plot(self.x_coo_shortestvalid_edge_list,  self.y_coo_shortestvalid_edge_list,  linewidth= 0.5, color = 'black')
        a, b = self.draw_circle()
        plt.plot(a, b)
    

        
        #plt.grid(color='lightgray',linestyle='--')
        #plt.show()
    
        #Drawing obstacles 
        th_length = self.X.bounds[0]
        th_height = self.X.bounds[1]
        
        counter = 0
       
        for obs in self.X.obs_visualization:
            # obs_[0] = obs[0] + th_length
            # obs_[1] = obs[1] + th_height
            # obs_[2] = obs[2] - th_length
            # obs_[3] = obs[3] - th_height

            ext_x = [obs[0] , obs[2], obs[2], obs[0]]
            ext_y = [obs[1], obs[1],  obs[3], obs[3]]
            plt.fill(ext_x, ext_y, color='blue')
            
            #Drawing ellipses around obstacle
          
            # x_c, theta, semi_axes =  rotate_ellipsoids_from_BF_to_GF(self.X.C[counter], self.X.semi_axes[counter],  self.X.theta_BF[counter], self.x_drone,  self.yaw_drone)
            # u= x_c[0]    #x-position of the center
            # v= x_c[1]   #y-position of the center
            # a= semi_axes[0]   #radius on the x-axis
            # b= semi_axes[1]   #radius on the y-axis
            
            # t = np.linspace(0, 2*pi, 100)
            # plt.plot( u+a*np.cos(t) , v+b*np.sin(t) )
            counter = counter + 1

        #Drawing the optimal path 
        if (len(self.x_coo_path) > 0):
            plt.plot(self.x_coo_path, self.y_coo_path,  linewidth= 1.0, color = 'red')
        

        plt.show(block=True)

       
    def plot_tree_2d(self):
        """
        Plot 2D trees
        :param trees: trees to plot
        """
        #iterate on the selected closer point of the tree to the new one 
        #print("len L_near: ", len(self.L_near))
        self.x_coo_nearest_points = []
        self.y_coo_nearest_points = []
        
        for ii in range(0, len(self.L_near_BF)):
           # print("x_coo = ", self.L_near[ii][1][0])
            #Rotate L_near Points in the world frame
            L_near_GF = rotation_from_BF_to_GF(self.L_near_BF[ii][1], self.x_drone, self.yaw_drone)
            self.x_coo_nearest_points.append(L_near_GF[0])
            self.y_coo_nearest_points.append(L_near_GF[1])
           
       
       
        self.x_coo_tree_line_list = []
        self.y_coo_tree_line_list = []

        for i, tree in enumerate(self.trees):
            for start, end in tree.E.items():
                if end is not None:
                    self.x_coo_tree_line_list.append(start[0])
                    self.x_coo_tree_line_list.append(end[0])
                    self.y_coo_tree_line_list.append(start[1])
                    self.y_coo_tree_line_list.append(end[1])
            
       
    def draw_obstacles(self, obstacles):
        # for ii in range(0, len(obstacles)):
         
        ext_x = [2, -2, -2, 2, 2]
        ext_y = [2, 2, -2, -2, 2]


    
    def plot_path(self, path):
        for i in path:
            #i =  rotation_from_BF_to_GF(i, self.x_drone, self.yaw_drone)
            self.x_coo_path.append(i[0])
            self.y_coo_path.append(i[1])
            
           
 


    def update_plot(self, X, x_goal, x_drone, trees, L_near_BF, x_new_BF, x_near_BF, obstacles, path, horizon, yaw_drone, goal_reached, safety_bound, counter_folder_plot):
        self.X = X
        self.x_goal = x_goal
        self.x_drone = x_drone
        self.trees = trees 
        self.L_near_BF= L_near_BF
        self.x_new_BF = x_new_BF
        self.x_near_BF = x_near_BF
        self.obstacles = obstacles
        self.horizon_ray = horizon
        self.yaw_drone = yaw_drone
        self.safety_bound = safety_bound
        self.desired_folder_number = counter_folder_plot
       
      
        if path is not None:
            self.path = path

        #Draw the tree adding to the list only the shortes valid edges
        # self.x_coo_shortestvalid_edge_list.append(self.x_near[0])
        # self.x_coo_shortestvalid_edge_list.append(self.x_new[0])
        # self.y_coo_shortestvalid_edge_list.append(self.x_near[1])
        # self.y_coo_shortestvalid_edge_list.append(self.x_new[1])


        #plot tree 
        if X.dimensions == 2:  # plot in 2D
            self.plot_tree_2d()
            if path is not None:
                self.plot_path(path)
            #self.draw_obstacles(obstacles)
        elif X.dimensions == 3:  # plot in 3D
            self.plot_tree_3d()
        else:  # can't plot in higher dimensions
            print("Cannot plot in > 3 dimensions")
        
        if goal_reached == False:
            drawnow(self.make_fig)
        else: 
            self.x_coo_path.clear()
            self.y_coo_path.clear()
            self.fix_plot()

       

