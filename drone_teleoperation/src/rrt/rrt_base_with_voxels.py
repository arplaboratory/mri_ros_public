import random

import numpy as np

from tree import Tree
from utilities.geometry import steer, point_from_world_to_body, rotation_from_BF_to_GF
from heuristics_with_voxels import distance_drone_steered_point
import math

class RRTBase(object):
    def __init__(self, X, Q, x_drone, x_goal_final_GF,obstacles, max_samples,horizon_ray, r, prc=0.01):
        """
        Template RRT planner
        :param X: Search Space
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        self.X = X
        self.samples_taken = 0
        self.max_samples = max_samples
        self.Q = Q
        self.r = r
        self.prc = prc
        self.x_drone = x_drone
        self.yaw_drone = 0 #initialized in the function or obtianed by the drobne
        self.x_goal_final_GF = x_goal_final_GF
        self.obstacles = obstacles
        self.horizon_ray = horizon_ray
        self.trees = []  # list of all trees
        self.add_tree()  # add initial tree
       

    def clear_all_the_trees(self):
        self.trees.clear()

    def add_tree(self):
        """
        Create an empty tree and add to trees
        """
        self.trees.append(Tree(self.X))

    def add_vertex(self, tree, v):
        """
        Add vertex to corresponding tree
        :param tree: int, tree to which to add vertex
        :param v: tuple, vertex to add
        """
        self.trees[tree].V.insert(0, v + v, v)
        self.trees[tree].V_count += 1  # increment number of vertices in tree
        self.samples_taken += 1  # increment number of samples taken

    def add_edge(self, tree, child, parent):
        """
        Add edge to corresponding tree
        :param tree: int, tree to which to add vertex
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.trees[tree].E[child] = parent

    def nearby(self, tree, x, n):
        """
        Return nearby vertices
        :param tree: int, tree being searched
        :param x: tuple, vertex around which searching
        :param n: int, max number of neighbors to return
        :return: list of nearby vertices
        """
        #between the nearest n = 10, return the one that has ethe minimuma angular difference with the goal and the father of the nearest self.trees[tree].E[nearest_father])
        return self.trees[tree].V.nearest(x, num_results=n, objects="raw")
        #nearest_father = self.trees[tree].V.nearest(x, num_results=n, objects="raw")

    def get_nearest(self, tree, x):
        """
        Return vertex nearest to x
        :param tree: int, tree being searched
        :param x: tuple, vertex around which searching
        :return: tuple, nearest vertex to x
        """
        return next(self.nearby(tree, x, 1))

    def new_and_near_with_voxels(self, tree, q):
        """
        Return a new steered vertex and the vertex in tree that is nearest
        :param tree: int, tree being searched
        :param q: length of edge when steering
        :return: vertex, new steered vertex, vertex, nearest vertex in tree to new vertex
        """
        #find a random point inside the drone circle:
       # x_rand = self.sample_random_point_in_horizon()

        x_rand_BF = self.sample_random_point_in_horizon_BF()
        #Convert in GF for evaluateion with the pc voxels
        x_rand_GF =rotation_from_BF_to_GF(x_rand_BF, self.x_drone, self.yaw_drone)
  
        traversable = self.X.voxels_obstacle_free(x_rand_GF,0.2)
        print("[NEW and NEAR] traversable: ", traversable)
        if (traversable == False):
            x_new_BF = None 
            x_nearest_BF = None
            return x_new_BF, x_nearest_BF

     
       # x_nearest = self.get_nearest(tree, x_rand)
       
        x_nearest_BF = self.get_nearest(tree, x_rand_BF)
       
        # print("x_rand: ", x_rand)
        # print("x_nearest: ", x_nearest)
        #Bound point must be searched inside the diameter of the drone 
        #Search steer(x_nearest, x_rand, q[0]) inside the circunference. The goal in the steer function is teh final goal position projected
        #on the drone circunference
        
        #after use bound_point to check if the point is inside the safe space.
        x_new_BF = steer(x_nearest_BF, x_rand_BF, q[0])
       
        #Check if the steered point is inside the drone horizon
        #self.bound_point_in_drone_horizon(steered_point)
        x_new_BF = self.bound_point(steer(x_nearest_BF, x_rand_BF, q[0])) #--> steer permits to find a point on the line between x_rand and x_nearest. the point is at the edge length from x_rnearest 
        
        #print("[new_and_near] x_new_BF: ", x_new_BF)
        # check if new point is in X_free and not already in V
        if not self.trees[0].V.count(x_new_BF) == 0 or not self.X.obstacle_free(x_new_BF):
            #print("[new_and_near] New point is not obstacle free or alredy in V")
            return None, None 
        self.samples_taken += 1
        return x_new_BF, x_nearest_BF
    
    
    def connect_to_point(self, tree, x_a, x_b):
        """
        Connect vertex x_a in tree to vertex x_b
        :param tree: int, tree to which to add edge
        :param x_a: tuple, vertex
        :param x_b: tuple, vertex
        :return: bool, True if able to add edge, False if prohibited by an obstacle
        """
        if self.trees[tree].V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            return True
        return False

    def can_connect_to_goal(self, tree):
        """
        Check if the goal can be connected to the graph
        :param tree: rtree of all Vertices
        :return: True if can be added, False otherwise
        """
       
        x_nearest = self.get_nearest(tree, self.x_goal_BF)
        if self.x_goal_BF in self.trees[tree].E: 
            if (x_nearest in self.trees[tree].E[self.x_goal_BF]):
            # tree is already connected to goal using nearest vertex
                return True
       
        if self.X.collision_free(x_nearest, self.x_goal_BF, self.r):  # check if obstacle-free
            return True
        return False

    def get_path(self):
        """
        Return path through tree from start to goal
        :return: path if possible, None otherwise
        """
        if self.can_connect_to_goal(0):
            print("Can connect to goal")
            self.connect_to_goal(0)
            x_drone_BF = (0.0,0.0)
            return self.reconstruct_path(0, x_drone_BF, self.x_goal_BF)
        print("Could not connect to goal")
        return None

    def connect_to_goal(self, tree):
        """
        Connect x_goal to graph
        (does not check if this should be possible, for that use: can_connect_to_goal)
        :param tree: rtree of all Vertices
        """
        x_nearest = self.get_nearest(tree, self.x_goal_BF)
        self.trees[tree].E[self.x_goal_BF] = x_nearest
        # print("x_nearest: ", x_nearest)
        # print("self.x_goal_BF: ", self.x_goal_BF)

    def reconstruct_path(self, tree, x_init, x_goal):
        """
        Reconstruct path from start to goal
        :param tree: int, tree in which to find path
        :param x_init: tuple, starting vertex
        :param x_goal: tuple, ending vertex
        :return: sequence of vertices from start to goal
        """
        path = [x_goal]
        current = x_goal
        if x_init == x_goal:
            return path
        while not self.trees[tree].E[current] == x_init:
            path.append(self.trees[tree].E[current])
            current = self.trees[tree].E[current]
        path.append(x_init)
        path.reverse()
        return path

    def check_solution(self):
        # probabilistically check if solution found
        if self.prc and random.random() < self.prc:
            print("Checking if can connect to goal at", str(self.samples_taken), "samples")
            path = self.get_path()
            if path is not None:
                return True, path
        # check if can connect to goal after generating max_samples
        if self.samples_taken >= self.max_samples:
            return True, self.get_path()
        return False, None

    def sample_random_point_in_horizon(self):
        # radius of the circle
        circle_r = self.horizon_ray
        # center of the circle (x, y)
        circle_x = self.x_drone[0]
        circle_y = self.x_drone[1]
        
        # random angle
        alpha = 2 * math.pi * random.random()
        # random radius
        r = circle_r * math.sqrt(random.random())
        # calculating coordinates
        x = r * math.cos(alpha) + circle_x
        y = r * math.sin(alpha) + circle_y
        point = (x, y)
        return point



    def sample_random_point_in_horizon_BF(self):

       
          # radius of the circle
        circle_r = self.horizon_ray
        # center of the circle (x, y)
        circle_x = 0
        circle_y = 0
        
        # random angle
        alpha = 2*math.pi* random.random()
        # if (random.random() >0.5):
        #     alpha = -1*alpha
        # random radius
        r = circle_r * math.sqrt(random.random())
        # calculating coordinates
        x = r * math.cos(alpha) + circle_x
        y = r * math.sin(alpha) + circle_y
        point = (x, y)
        return point


    def bound_point(self, point):
        # if point is out-of-bounds, set to bound
        #The dimension lengths now are the diameter around the drone 
        # print("self.X.dimension_lengths[:, 0]: ", self.X.dimension_lengths[:, 0])
        # print("self.X.dimension_lengths[:, 1]: ", self.X.dimension_lengths[:, 1])
        
        #Bound point  in BF search space
        x_lim =  self.X.dimension_lengths[0]
        y_lim =  self.X.dimension_lengths[1]
        
        P_bound_lower_corner = (x_lim[0], y_lim[0])
        P_bound_upper_corner = (x_lim[1], y_lim[1])
        
        #Trasform the point in the GF 
        point_GF = rotation_from_BF_to_GF(point, self.x_drone, self.yaw_drone)
        
        #check if the condition on the safety bound is respected in the GF
         #Check the boudary in the body frame: 
        point = np.maximum(point_GF, P_bound_lower_corner)
        point = np.minimum(point_GF, P_bound_upper_corner)     
        #reconvert the point in the BF
        point = point_from_world_to_body(point, self.x_drone, self.yaw_drone)

        return tuple(point)

    def bound_point_in_drone_horizon(self, point):
       # print("steered point: ", point)
        #check distance between steered point and the drone position 
        dist = distance_drone_steered_point(self.x_drone, self.x_drone)
        #print("dist: ", dist)
        start, end = np.array(self.x_drone), np.array(self.x_drone)
        v = end - start
        u = v / (np.sqrt(np.sum(v ** 2)))
        point = start + u * self.horizon_ray
        #print("steered point: ", point)