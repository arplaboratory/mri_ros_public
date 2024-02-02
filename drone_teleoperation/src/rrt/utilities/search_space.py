# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import numpy as np
from rtree import index
import math
import time

from utilities.geometry import es_points_along_line, point_from_world_to_body, generate_ellipsoids, rotate_ellipsoids_from_BF_to_GF, rotation_from_BF_to_GF
from utilities.obstacle_generation import obstacle_generator
#from ros_data import obtain_voxels_positions


class SearchSpace(object):
    def __init__(self, dimension_lengths, safety_bound, O=None):
        """
        Initialize Search Space
        :param dimension_lengths: range of each dimension
        :param O: list of obstacles
        """

        # sanity check
        if len(dimension_lengths) < 2:
            raise Exception("Must have at least 2 dimensions")
        self.dimensions = len(dimension_lengths)  # number of dimensions
        # sanity checks
        if any(len(i) != 2 for i in dimension_lengths):
            raise Exception("Dimensions can only have a start and end")
        if any(i[0] >= i[1] for i in dimension_lengths):
            raise Exception("Dimension start must be less than dimension end")
        self.dimension_lengths = dimension_lengths  # length of each dimension
        p = index.Property()
        p.dimension = self.dimensions

        #O is the list of the obstacles vertices organied as (x_min y_min x_max y_max)
        if O is None:
            #Initialize the index cklass of rtree
            self.obs = index.Index(interleaved=True, properties=p)
            self.obs_visualization = index.Index(interleaved=True, properties=p)
            
        else:
            # r-tree representation of obstacles
            #add a boundary around the obsacle 

            th_length = safety_bound[0]
            th_high = safety_bound[1]
            O_vis = O
            for i in range(0, len(O)):
                O[i][0] =  O[i][0] - th_length
                O[i][1] =  O[i][1] - th_high
                O[i][2] =  O[i][2] + th_length
                O[i][3] =  O[i][3] + th_high
                
           
            # sanity check
           
            if any(len(o) / 2 != len(dimension_lengths) for o in O):
                raise Exception("Obstacle has incorrect dimension definition")
            if any(o[i] >= o[int(i + len(o) / 2)] for o in O for i in range(int(len(o) / 2))):
                raise Exception("Obstacle start must be less than obstacle end")
            self.obs = index.Index(obstacle_generator(O), interleaved=True, properties=p)
            self.obs_visualization = index.Index(obstacle_generator(O_vis), interleaved=True, properties=p)
            
            #Generate ellipsoid around the rectangular obstacle.
            self.C = []
            self.semi_axes = []
            self.theta = []
            self.theta_BF = []
            self.drone_GF = (0.0, 0.0)
            self.drone_height = 0.0
            self.yaw = 0.0
            drone_GF = (0.0, 0.0) # in initialization
            yaw = 0.0 # in intialization
            self.surface_pc = []
            self.C, self.semi_axes, self.theta = generate_ellipsoids(O, drone_GF, yaw)
            #relatively to the voxels positions 
            self.obtain_voxels_collisions = False
            self.safety_bounds = (th_length, th_high)
            self.obstacle_hit = False
            self.init_replanning = False

          
    


 
    def obstacle_free(self, x_BF):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
       
        #     #rotating the point in input form the body to the ground
        x = rotation_from_BF_to_GF(x_BF,  self.drone_GF, self.yaw)
        return self.obs.count(x) == 0
   
    
    def voxels_obstacle_free(self, point, ray):
        """
        check if the drone horizon is free from voxels and real obstacles. 

        """
        ray = 0.4
        counter = 0
        for i in range(0, len(self.surface_pc)):
            #evaluate disatnce from the selected point position from the voxels positions 
            a = pow(point[0] - self.surface_pc[i][0], 2)
            b = pow(point[1] - self.surface_pc[i][1], 2)
            #the point height is given by the height of the drone in that moment 
            c = self.drone_height -self.surface_pc[i][2]
          
            d = math.sqrt(a + b) #2d distance
           
            if (d < ray and c < abs(0.1)):
                counter = counter + 1
                break
        traversable_point = False
        if (counter == 0):
            traversable_point = True
        print("[voxels_obstacle_free] traversable_point: ", traversable_point)
        print("[voxels_obstacle_free] point: ", point)
        print("[voxels_obstacle_free] self.drone_height: ", self.drone_height)
        # time.sleep(2)
        return traversable_point
            
            



    def sample_free(self):
        """
        Sample a location within X_free

        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        points = es_points_along_line(start, end, r)
        coll_free = all(map(self.obstacle_free, points))
        return coll_free

    def sample(self):
        """
        Return a random location within X
        :return: random location within X (not necessarily X_free)
        """
        x = np.random.uniform(self.dimension_lengths[:, 0], self.dimension_lengths[:, 1])
        return tuple(x)

    

    