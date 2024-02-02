#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from core.rrt_star_voxels import RRTStar #rrt_star
from utilities.search_space import SearchSpace
# from utilities.plotting import Plot
# from utilities.plotting_v2 import Plot_v2

X_dimensions = np.array([(-4, 4), (-5, 5)])  # dimensions of Search Space
# obstacles
#Obstacles = np.array([(1, 4, 3, 8), (4, 0, 6, 3), (7, 4, 9, 6), (4, 6, 6, 10)])
Obstacles = np.array([(-1, -1, -0.7, -0.7), (-1, 0.7, -0.7, 1.0)]) #
x_drone = (-1.0, -0.5)  # starting location (0, 3)
x_goal_final_GF= (3.0, 1.0)  # goal location (10, 1)
horizon_ray = 0.5

Q = np.array([(0.3, 4)])  #length of tree edges
                        # first number: is the length of the edge 
                        # second number: number of edges required to reach the goal 

r = 0.2  # length of smallest edge to check for intersection with obstacles
max_samples = 2000  # max number of samples to take before timing out
rewire_count = 70  # optional, number of nearby branches to rewire
prc = 0.7 # probability of checking for a connection to goal

# create Search Space
th_length = 0.2
th_height = 0.2
safety_bound = (th_length, th_height)
X = SearchSpace(X_dimensions, safety_bound, Obstacles)
X.obtain_voxels_collisions = True

# create rrt_search
rrt = RRTStar(X, Q, x_drone, Obstacles,max_samples,horizon_ray, r, prc, rewire_count=None)
path = rrt.rrt_star()

# # plot
# plot = Plot("rrt_star_2d")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)
# plot.plot_start(X, x_drone)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)