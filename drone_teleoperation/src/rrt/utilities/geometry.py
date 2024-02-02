# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from itertools import tee
import math

import numpy as np

def align_drone_yaw_to_next_goal( x_goal_GF, x_drone):
    #Evaluate the angle between the goal in the BF and the and the drone body frame 
    a = x_goal_GF[1] - x_drone[1]
    b = x_goal_GF[0] - x_drone[0]
    d = math.sqrt(pow(a,2) + pow(b,2))
    if (a > 0):
        a = b/d
        yaw_des = math.acos(a)
    else:
        yaw_des = -1*math.acos(b/d)
    
    print("yaw_des: ", yaw_des)
    return yaw_des

def find_goal_hor_proj(X, quad_pos, yaw, final_goal_GF, final_goal_BF, x_goal_GF_old,  horizon_ray, init):
    x_goal_BF = final_goal_BF[0]
    y_goal_BF = final_goal_BF[1]
    
    print("yaw: ", yaw)
    print("quad_pos: ", quad_pos)
    print("final_goal_BF: ", final_goal_BF)
    

    if (x_goal_BF <= 0.01 and x_goal_BF >= 0):
        x_goal_BF = 0.00001
        
    if (x_goal_BF >= -0.01 and x_goal_BF < 0):
        x_goal_BF = -0.00001
        
    m = y_goal_BF/x_goal_BF
       
    
    theta = math.atan(m)
    yaw = yaw + theta
    if (yaw > math.pi):
        diff = yaw - math.pi
        yaw = - math.pi + diff
    
    if (yaw < -math.pi):
        diff = yaw + math.pi
        yaw = math.pi - diff
    
    x_rel_goal_BF = horizon_ray
    y_rel_goal_BF = 0.0
    print("yaw: ", yaw)
    
    if (-math.pi/2 - theta >-0.1 and -math.pi/2 - theta < 0.1):  # gola direction parallel to the y axis from the positive side
        #check if the distance with the goalis decreasing otherwise change the sign 
        y_rel_goal_BF = -1*y_rel_goal_BF
    
    rel_goal_BF = (x_rel_goal_BF, y_rel_goal_BF)
    print("rel_goal_BF: ", rel_goal_BF)
   
    #Evaluate the distance between the horizon goal and the goal in the body frame 
    dist_horizon_goal_final_goal_BF = dist_between_points(rel_goal_BF,final_goal_BF)
    
    #Evaluate the distance between the drone and the goal in the body frame 
    dist_drone_final_goal_BF = dist_between_points(0,final_goal_BF)
    flag = False
    # if (dist_horizon_goal_final_goal_BF > dist_drone_final_goal_BF + horizon_ray/2):
    #     #Projecting the value on the line but with opposite sign, 
    #     #to avoid that the drone start to go far away from the obstacle surface 
    #     y_rel_goal_BF = m*(-1*x_rel_goal_BF) 
    #     rel_goal_BF = (-1*x_rel_goal_BF, y_rel_goal_BF)
    #     theta = -np.pi + theta
    #     flag = True #if true the amgle must be only increased
        
    # else:
    #     flag = False
    
    #Evaluate in GF for voxels 
    rel_goal_GF = rotation_from_BF_to_GF(rel_goal_BF, quad_pos,  yaw)
    print("rel_goal_GF: ", rel_goal_GF)
    if ( X.voxels_obstacle_free(rel_goal_GF,0.5) == False):

        print("Point inside obstacle")
        rel_goal_BF, rel_goal_GF= replan_horizon_goal_position(X, horizon_ray, theta, quad_pos, rel_goal_BF, final_goal_BF,final_goal_GF, yaw, init, flag)
    return rel_goal_BF, rel_goal_GF, yaw 
    
    
def replan_horizon_goal_position(X, ray, theta, x_drone, rel_goal_BF,final_goal_BF,final_goal_GF, yaw_drone, init, flag):
    #All the evaljations are in the drone BF
    x_drone_coo = 0.0
    y_drone_coo = 0.0

    x_drone_BF = (0.0, 0.0)
    
    x_rel_goal = rel_goal_BF[0]
    y_rel_goal = rel_goal_BF[1]
    #random_number = np.random.rand()
    #selct if increase or decrease the angle
    step = 0.1; 
    yaw_original = yaw_drone
    yaw_drone_BF = 0.0 #The robot is pointing to the real goal with the yaw
    counter_positive = 0.0
    counter_negative = 0.0
    horizon_goal_positive_BF  = (0.0, 0.0)
    horizon_goal_negative_BF = (0.0, 0.0)
    horizon_goal_positive_GF  = (0.0, 0.0)
    horizon_goal_negative_GF = (0.0, 0.0)
    x_goal_BF_old = 0.0
    # if (mid_goal_counter_update <= 2 and init == False):
    #     #convert previous Goal in the body frame, to project the drone position there 
    #     x_goal_BF_old = point_from_world_to_body(x_goal_GF_old,x_drone, yaw_drone)
    init = True
    val = 2
    for jj in range(0,2):
        yaw_drone = yaw_original
        yaw_drone_BF = 0
        for ii in range(0, 100):
            if (jj == 0):
                #DO a trial with positive direction 
                
                #Test if adding or reducing the angle dcrease the distance with the final goal 
                #This evaluation is in the body frame. We add a step to the yaw evaluated in the body frame. 
                #We consider the goal always sarting a yaw = 0.0 degree respect the robot, because we compute the new yaw in the previosu function
                #considering the angle theta. Theta provide us an offset that make possible to align the yaw. 
                yaw_drone_BF = yaw_drone_BF + step
                x_rel_goal = ray*math.cos(yaw_drone_BF)
                y_rel_goal = ray*math.sin(yaw_drone_BF)
                rel_goal = (x_rel_goal, y_rel_goal)
                horizon_goal_positive_BF = rel_goal
                h_goal = rotation_from_BF_to_GF(rel_goal, x_drone, yaw_drone)
                horizon_goal_positive_GF = h_goal
                counter_positive = counter_positive + 1
            else:
                yaw_drone_BF = yaw_drone_BF - step
                x_rel_goal = ray*math.cos(yaw_drone_BF)
                y_rel_goal = ray*math.sin(yaw_drone_BF)
                rel_goal = (x_rel_goal, y_rel_goal)
                horizon_goal_negative_BF = rel_goal
                h_goal = rotation_from_BF_to_GF(rel_goal, x_drone, yaw_drone)
                horizon_goal_negative_GF = h_goal
                counter_negative = counter_negative + 1

            #check if horizon goal is inside an obstacle, otherwise replan another point on the circunferece until it is outside the obstacle
            #if (X.obstacle_free(horizon_goal_positive_BF) == True and X.collision_free(x_drone_BF,horizon_goal_positive_BF,0.1 )) and X.voxels_obstacle_free(horizon_goal_positive_GF,0.5) == True:
            #    break
            if (X.voxels_obstacle_free(h_goal,0.5) == True):
                #if the direction is obstacle free stop the counter 
                break
    if (counter_negative > counter_positive):
        return horizon_goal_positive_BF, horizon_goal_positive_GF
    else:
        return horizon_goal_negative_BF, horizon_goal_negative_GF
    
                
          





def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians



def dist_between_points(a, b):
    """
    Return the Euclidean distance between two points
    :param a: first point
    :param b: second point
    :return: Euclidean distance between a and b
    """
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance


def pairwise(iterable):
    """
    Pairwise iteration over iterable
    :param iterable: iterable
    :return: s -> (s0,s1), (s1,s2), (s2, s3), ...
    """
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


def es_points_along_line(start, end, r):
    """
    Equally-spaced points along a line defined by start, end, with resolution r
    :param start: starting point
    :param end: ending point
    :param r: maximum distance between points
    :return: yields points along line from start to end, separated by distance r
    """
    d = dist_between_points(start, end)
    n_points = int(np.ceil(d / r)) # r is the edge length specified by the user when the tree has to turn around a n obstacle
    if n_points > 1:
        step = d / (n_points - 1) # step where locating next point
        for i in range(n_points):
            next_point = steer(start, end, i * step)
            if (next_point == (0.0, 0.0)):
                continue
            yield next_point


def steer(start, goal, d):
    """
    This function is callsed when the new points need to go around an obstackle.
    Return a point in the direction of the goal, that is distance away from start
    :param start: start location
    :param goal: goal location
    :param d: distance away from start
    :return: point in the direction of the goal, distance away from start
    """
    # Take a new point lying on the line in the direction of the goal. 
    # The new point is taken a distance d along the vector u (starting from start)
    # in the direction of the goal
    
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    steered_point = start + u * d
    return tuple(steered_point)


#Point rotations from world to body frame 
def point_from_world_to_body(point, drone_GF, theta):
    x_goal_GF = point[0]
    y_goal_GF = point[1]

    x_drone_GF = drone_GF[0]
    y_drone_GF = drone_GF[1]

  
    x_b = x_goal_GF* math.cos(theta) + y_goal_GF* math.sin(theta) - x_drone_GF* math.cos(theta) - y_drone_GF*math.sin(theta)
    y_b = -x_goal_GF*math.sin(theta) + y_goal_GF*math.cos(theta) + x_drone_GF*math.sin(theta) - y_drone_GF*math.cos(theta)
    
    point = (x_b, y_b)
    return point

def rotation_from_BF_to_GF(point, drone_GF, theta):
    x_point_BF = point[0]
    y_point_BF = point[1]

    x_drone_GF = drone_GF[0]
    y_drone_GF = drone_GF[1]
    
   
    x_GF = x_point_BF * math.cos(theta) - y_point_BF * math.sin(theta) + x_drone_GF
    y_GF = x_point_BF * math.sin(theta) + y_point_BF * math.cos(theta) + y_drone_GF

    point = (x_GF, y_GF)
    return point



     
def generate_ellipsoids(Obs,  drone_GF, yaw):
    # !!!! This function works only for the rtree generated obstacles,which are parallel to the axis
    # The ellispsoid are express in drone BF 


    C = []
    semi_axis = []
    theta_BF_list = []
    offset_x = 0.2
    offset_y = 0.1
    #Iterate  on the number of obstacles
    for o in Obs:
        x_min = o[0] - offset_x
        y_min = o[1] #+ offset_y
        x_max = o[2] + offset_x
        y_max = o[3] #- offset_y
        
        #Supposimg rectangular or square obstacle aligned with the axis 
        l = (x_max) - (x_min)
        h = y_max - y_min
        
        left_min = (x_min, y_min)
        right_min = (x_min + l, y_max - h)


         #evaliate m of the line passing between the bottom left and the bottom right corner 
        m_GF = (right_min[1] -left_min[1])/(right_min[0] - left_min[0]) #Generally is zero because the obstacle are generated in this way with thsi procedure 
        theta_GF = math.atan(m_GF)
        
      
        theta_BF = theta_GF - yaw
        theta_BF_list.append(theta_BF)
        #print("theta_BF: ", theta_BF)
        
        #if (theta_GF == 0)
        #Thenere conto dell'angolo theta. Ruotare i punti allineati all'asse x, calcolare ellisse e ruotare elisse se non dovesse essere zero
        #Evaluate center of the ellipse in GF
        x_c = abs(x_max -x_min)/2
        x_c = x_min + x_c
        y_c = abs(y_max -y_min)/2
        y_c = y_min + y_c
        
       
        #ROtate center poinst in the BF
        point = (x_c, y_c)
        center_BF = point_from_world_to_body(point, drone_GF, yaw)
        C.append(center_BF)
        
        #Evaluate the length of the semi-axes innGF 
        #Evaluate the semi axes a, b, where a is the shorter semiaxis and b the longer 
        if (abs(y_max - y_min)/2 > abs(x_max - x_min)/2 ):
            b = abs(y_max - y_min)/2
            a = abs(x_max - x_min)/2
        else:
            b = abs(x_max - x_min)/2
            a = abs(y_max - y_min)/2
       
        semi_axis.append([1.5*a, 1.5*b])
              
       #ritorna semi-axes and center
    return C, semi_axis, theta_BF_list  #In



def rotate_ellipsoids_from_BF_to_GF(C, semi_axis, theta_BF, drone_GF, yaw):
    #Rotate the ellipsoid from BF to GF for Visualization 
    counter = 0
    C_GF_list = []
    theta_GF_list = []
    semi_axis_GF = []
   
     #rotate C_BF in C_GF
    c_GF = rotation_from_BF_to_GF(C, drone_GF, yaw)
    #evaluate theta in GF
   
    theta_GF = yaw + theta_BF
    semi_axis_GF = semi_axis
   
    return c_GF, theta_GF, semi_axis_GF



def evaluate_drone_goal_distance(drone_pos, goal):
    a = pow(drone_pos[0] - goal[0], 2)
    b = pow(drone_pos[1] - goal[1], 2)
    distance = math.sqrt(a + b)

    return distance 

