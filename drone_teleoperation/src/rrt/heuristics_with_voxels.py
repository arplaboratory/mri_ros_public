# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from utilities.geometry import dist_between_points, point_from_world_to_body, rotation_from_BF_to_GF
import math
import numpy as np

def cost_to_go(a: tuple, b: tuple) -> float:
    """
    :param a: current location
    :param b: next location
    :return: estimated segment_cost-to-go from a to b
    """
    return dist_between_points(a, b)

def path_cost(E, a, b):
    """
    Cost of the unique path from x_init to x
    :param E: edges, in form of E[child] = parent
    :param a: initial location
    :param b: goal location
    :return: segment_cost of unique path from x_init to x
    """
    cost = 0
    while not b == a:
        p = E[b]
        cost += dist_between_points(b, p)
        b = p
    return cost


def segment_cost(a, b):
    """
    Cost function of the line between x_near and x_new
    :param a: start of line
    :param b: end of line
    :return: segment_cost function between a and b
    """
    
    return dist_between_points(a, b)


def distance_drone_goal(x_drone, x_goal): #Evaluate in BF (cambiare ora è in GF)+
    a = x_drone[0] - x_goal[0]
    a1 =  x_drone[1] - x_goal[1]
    a2 = math.pow(a, 2)
    a3 = math.pow(a1, 2)
    dist = math.sqrt(a2 + a3)

    return dist

def distance_drone_steered_point(x_drone, point): #Evaluate in BF (cambiare ora è in GF)+
    a = x_drone[0] - point[0]
    a1 =  x_drone[1] - point[1]
    a2 = math.pow(a, 2)
    a3 = math.pow(a1, 2)
    dist = math.sqrt(a2 + a3)

    return dist


def replan_horizon_goal_position(X, ray, theta, x_drone, rel_goal_BF,final_goal_BF,  x_goal_GF_old, mid_goal_counter_update, yaw_drone, init, flag):

    #All the evaljations are in the drone BF
    x_drone_coo = 0.0
    y_drone_coo = 0.0
  
    x_drone_BF = (0.0, 0.0)
    
    x_rel_goal = rel_goal_BF[0]
    y_rel_goal = rel_goal_BF[1]

    #random_number = np.random.rand()
    #selct if increase or decrease the angle
    step = 0.1; 
    theta_original = theta
    distance_positive = 0.0
    distance_negative = 0.0
    horizon_goal_positive  = (0.0, 0.0)
    horizon_goal_negative = (0.0, 0.0)
    x_goal_BF_old = 0.0
    # if (mid_goal_counter_update <= 2 and init == False):
    #     #convert previous Goal in the body frame, to project the drone position there 
    #     x_goal_BF_old = point_from_world_to_body(x_goal_GF_old,x_drone, yaw_drone)
    init = True
    val = 2
    for jj in range(0,2):
        theta = theta_original
        for ii in range(0, 100):
            if (jj == 0):
                #DO a trial with positive direction 
                if (init == True):
                    #Test if adding or reducing the angle dcrease the distance with the final goal 
                    theta = theta + step
                    x_rel_goal = ray*math.cos(theta)
                    y_rel_goal = ray*math.sin(theta)
                    distance_positive = math.sqrt(pow(x_rel_goal- final_goal_BF[0],2) + pow(y_rel_goal- final_goal_BF[1],2))
                    
                    #Test the negative angle 
                    theta = theta - 2*step  #the step is double because we previously add a step angle computing the positive distance 
                    x_rel_goal = ray*math.cos(theta)
                    y_rel_goal = ray*math.sin(theta)
                    distance_negative= math.sqrt(pow(x_rel_goal- final_goal_BF[0],2) + pow(y_rel_goal- final_goal_BF[1],2))

                    if (distance_positive > distance_negative):
                        val = 0  #the angle must be redueced
                    else:
                        val = 1
                    
                    init = False
                

                if (val == 0):
                    theta = theta + step
                else:
                    theta = theta - step




                print("[replan_horizon_goal_position] theta +: ",theta )
                # x_rel_goal = 0.0
                # y_rel_goal = 0.0
            
                # if (x_goal_coo < 0):
                #     x_rel_goal = -1* ray*math.cos(theta) # +x_goal_BF_old[0] #il segno della somma va cambiato a seconda che il gol si trovi su asse x
                # else:                                                           #positivo onegativo del drone. Se in body frame non necessario se yaw orientato verso il target
                #     x_rel_goal = ray*math.cos(theta)# +x_goal_BF_old[0]
        
                # if (y_goal_coo < 0):
                #     y_rel_goal = -1*ray*math.sin(theta)#  +x_goal_BF_old[1]
                # else:

                x_rel_goal = ray*math.cos(theta)
                y_rel_goal = ray*math.sin(theta)
                #y_rel_goal = ray*math.sin(theta)#  +x_goal_BF_old[1]
        
                horizon_goal_positive_BF = (x_rel_goal, y_rel_goal)
                # print("horizon_goal_positive_BF: ", horizon_goal_positive_BF)
                horizon_goal_positive_GF = rotation_from_BF_to_GF(horizon_goal_positive_BF, x_drone, yaw_drone)
                # print("horizon_goal_positive_GF: ", horizon_goal_positive_GF) 
                #check if horizon goal is inside an obstacle, otherwise replan another point on the circunferece until it is outside the obstacle
                if (X.obstacle_free(horizon_goal_positive_BF) == True and X.collision_free(x_drone_BF,horizon_goal_positive_BF,0.1 )) and X.voxels_obstacle_free(horizon_goal_positive_GF,0.2) == True:
                  
                    break
            # else:
            #     #DO a trial with positive direction 
            #     theta = theta - step
            #      #find the coordinates 
            #     x_rel_goal = 0.0
            #     y_rel_goal = 0.0
            
            #     if (x_goal_coo < 0):
            #         x_rel_goal = -1* ray*math.cos(theta) #+x_goal_BF_old[0]#il segno della somma va cambiato a seconda che il gol si trovi su asse x
            #     else:                                                           #positivo onegativo del drone. Se in body frame non necessario se yaw orientato verso il target
            #         x_rel_goal = ray*math.cos(theta)# +x_goal_BF_old[0]
        
            #     # if (y_goal_coo < 0):
            #     #     y_rel_goal = -1*ray*math.sin(theta)  # +x_goal_BF_old[1]
            #     # else:
            #     y_rel_goal = ray*math.sin(theta)# +x_goal_BF_old[1]
        
            #     horizon_goal_negative_BF= (x_rel_goal, y_rel_goal)
            #     print("horizon_goal_negative_BF: ", horizon_goal_negative_BF)   
            #     # evaluate position in the GF
            #     horizon_goal_negative_GF = rotation_from_BF_to_GF(horizon_goal_negative_BF, x_drone, yaw_drone)

            #     print("horizon_goal_negative_GF: ", horizon_goal_negative_GF)      
            #     #check if horizon goal is inside an obstacle, otherwise replan another point on the circunferece until it is outside the obstacle
            #     if (X.obstacle_free(horizon_goal_negative) == True and X.collision_free(x_drone_BF,horizon_goal_negative_BF,0.1)):
            #         distance_negative = math.sqrt(pow(x_rel_goal- x_goal_coo,2) + pow(y_rel_goal- y_goal_coo,2))
                    
            #         break
           
    return horizon_goal_positive_BF 
    # if (distance_positive < distance_negative or flag == True):
    #     print("[replan_horizon_goal_position] return distance_positive: ",distance_positive)
    #     return horizon_goal_positive_BF 
    # else:
    #     print("[replan_horizon_goal_position] return distance_negative: ",distance_negative)
    #     return horizon_goal_negative_BF



 