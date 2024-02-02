from itertools import tee
import math

import numpy as np

def set_goal():
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
        
    x_goal_final_GF = (float(x_goal),float(y_goal))
        
    if ( flag1 == True and flag1 == True):
        goal_defined = True

    return goal_defined, x_goal_final_GF