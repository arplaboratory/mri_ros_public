import numpy as np
from std_msgs.msg import String, Header, Bool, Float32, Int32
from scene_understanding_pkg_msgs.msg  import RRTPathPoints2D, RRTObstaclesCoo
from geometry_msgs.msg import  Point
import rospy

def publish_APVI_ON(flag):
    pub =  rospy.Publisher('/rrt/start_assistive_guidance', Bool, latch=True, queue_size=1)  
    message = Bool()
    message.data = flag
    pub.publish(message)

#Publish flag final goal reached to the telecontrol script
def publish_final_goal_reached(goal_reached):
    pub = rospy.Publisher('/rrt/final_goal_reached', Bool, queue_size=1)
    msg = Bool()
    msg.data = goal_reached
    pub.publish(msg)

def publish_final_goal_position(goal):
    pub =  rospy.Publisher('/rrt/final_goal', Point, latch=True, queue_size=1) 
    point = Point()
    point.x = goal[0]
    point.y = goal[1]
    point.z = 0.9
    pub.publish(point) 

def publish_path_array_to_goal(path):
    """
    Publish a point array containing the coordinates oif the segnment evaliate to the goal
    """
    pub = rospy.Publisher('/rrt/path', RRTPathPoints2D, queue_size=1)
    msg = RRTPathPoints2D()
    counter = 0
    for i in path:
        point_ = Point(i[0], i[1], 1.0)
        msg.point.append(point_)
        counter = counter + 1
    pub.publish(msg)

def publish_desired_mode(mode):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/case_switcher', Int32, latch=True, queue_size=1)  
    message = Int32()
    message.data = mode
    pub.publish(message)

def publish_increasing_decreasing_yaw(val):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/change_yaw', Int32, latch=True, queue_size=1)  
    message = Int32()
    message.data = val
    pub.publish(message)

def publish_goals(val):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/keyboard_input/visualize_goals', Bool, latch=True, queue_size=1)  
    message = Bool()
    message.data = val
    pub.publish(message)
