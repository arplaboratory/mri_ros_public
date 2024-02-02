#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray 
from std_msgs.msg import String

def convert_bbox3d(param_list):
    bbox3darray_msg = BoundingBox3DArray()
    bbox3darray_msg.header.stamp = rospy.Time.now()
    bbox3darray_msg.header.frame_id="mr_map"
    for item in param_list:
        bbox3d_msg = BoundingBox3D()
        bbox3d_msg.center = Pose()
        bbox3d_msg.center.position =Point(item['x'], item['y'], item['z'])
        bbox3d_msg.center.orientation = Quaternion(item['qx'], item['qy'], item['qz'], item['qw'])
        bbox3d_msg.size = Vector3(item['sx'], item['sy'], item['sz'])
        bbox3darray_msg.boxes.append(bbox3d_msg)
       
    return bbox3darray_msg

if __name__ == '__main__':
    rospy.init_node('mr_map_publisher', anonymous=True)
    obstacle_pub = rospy.Publisher("mr_obstacle", BoundingBox3DArray, queue_size=1)
    gate_pub = rospy.Publisher("mr_gate", BoundingBox3DArray, queue_size = 1)
    obstacle_list = rospy.get_param("/map_obs_param/obstacle_list")
    gate_list = rospy.get_param("/map_obs_param/gate_list")
    obstacle_msg = convert_bbox3d(obstacle_list)
    gate_msg = convert_bbox3d(gate_list)
    max_count = 100
    counter = 0
    while(counter < max_count):
        obstacle_pub.publish(obstacle_msg)
        gate_pub.publish(gate_msg)
        counter = counter + 1
        rospy.sleep(0.1)


       
   
    

    
    


   
