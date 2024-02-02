cmake_minimum_required(VERSION 2.8.3)
project(scene_understanding_pkg_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
 
)



#Place Here all the messages generated for the unity communication
add_message_files(FILES
    MeshVertexPosition.msg
    PosRot.msg
    MeshPosArray.msg
    MeshRelatedData.msg
    waypointMsg.msg
    waypointArray.msg
    MeshPos.msg
    RRTPathPoints2D.msg
    RRTObstaclesCoo.msg
    ObstacleRepForce.msg
    IntermediatePointsFPVI.msg

  
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs

  )
#Dal nome del messaggion nel file .cs togliere Msg nella parte terminakle

#Place Here all the srv created
#add_service_files(FILES
#    ObjectPoseService.srv
#	PositionService.srv
#)



catkin_package(CATKIN_DEPENDS
    message_runtime)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotics_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
