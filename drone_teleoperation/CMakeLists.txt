set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.1...3.15)
project(drone_teleoperation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mav_manager
  trackers_msgs
  quadrotor_msgs
  scene_understanding_pkg_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mav_manager
   CATKIN_DEPENDS roscpp rospy std_msgs mav_manager trackers_msgs quadrotor_msgs scene_understanding_pkg_msgs
   scene_understanding_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/drone_teleoperation.cpp
# )
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/admittance_hri_server.cpp
  src/utils/admittance_utils.cpp
  src/utils/planner_utils.cpp
  src/utils/ros_visualization.cpp
  src/utils/log_files_writing.cpp
  src/core/admittance_core.cpp
  src/core/Admittance_Controller.cpp
  src/core/KF.cpp
)

add_library(KF src/core/KF.cpp)
target_link_libraries(KF ${catkin_LIBRARIES})

add_library(Admittance_Controller src/core/Admittance_Controller.cpp)
target_link_libraries(Admittance_Controller ${catkin_LIBRARIES})

add_library(admittance_utils src/utils/admittance_utils.cpp)
target_link_libraries(admittance_utils ${catkin_LIBRARIES})

add_library(log_files_writing src/utils/log_files_writing.cpp)
target_link_libraries(log_files_writing ${catkin_LIBRARIES})

add_library(planner_utils src/utils/planner_utils.cpp)
target_link_libraries(planner_utils ros_visualization ${catkin_LIBRARIES})

add_library(ros_visualization src/utils/ros_visualization.cpp)
target_link_libraries(ros_visualization ${catkin_LIBRARIES})


add_library(admittance_core src/core/admittance_core.cpp)
target_link_libraries(admittance_core admittance_utils Admittance_Controller KF planner_utils ros_visualization ${catkin_LIBRARIES})



#add_executable(admittance_teleop_sim
#               src/admittance_teleop_sim.cpp)

#target_link_libraries(admittance_teleop_sim Admittance_Controller KF ${catkin_LIBRARIES})
#add_dependencies(admittance_teleop_sim ${catkin_EXPORTED_TARGETS})


#add_executable(admittance_teleop_voxl
#               src/admittance_teleop_voxl.cpp)
#target_link_libraries(admittance_teleop_voxl Admittance_Controller KF ${catkin_LIBRARIES})
#add_dependencies(admittance_teleop_voxl ${catkin_EXPORTED_TARGETS})


cs_add_executable(admittance_hri_server
  src/admittance_hri_server_node.cpp
)
target_link_libraries(admittance_hri_server  admittance_utils admittance_core Admittance_Controller ros_visualization log_files_writing ${PROJECT_NAME} ${catkin_LIBRARIES})


