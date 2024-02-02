#cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.0.2)
project(scene_understanding_pkg)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#set(CMAKE_MODULE_PATH "/home/arpl/voxblox_ws/build/voxblox/catkin_generated/installspace" ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  visualization_msgs
  pcl_msgs
  cv_bridge
  image_transport
  pcl_ros
  voxblox
  voxblox_msgs
  scene_understanding_pkg_msgs
)



FIND_PACKAGE(OpenCV REQUIRED)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(OpenCV REQUIRED)
#find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video features2d ml highgui objdetect)


catkin_package( INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    ros_tcp_endpoint
    message_runtime
    scene_understanding_pkg_msgs
    DEPENDS system_lib
    tf
    cv_bridge
    image_transport)


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/perception_server.cpp
  src/utils/perception_utils.cpp
)
  

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  
  include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS})

  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  # link_directories(${catkin_INCLUDE_DIRS}   ${OpenCV_INCLUDE_DIRS}  ${PCL_LIBRARY_DIRS} )
  










add_executable(triangle_mesh_rviz_from_txt_file 
               src/triangle_mesh_rviz_from_txt_file.cpp)
target_link_libraries(triangle_mesh_rviz_from_txt_file ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(display_received_meshes_in_rviz 
               src/display_received_meshes_in_rviz.cpp)
target_link_libraries(display_received_meshes_in_rviz ${catkin_LIBRARIES})



###### FEATURE DETECTOR 
#add_library(Feature_detection src/Feature_detector/Feature_detection.cpp)
#target_link_libraries(Feature_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(feature_detector  
#               src/Feature_detector/feature_detector.cpp)

#target_link_libraries(feature_detector Feature_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_library(Mesh_constructor src/Mesh_constructor.cpp)
target_link_libraries(Mesh_constructor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(SU_Unity_comm src/SU_Unity_comm.cpp)
target_link_libraries(SU_Unity_comm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(perception_utils src/utils/perception_utils.cpp)
target_link_libraries(perception_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(perception_server_lib src/perception_server.cpp)
target_link_libraries(perception_server_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Kimera_vio_data 
               src/Kimera_vio_data.cpp)
target_link_libraries(Kimera_vio_data Mesh_constructor SU_Unity_comm  ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
#add_dependencies(Kimera_vio_data ${catkin_EXPORTED_TARGETS})

add_executable(Kimera_vio_data_for_rosbag 
               src/Kimera_vio_data_for_rosbag.cpp)

target_link_libraries(Kimera_vio_data_for_rosbag Mesh_constructor ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

cs_add_executable(perception_server
  src/perception_server_node.cpp
)
target_link_libraries(perception_server perception_utils ${PROJECT_NAME})

#add_executable(perception_server 
#               src/perception_server_node.cpp)
#target_link_libraries(perception_server  Mesh_constructor SU_Unity_comm perception_utils ${catkin_LIBRARIES} #${PCL_LIBRARIES})





## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scene_understanding_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
