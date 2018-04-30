cmake_minimum_required(VERSION 2.8.3)
project(ros_openpose_gossip_srvs)


find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    openpose_ros_msgs
    ros_openpose_gossip_msgs
    message_generation
)


 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   OpenPoseGossip.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   openpose_ros_msgs
   ros_openpose_gossip_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_face_identification
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
