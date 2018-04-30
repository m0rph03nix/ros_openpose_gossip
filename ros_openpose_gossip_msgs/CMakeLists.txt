cmake_minimum_required(VERSION 2.8.3)
project(ros_openpose_gossip_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    std_msgs
    message_generation
)

add_message_files(FILES
  PersonGossip.msg
  PersonsGossip.msg

)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_face_identification
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

