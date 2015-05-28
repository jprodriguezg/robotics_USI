cmake_minimum_required(VERSION 2.8.3)
project(robotics_lab_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

add_message_files(
   FILES
   ${messages_to_build}
 )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 
		geometry_msgs   
		nav_msgs   
		sensor_msgs   
		std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS 
		geometry_msgs 
		message_runtime 
		nav_msgs rospy 
		sensor_msgs 
		std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



#############
## Testing ##
#############


