cmake_minimum_required(VERSION 2.8.3)
project(jy901_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
  dynamic_reconfigure
  rosconsole
  roscpp_serialization 
  roslib 
  rostime
  sensor_msgs
  serial
)

catkin_package(
 INCLUDE_DIRS src
 CATKIN_DEPENDS serial roscpp sensor_msgs tf
 DEPENDS system_lib
 message_runtime 
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

add_executable(jy901_driver_node
  src/jy901_driver_node.cpp
)

target_link_libraries(jy901_driver_node
  ${catkin_LIBRARIES}
)

