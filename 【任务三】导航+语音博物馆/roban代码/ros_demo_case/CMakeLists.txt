cmake_minimum_required(VERSION 2.8.3)
project(ros_demo_case)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
