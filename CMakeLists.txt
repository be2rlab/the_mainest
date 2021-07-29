cmake_minimum_required(VERSION 3.0.2)
project(the_mainest)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  Marker.msg
  Markers.msg
)

add_service_files(
  FILES
  GoalPoses.srv
  JSPosition.srv
  CartPosition.srv
  ObbArr.srv
  Give.srv
  GetPNPPoses.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs msdp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
