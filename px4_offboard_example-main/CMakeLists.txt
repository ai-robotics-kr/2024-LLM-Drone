cmake_minimum_required(VERSION 2.8.3)
project(offboard_example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  mavros_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  geographic_msgs
  message_generation
)

add_message_files(
  FILES
  WayPoint.msg
)

generate_messages(
  DEPENDENCIES
  mavros_msgs
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

