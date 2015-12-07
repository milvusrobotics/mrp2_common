cmake_minimum_required(VERSION 2.8.3)
project(mrp2_display)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(geniePi src/geniePi.c)
add_executable(mrp2_display_node src/display.cpp)
target_link_libraries(mrp2_display_node geniePi ${catkin_LIBRARIES})