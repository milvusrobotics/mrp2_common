cmake_minimum_required(VERSION 2.8.3)
project(mrp2_analyzer)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_aggregator
  diagnostic_msgs
  roscpp
  rospy
)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mrp2_analyzer src/mrp2_analyzer.cpp)
target_link_libraries(mrp2_analyzer ${catkin_LIBRARIES})
