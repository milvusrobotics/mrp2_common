cmake_minimum_required(VERSION 2.8.3)
project(mrp2_bumpers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mrp2_bumpers src/mrp2_bumpers.cpp)
target_link_libraries(mrp2_bumpers ${catkin_LIBRARIES})