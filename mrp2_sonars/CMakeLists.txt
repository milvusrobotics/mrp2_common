cmake_minimum_required(VERSION 2.8.3)
project(mrp2_sonars)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mrp2_sonar_serial src/mrp2_serial.cpp src/rs232.c)
add_executable(mrp2_sonars_node src/mrp2_sonars.cpp)
target_link_libraries(mrp2_sonars_node mrp2_sonar_serial ${catkin_LIBRARIES})