cmake_minimum_required(VERSION 2.8.3)
project(mrp2_sonars)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrp2_hardware
)

find_package(mrp2_hardware REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${mrp2_hardware_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(serial_comm src/serial_comm.cpp)
#add_library(mrp2_sonar_serial src/mrp2_serial.cpp src/rs232.c)
#target_link_libraries(mrp2_sonar_serial  serial_comm ${catkin_LIBRARIES})

add_executable(mrp2_sonars_node src/mrp2_sonars.cpp)
target_link_libraries(mrp2_sonars_node mrp2_serial ${catkin_LIBRARIES})