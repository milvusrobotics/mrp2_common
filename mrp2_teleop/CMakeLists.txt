cmake_minimum_required(VERSION 2.8.3)
project(mrp2_teleop)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  interactive_marker_twist_server
  joint_state_controller
  joy
  robot_localization
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})