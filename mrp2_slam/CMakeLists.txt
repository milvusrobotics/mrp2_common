cmake_minimum_required(VERSION 2.8.3)
project(mrp2_slam)

find_package(catkin REQUIRED COMPONENTS
  amcl
  gmapping
  map_server
  move_base
  urdf
  xacro
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()