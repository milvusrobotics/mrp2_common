# MRP2 Common ROS Packages #

This repository contains common ROS packages for MRP2 Robot. These packages are needed for both the robot itself and the simulation. This version has ros_control implemented with additional built-in packages developed by the community.

### 1. Active Packages ###

* mrp2_analyzer
* mrp2_common
* mrp2_control
* mrp2_description
* mrp2_navigation

### 2. Information on Packages ###
#### 2.1. mrp2_analyzer ####
This package is used for diagnostic purposes. It makes use of [diagnostic_aggregator](http://wiki.ros.org/diagnostic_aggregator) and publishes states of right and left motors, battery and lights.

#### 2.2. mrp2_common ####
This is the metapackage only. No other applications.

#### 2.3. mrp2_control ####
This package contains teleoperation and [ros_control](http://wiki.ros.org/ros_control) related launch files and configurations.

#### 2.4. mrp2_description ####
This package has URDF and xacro files for description of MRP2. Both gazebo and ros_control use this package for visualization and navigation purposes.

#### 2.5. mrp2_navigation ####
This package includes launch files, parameters and maps for different navigation applications.
