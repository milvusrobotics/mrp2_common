# README #

This repository contains the main ROS packages for MRP2. This version has ros_control implemented with additional built-in packages developed by the community.

### Active Packages ###

* mrp2
* mrp2_bringup
* mrp2_controller_configuration
* mrp2_description
* mrp2_hardware
* mrp2_display

### Obselete Packages ###

* joystick_controller
* motion_controller
* powerboard_messenger
* robot_setup_tf


### How to launch? ###

robot_upstart is not implemented yet on the new ROS setup of MRP2. Though the main launch file for the startup is as follows:
```sh
roslaunch mrp2_bringup mrp2.launch
```
Launch sequence is as follows:

* mrp2.launch
* upload_mrp2.launch
* mrp2_bringup.launch
    * default_controllers.launch
    * twist_mux.launch
    * joystick_teleop.launch
* starter_gmapping.launch

