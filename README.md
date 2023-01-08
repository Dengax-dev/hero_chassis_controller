# hero_chassis_controller

## Overview

This is controller package, Used to control the movement of the robot

**Keywords:** RoboMaster, ROS, ros_control

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Dengjiaxiong<br />
Affiliation: [Dynamicx]()<br />
Maintainer: Dengjiaxiong, dengax1314@qq.com**

The simple_chassis_controller package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
    # git clone
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller hero_chassis_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of simple_chassis_controller and joint_state_controller.

## Launch files

* **hero_chassis_controller.launch:** 

## Bugs & Feature Requests

[ROS]: http://www.ros.org

[add]pid_controller
