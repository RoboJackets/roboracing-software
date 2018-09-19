# RoboRacing Software [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-software.svg?style=svg)](https://circleci.com/gh/RoboJackets/roboracing-software)

<img src="https://raw.githubusercontent.com/wiki/RoboJackets/roboracing-software/images/buzz_bigoli.jpg" style="max-height=400px;">

This repository contains [ROS](http://ros.org) packages for the [RoboJackets](http://robojackets.org) RoboRacing team.

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Evan%20Bretl-blue.svg)](https://github.com/ebretl)

[![Project Manager](https://img.shields.io/badge/Project%20Manager-Varun%20Madabushi-blue.svg)](https://github.com/varunm99)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

## Organization

Most folders in this repository are ROS packages.

**avc**: This package contains mission code for the [Sparkfun Autonomous Vehicle Challenge](http://avc.sparkfun.com).

**iarrc**: This package contains mission code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_common**: This package contains general-purpose or mission-agnostic intelligence code.

**rr_description**: This package contains URDFs and meshes that describe the platform's layout.

**rr_gazebo**: This package contains resources for running the car in the [Gazebo](http://gazebosim.org) simulator.

**rr_platform**: This package contains code for interfacing with the various cars built and maintained by the team.

**sandbox**: This package contains utilities and non-ROS code. This includes tools for working with log files and the Arduino code for the car.

The following files and folders enable our continuous integration system.

* .circleci
* Dockerfile

## Installation

1. Make sure you have the appropriate ROS version installed for your version of Ubuntu.

2. Clone this repo into the _src_ directory of your catkin workspace.

3. Install the remaining dependencies with the following command in your catkin workspace folder:

   ```sh
   rosdep install --from-path src --ignore-src -y
   ```

	- Some packages may have to be installed from source. For example, if realsesnse2_camera cannot be found, then search the error and find the ROS wiki page http://wiki.ros.org/realsense2_camera. Find the link to the source git repo and clone that into catkin_ws/src.

4. Install keras with :

   ```
   pip install keras
   ```

5. Install librealsense following the instructions at https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

6. You should now be able to build the project by running `catkin_make` in the catkin_ws directory.


## Simulation

You can get started with the RoboRacing code base right away by launching our simulator!

The following command will load our platform in the Sparkfun AVC track:
```
roslaunch rr_gazebo macaroni_avc.launch
```
Then, the following command will start our race AI and drive the car around the track:
```
roslaunch avc avc.launch
```
Alternatively, you can control the car manually with a USB gamepad with this command:
```
roslaunch rr_platform joystick_driver.launch
```
