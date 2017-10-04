# RoboRacing Software [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-software.svg?style=svg)](https://circleci.com/gh/RoboJackets/roboracing-software)

<img src="https://raw.githubusercontent.com/wiki/RoboJackets/roboracing-software/images/Macaroni.jpg" style="max-height=400px;">

This repository contains [ROS](http://ros.org) packages for the [RoboJackets](http://robojackets.org) RoboRacing team.

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Sahit%20Chintalapudi%20-blue.svg)](https://github.com/chsahit)

[![Project Manager](https://img.shields.io/badge/Project%20Manager-Evan%20Bretl-blue.svg)](https://github.com/ebretl)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

## Organization

Most folders in this repository are ROS packages.

**avc**: This package contains intelligence code for the [Sparkfun Autonomous Vehicle Challenge](http://avc.sparkfun.com).

**iarrc**: This package contains intelligence code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_description**: This package contains URDFs and meshes that describe the platform's layout.

**rr_gazebo**: This package contains resources for running the car in the [Gazebo](http://gazebosim.org) simulator.

**rr_platform**: This package contains code for interfacing with the various cars built and maintained by the team.

**sandbox**: This package contains utilities and non-ROS code. This includes tools for working with log files and the Arduino code for the car.

The following files and folders enable our continuous integration system.

* external
* util
* circle.yml
* config.docif

## Installation

1. Make sure you have the appropriate ROS version installed for your version of Ubuntu.

2. Clone this repo into the _src_ directory of your catkin workspace.

3. Install the remaining dependencies with the following command in your catkin workspace folder:

   ```
   rosdep install --from-path src --ignore-src -y
   ```
    
4. You should now be able to build the project with `catkin_make`.

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
