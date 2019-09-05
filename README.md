# RoboRacing Software [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-software.svg?style=svg)](https://circleci.com/gh/RoboJackets/roboracing-software)

<img src="https://raw.githubusercontent.com/wiki/RoboJackets/roboracing-software/images/sedanii.png" width="1500" height="500">

Welcome to the [RoboJackets](http://robojackets.org) RoboRacing software repository! This document will give you a brief outline of the repository's layout and some simple instructions for setting up the project. For more detailed information, please visit the [wiki](https://wiki.robojackets.org/RoboRacing).

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Daniel%20Martin-blue.svg)](https://github.com/daniel-martin576)

[![Project Manager](https://img.shields.io/badge/Project%20Manager-Austin%20Keener-blue.svg)](https://github.com/akeener97)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

## Organization

This repository is comprised of multiple ROS packages and one sandbox folder for miscellaneous resources.

**rr_avc**: This package contains mission code for the [Sparkfun Autonomous Vehicle Challenge](http://avc.sparkfun.com).

**rr_iarrc**: This package contains mission code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_common**: This package contains general-purpose or mission-agnostic intelligence code.

**rr_description**: This package contains URDFs and meshes that describe the platform's layout.

**rr_gazebo**: This package contains resources for running the car in the [Gazebo](http://gazebosim.org) simulator.

**rr_platform**: This package contains code for interfacing with the various cars built and maintained by the team.

**rr_ml**: This package contains our machine learning models and tools for working with datasets.

**sandbox**: This package contains utilities and non-ROS code. This includes tools for working with log files and the Arduino code for the car.

The following files and folders enable our continuous integration system.

* .circleci
* Dockerfile

## Installation

This repository should be cloned into the src directory of a catkin workspace. Use ```catkin_make``` in the workspace directory to build the code. (NOTE: Be sure to ```source devel/setup.sh``` before referencing roboracing packages.)

For a guide on installing our code please go to [our guide](https://wiki.robojackets.org/RoboRacing_Software_Installation_Instructions).


## Simulation

You can get started with the RoboRacing code base right away by launching our simulator!

The following command will load our platform in the Sparkfun AVC track:
```
roslaunch rr_gazebo macaroni_avc.launch
```
Then, the following command will start our race AI and drive the car around the track:
```
roslaunch rr_avc avc_sim.launch
```
