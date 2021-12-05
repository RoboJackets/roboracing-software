![alt text](https://raw.githubusercontent.com/wiki/RoboJackets/roboracing-software/images/sedanii.jpeg "Photo by Brian Cochran")

Welcome to the ROS2 version of the [RoboJackets](http://robojackets.org) RoboRacing software repository! This document will give you a brief outline of the repository's layout and some simple instructions for setting up the project. For more detailed information, please visit the [wiki](https://wiki.robojackets.org/RoboRacing).

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Nico%20Bartholomai-blue.svg)](https://github.com/NicoBartholomai)

![Project Manager](https://img.shields.io/badge/Project%20Manager-Sam%20Walters-blue.svg)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)


## Documentation

View our documentation at https://example.com

## Organization

This repository is comprised of multiple ROS packages and one sandbox folder for miscellaneous resources.

**rr_description**: This package contains URDFs and meshes that describe the platform's layout.

**rr_evgp**: This package contains mission code for the [Electric Vehicle Grand Prix](https://evgrandprix.org/).

**rr_gazebo**: This package contains resources for running the car in the [Gazebo](http://gazebosim.org) simulator.

**rr_iarrc**: This package contains mission code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_msgs**: This package contains all of the message types used by ROS.

**rr_rviz_plugins**: This package contains some rviz plugins.

**rr_util**: This package contains general code that is used for both evgp and iarrc

**sandbox**: This directory contains miscellaneous code.

**third_party**: This directory contains code from other open-source projects.

## Installation

This repository should be cloned into the src directory of a colcon workspace. Use ```colcon build``` in the workspace directory to build the code. (NOTE: Be sure to ```source devel/setup.bash``` before referencing roboracing packages.)

For a guide on installing our code please go to [our guide](https://wiki.robojackets.org/RoboRacing_Software_Installation_Instructions_ROS2).
