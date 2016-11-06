# RoboRacing Software [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-software.svg?style=svg)](https://circleci.com/gh/RoboJackets/roboracing-software)

![alt text](https://raw.githubusercontent.com/wiki/RoboJackets/roboracing-software/images/RaceCar.JPG "Picture of first RoboRacing car.")

This repository contains [ROS](http://ros.org) packages for the [RoboJackets](http://robojackets.org) RoboRacing team.

## Organization

Each folder in this repository is a ROS package.

**avc**: This package contains intelligence code for the [Sparkfun Autonomous Vehicle Challenge](http://avc.sparkfun.com).

**iarrc**: This package contains intelligence code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_platform**: This package contains code for interfacing with the various cars built and maintained by the team.

**sandbox**: This package contains utilities and non-ROS code. This includes tools for working with log files and the Arduino code for the car.

## Installation

1. Make sure you have the appropriate ROS version installed for your version of Ubuntu.

2. Clone this repo into the _src_ directory of your catkin workspace.

3. Install the dependencies with the following command in your catkin workspace folder:

   ```
   rosdep install --from-path src --ignore-src -y
   ```

4. You should now be able to build the project with `catkin_make`.
